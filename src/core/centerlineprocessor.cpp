#include "centerlineprocessor.h"
#include "centerlineroutes.h"
#include "../utils/loggingcategories.h"

#include <QDebug>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <opencv2/imgproc.hpp>
#include <opencv2/geometry.hpp>

// Frames an end may stay unobserved before its last position stops mattering.
static constexpr int kMaxTipAge = 60;
// Clean-frame length samples needed before the baseline replaces Sweep 0's median.
static constexpr int kMinBodyLengthSamples = 30;

namespace Centerline {

EndpointResult detectEndpoints(const Tracking::DetectedBlob& blob,
                               const HeadTailPredictor& predictor,
                               const TipFeatureBaseline& baseline,
                               bool inMergeGroup,
                               Debug::EndpointDebug* debugOut)
{
    EndpointResult r;
    if (debugOut) *debugOut = Debug::EndpointDebug{};

    if (!blob.isValid) {
        r.topology = Tracking::TopologyState::Lost;
        return r;
    }
    // Merged blobs still get their skeleton + tips harvested when possible —
    // the topology label is set to Merged at the end. centerlineworker treats
    // Merged as a no-op for centerline computation but the tip data is still
    // useful for the renderer.
    if (blob.contourPoints.size() < 8) {
        r.topology = inMergeGroup ? Tracking::TopologyState::Merged
                                  : Tracking::TopologyState::SelfCrossed;
        return r;
    }

    // (a) ── Mask + DT ──────────────────────────────────────────────────────
    cv::Mat mask;
    r.localBounds = buildCenterlineMask(blob, mask);
    if (mask.empty()) {
        r.topology = inMergeGroup ? Tracking::TopologyState::Merged
                                  : Tracking::TopologyState::SelfCrossed;
        return r;
    }
    cv::distanceTransform(mask, r.distTransform, cv::DIST_L2, 3);

    const cv::Point2f originOffset(static_cast<float>(r.localBounds.x),
                                   static_cast<float>(r.localBounds.y));

    // (b) ── Skeleton + adjacency + indexImage ──────────────────────────────
    // Shared with the centerline path so endpoint detection and extraction
    // reason over exactly the same graph representation.
    r.skeleton = Centerline::buildSkeletonGraph(mask);
    if (r.skeleton.points.size() < 2) {
        r.topology = inMergeGroup ? Tracking::TopologyState::Merged
                                  : Tracking::TopologyState::SelfCrossed;
        return r;
    }
    std::vector<int> rawEndpoints = r.skeleton.endpointIndices;
    if (debugOut) debugOut->rawSkeletonEndpointIndices = rawEndpoints;

    // (c) ── Prune to ≤ 2 endpoints (longest-path pair) ─────────────────────
    int rawEndpointCount = static_cast<int>(rawEndpoints.size());
    if (rawEndpoints.size() <= 2) {
        r.skeleton.endpointIndices = rawEndpoints;
    } else {
        int bestA = rawEndpoints[0];
        int bestB = rawEndpoints[1];
        double bestDist = -1.0;
        for (int ep : rawEndpoints) {
            Centerline::GraphSearchResult sr = Centerline::dijkstraSkeleton(
                r.skeleton.points, r.skeleton.adjacency, ep);
            for (int other : rawEndpoints) {
                if (other == ep) continue;
                const double d = sr.distances[other];
                if (std::isfinite(d) && d > bestDist) {
                    bestDist = d;
                    bestA = ep;
                    bestB = other;
                }
            }
        }
        r.skeleton.endpointIndices = {bestA, bestB};
    }

    // (d) ── Outer-contour signed curvature + local maxima ─────────────────
    const std::vector<cv::Point>& contour = blob.contourPoints;
    const int nContour = static_cast<int>(contour.size());
    std::vector<cv::Point2f> contourLocal;
    contourLocal.reserve(nContour);
    for (const cv::Point& p : contour) {
        contourLocal.emplace_back(static_cast<float>(p.x - r.localBounds.x),
                                  static_cast<float>(p.y - r.localBounds.y));
    }

    constexpr int kCurvatureWindow = 5;
    const int k = std::max(2, kCurvatureWindow);
    std::vector<float> curvature(nContour, 0.f);
    for (int i = 0; i < nContour; ++i) {
        const cv::Point2f& a = contourLocal[(i - k + nContour) % nContour];
        const cv::Point2f& b = contourLocal[i];
        const cv::Point2f& c = contourLocal[(i + k) % nContour];
        const cv::Point2f v1 = b - a;
        const cv::Point2f v2 = c - b;
        const float cross = v1.x * v2.y - v1.y * v2.x;
        const float dot   = v1.x * v2.x + v1.y * v2.y;
        const float angle = std::atan2(cross, dot);
        const float arc   = 0.5f * (std::hypot(v1.x, v1.y) + std::hypot(v2.x, v2.y));
        curvature[i] = (arc > 1e-3f) ? (angle / arc) : 0.f;
    }

    // Curvature peak threshold: scaled by baseline if reliable, else default.
    const float curvFloor = baseline.isReliable()
        ? std::max(0.5f * baseline.meanAbsCurvature, 0.04f)
        : 0.08f;

    std::vector<int> curvaturePeakIdx;
    curvaturePeakIdx.reserve(8);
    for (int i = 0; i < nContour; ++i) {
        const float mag = std::abs(curvature[i]);
        if (mag < curvFloor) continue;
        bool isLocalMax = true;
        for (int d = -k; d <= k; ++d) {
            if (d == 0) continue;
            const int j = (i + d + nContour) % nContour;
            if (std::abs(curvature[j]) > mag) { isLocalMax = false; break; }
        }
        if (isLocalMax) curvaturePeakIdx.push_back(i);
    }
    if (debugOut) {
        debugOut->contourCurvatures = curvature;
        debugOut->contourCurvaturePeaks = curvaturePeakIdx;
        debugOut->contourPoints.clear();
        debugOut->contourPoints.reserve(nContour);
        for (const cv::Point& p : contour) {
            debugOut->contourPoints.emplace_back(static_cast<float>(p.x),
                                                 static_cast<float>(p.y));
        }
    }

    // ── Width probe ────────────────────────────────────────────────────────
    auto probeDT = [&](const cv::Point2f& origin, const cv::Point2f& dir, float depth) -> float {
        const int px = std::clamp(static_cast<int>(std::round(origin.x + dir.x * depth)),
                                  0, r.distTransform.cols - 1);
        const int py = std::clamp(static_cast<int>(std::round(origin.y + dir.y * depth)),
                                  0, r.distTransform.rows - 1);
        return r.distTransform.at<float>(py, px);
    };

    auto widthAt = [&](const cv::Point2f& tipLocal, int contourIdx) -> float {
        const int nC = static_cast<int>(contourLocal.size());
        if (nC < 4) return 0.f;
        const int tangentK = 3;
        const cv::Point2f& a = contourLocal[(contourIdx - tangentK + nC) % nC];
        const cv::Point2f& c = contourLocal[(contourIdx + tangentK) % nC];
        cv::Point2f tangent = c - a;
        const float tNorm = std::hypot(tangent.x, tangent.y);
        if (tNorm < 1e-3f) return 0.f;
        tangent *= 1.f / tNorm;
        const cv::Point2f perpA(-tangent.y,  tangent.x);
        const cv::Point2f perpB( tangent.y, -tangent.x);
        const float dtA = probeDT(tipLocal, perpA, 1.5f);
        const float dtB = probeDT(tipLocal, perpB, 1.5f);
        if (dtA < 0.5f && dtB < 0.5f) return 0.f;
        const cv::Point2f inward = (dtA >= dtB) ? perpA : perpB;
        return 2.f * probeDT(tipLocal, inward, 3.f);
    };

    // ── nearest contour idx to a local-coords point ────────────────────────
    auto nearestContourIdx = [&](const cv::Point2f& q) -> int {
        int bestIdx = -1;
        float bestDistSq = std::numeric_limits<float>::max();
        for (int i = 0; i < nContour; ++i) {
            const cv::Point2f d = contourLocal[i] - q;
            const float dsq = d.x * d.x + d.y * d.y;
            if (dsq < bestDistSq) { bestDistSq = dsq; bestIdx = i; }
        }
        return bestIdx;
    };

    auto endpointOutwardDirection = [&](int epIdx) -> cv::Point2f {
        if (epIdx < 0 || epIdx >= static_cast<int>(r.skeleton.points.size()) ||
            r.skeleton.adjacency[epIdx].empty()) {
            return cv::Point2f(0.f, 0.f);
        }

        int cur = r.skeleton.adjacency[epIdx].front();
        // Pixel adjacency can contain small cycles even at an otherwise clean
        // body end. Never walk back to an earlier node (especially epIdx).
        std::vector<int> visited{epIdx, cur};
        cv::Point inner = r.skeleton.points[cur];
        constexpr int kEndpointDirectionSteps = 6;
        for (int step = 1; step < kEndpointDirectionSteps; ++step) {
            int next = -1;
            for (int candidate : r.skeleton.adjacency[cur]) {
                if (std::find(visited.begin(), visited.end(), candidate) == visited.end()) {
                    next = candidate;
                    break;
                }
            }
            if (next < 0) {
                break;
            }
            cur = next;
            visited.push_back(cur);
            inner = r.skeleton.points[cur];
        }

        const cv::Point& ep = r.skeleton.points[epIdx];
        cv::Point2f dir(static_cast<float>(ep.x - inner.x),
                        static_cast<float>(ep.y - inner.y));
        const float norm = std::hypot(dir.x, dir.y);
        if (norm < 1e-3f) {
            return cv::Point2f(0.f, 0.f);
        }
        return dir * (1.f / norm);
    };

    auto endpointSearchLimits = [](float dtAtEp, float& maxForward, float& maxSide) {
        // Degree-1 skeleton endpoints are already close to the contour cap.
        // Keep this local so a tightly curled endpoint cannot search across
        // the body and snap onto the other tip's cap.
        maxForward = std::clamp(1.25f + 2.0f * dtAtEp, 3.0f, 7.0f);
        maxSide = std::clamp(1.0f + 1.5f * dtAtEp, 2.5f, 5.0f);
    };

    auto projectedEndpointContourIdx = [&](const cv::Point2f& epLocalF,
                                           const cv::Point2f& outwardDir,
                                           float dtAtEp) -> int {
        if (std::hypot(outwardDir.x, outwardDir.y) < 1e-3f) {
            return nearestContourIdx(epLocalF);
        }

        float maxForward = 0.f;
        float maxSide = 0.f;
        endpointSearchLimits(dtAtEp, maxForward, maxSide);
        int bestIdx = -1;
        float bestScore = -std::numeric_limits<float>::max();
        for (int i = 0; i < nContour; ++i) {
            const cv::Point2f rel = contourLocal[i] - epLocalF;
            const float dist = std::hypot(rel.x, rel.y);
            const float forward = rel.x * outwardDir.x + rel.y * outwardDir.y;
            if (forward < -1.f || forward > maxForward) {
                continue;
            }
            const float cross = rel.x * outwardDir.y - rel.y * outwardDir.x;
            const float side = std::abs(cross);
            if (side > maxSide) {
                continue;
            }

            const float curvatureBonus = 2.f * std::abs(curvature[i]);
            const float score = 0.5f * forward - 0.75f * dist - 0.25f * side + curvatureBonus;
            if (score > bestScore) {
                bestScore = score;
                bestIdx = i;
            }
        }

        return bestIdx >= 0 ? bestIdx : nearestContourIdx(epLocalF);
    };

    // (e) ── Extend each skeleton endpoint to the strongest reachable peak ──
    auto rawEndpointOrderForGraphIndex = [&](int graphIdx) -> int {
        for (int order = 0; order < static_cast<int>(rawEndpoints.size()); ++order) {
            if (rawEndpoints[order] == graphIdx) {
                return order;
            }
        }
        return -1;
    };

    int prunedEndpointOrder = 0;
    std::vector<int> usedPeakIndices;
    std::vector<cv::Point2f> usedFinalTipPoints;
    auto peakAlreadyUsed = [&](int peakIdx) -> bool {
        return std::find(usedPeakIndices.begin(), usedPeakIndices.end(), peakIdx) !=
               usedPeakIndices.end();
    };
    auto finalPointAlreadyUsed = [&](const cv::Point2f& point) -> bool {
        constexpr float kMinDistinctTipDistance = 1.5f;
        for (const cv::Point2f& used : usedFinalTipPoints) {
            const cv::Point2f delta = point - used;
            if (std::hypot(delta.x, delta.y) < kMinDistinctTipDistance) {
                return true;
            }
        }
        return false;
    };

    const bool cleanVisibleTips = !inMergeGroup && blob.holeContourPoints.empty() &&
                                  r.skeleton.endpointIndices.size() == 2;
    for (int epIdx : r.skeleton.endpointIndices) {
        const cv::Point& epLocal = r.skeleton.points[epIdx];
        const cv::Point2f epLocalF(static_cast<float>(epLocal.x),
                                   static_cast<float>(epLocal.y));
        const cv::Point2f epWorld(epLocalF.x + originOffset.x,
                                  epLocalF.y + originOffset.y);

        const float dtAtEp = r.distTransform.at<float>(epLocal.y, epLocal.x);
        const cv::Point2f outwardDir = endpointOutwardDirection(epIdx);
        float maxForward = 0.f;
        float maxSide = 0.f;
        endpointSearchLimits(dtAtEp, maxForward, maxSide);

        Debug::EndpointCandidateDebug endpointDbg;
        endpointDbg.rawEndpointOrder = rawEndpointOrderForGraphIndex(epIdx);
        endpointDbg.prunedEndpointOrder = prunedEndpointOrder++;
        endpointDbg.graphIndex = epIdx;
        endpointDbg.graphDegree =
            (epIdx >= 0 && epIdx < static_cast<int>(r.skeleton.adjacency.size()))
                ? static_cast<int>(r.skeleton.adjacency[epIdx].size()) : 0;
        endpointDbg.skeletonLocal = epLocalF;
        endpointDbg.skeletonVideo = epWorld;
        endpointDbg.outwardDir = outwardDir;
        endpointDbg.dtAtEndpoint = dtAtEp;
        endpointDbg.maxForward = maxForward;
        endpointDbg.maxSide = maxSide;

        // Skeleton endpoint projected outward to the cap contour. A nearest
        // contour snap often lands on the side of a rounded tip instead of at
        // the end-cap apex.
        const int snapIdx = cleanVisibleTips ? nearestContourIdx(epLocalF)
            : projectedEndpointContourIdx(epLocalF, outwardDir, dtAtEp);
        cv::Point2f snapLocal(0.f, 0.f);
        cv::Point2f snapVideo = epWorld;
        if (snapIdx >= 0) {
            snapLocal = contourLocal[snapIdx];
            snapVideo = cv::Point2f(snapLocal.x + originOffset.x,
                                    snapLocal.y + originOffset.y);
        }
        endpointDbg.snapContourIdx = snapIdx;
        endpointDbg.snapVideo = snapVideo;
        endpointDbg.snapCurvature = (snapIdx >= 0) ? curvature[snapIdx] : 0.f;

        // Find a strong curvature peak in the local end-cap region defined by
        // the skeleton endpoint and its outward direction. This avoids making
        // curvature depend on a possibly side-biased contour snap.
        int bestPeak = -1;
        float bestScore = -std::numeric_limits<float>::max();
        int reachablePeakCount = 0;
        for (int peakIdx : curvaturePeakIdx) {
            if (cleanVisibleTips) break;
            if (peakAlreadyUsed(peakIdx)) {
                continue;
            }
            const cv::Point2f& peakLocal = contourLocal[peakIdx];
            const cv::Point2f rel = peakLocal - epLocalF;
            const float dist = std::hypot(rel.x, rel.y);
            float forward = 0.f;
            float side = dist;
            if (std::hypot(outwardDir.x, outwardDir.y) >= 1e-3f) {
                forward = rel.x * outwardDir.x + rel.y * outwardDir.y;
                const float cross = rel.x * outwardDir.y - rel.y * outwardDir.x;
                side = std::abs(cross);
            }
            if (forward < -1.f || forward > maxForward || side > maxSide) {
                continue;
            }
            ++reachablePeakCount;

            const float score = 0.5f * forward - 0.5f * dist -
                                0.25f * side + 4.f * std::abs(curvature[peakIdx]);
            if (score > bestScore) {
                bestScore = score;
                bestPeak = peakIdx;
            }
        }
        endpointDbg.reachablePeakCount = reachablePeakCount;
        endpointDbg.bestPeakContourIdx = bestPeak;
        endpointDbg.bestPeakScore = bestPeak >= 0 ? bestScore : 0.f;

        const int preShiftBestPeak = bestPeak;
        if (bestPeak >= 0) {
            const cv::Point2f peakDelta = contourLocal[bestPeak] - snapLocal;
            const float peakDist = std::hypot(peakDelta.x, peakDelta.y);
            const float maxPeakShift = std::clamp(1.0f + 0.75f * dtAtEp, 2.0f, 4.0f);
            endpointDbg.bestPeakVideo =
                cv::Point2f(contourLocal[bestPeak].x + originOffset.x,
                            contourLocal[bestPeak].y + originOffset.y);
            endpointDbg.bestPeakCurvature = curvature[bestPeak];
            endpointDbg.bestPeakDistanceFromSnap = peakDist;
            endpointDbg.maxPeakShift = maxPeakShift;
            if (peakDist > maxPeakShift) {
                bestPeak = -1;
                endpointDbg.peakRejectReason =
                    QStringLiteral("peak rejected: distance from contour snap %1 > maxPeakShift %2")
                        .arg(peakDist, 0, 'f', 3)
                        .arg(maxPeakShift, 0, 'f', 3);
            }
        }
        if (cleanVisibleTips) {
            endpointDbg.peakRejectReason = QStringLiteral("not used: clean terminal-axis selection");
        } else if (preShiftBestPeak < 0) {
            endpointDbg.peakRejectReason = reachablePeakCount == 0
                ? QStringLiteral("no curvature peak inside endpoint search window")
                : QStringLiteral("no curvature peak selected");
        } else if (bestPeak >= 0) {
            endpointDbg.peakAccepted = true;
            endpointDbg.peakRejectReason = QStringLiteral("accepted");
        }

        if (bestPeak >= 0) {
            const cv::Point2f peakWorld(contourLocal[bestPeak].x + originOffset.x,
                                        contourLocal[bestPeak].y + originOffset.y);
            if (finalPointAlreadyUsed(peakWorld)) {
                endpointDbg.peakAccepted = false;
                endpointDbg.peakRejectReason =
                    QStringLiteral("peak rejected: final tip would duplicate a previous endpoint");
                bestPeak = -1;
            }
        }

        Debug::TipCapDebug capDbg;
        capDbg.valid = true;
        capDbg.skelEndpoint = epWorld;
        capDbg.outwardDir = outwardDir;
        capDbg.dtAtEp = dtAtEp;
        capDbg.snapPoint = snapVideo;
        capDbg.peakOrSnapPoint = bestPeak >= 0 ? contourLocal[bestPeak] + originOffset : snapVideo;
        capDbg.hadPeak = bestPeak >= 0;
        capDbg.selectedEstimator = bestPeak >= 0 ? QStringLiteral("curvature peak") : QStringLiteral("contour snap");
        capDbg.selectionReason = QStringLiteral("non-clean topology; retained peak/snap routing");

        TrueTip t;
        t.skelPoint = snapVideo;
        if (bestPeak >= 0) {
            const cv::Point2f peakLocal = contourLocal[bestPeak];
            t.point     = cv::Point2f(peakLocal.x + originOffset.x,
                                      peakLocal.y + originOffset.y);
            t.curvature = curvature[bestPeak];
            t.width     = widthAt(peakLocal, bestPeak);
            t.extended  = true;
        } else {
            t.point     = snapVideo;
            t.curvature = (snapIdx >= 0) ? curvature[snapIdx] : 0.f;
            t.width     = (snapIdx >= 0) ? widthAt(snapLocal, snapIdx) : 0.f;
            t.extended  = false;
        }
        if (cleanVisibleTips) {
            // Fit the terminal axis to six uniformly spaced interior samples
            // on the endpoint-to-endpoint shortest path. Contour vertices do
            // not participate in the fit or vote for the tip position.
            const int other = r.skeleton.endpointIndices[0] == epIdx
                ? r.skeleton.endpointIndices[1] : r.skeleton.endpointIndices[0];
            const auto search = dijkstraSkeleton(r.skeleton.points, r.skeleton.adjacency, epIdx);
            std::vector<cv::Point2f> path;
            if (std::isfinite(search.distances[other])) {
                for (int node = other; node >= 0; node = search.parents[node]) {
                    path.push_back(cv::Point2f(r.skeleton.points[node]) + originOffset);
                    if (node == epIdx) break;
                }
                std::reverse(path.begin(), path.end());
            }
            float walked = 0.f;
            float nextSample = 1.f;
            for (size_t j = 1; j < path.size() && nextSample <= 6.f; ++j) {
                const float length = cv::norm(path[j] - path[j - 1]);
                while (nextSample <= 6.f && nextSample <= walked + length) {
                    capDbg.axisSamples.push_back(path[j - 1] +
                        (path[j] - path[j - 1]) * ((nextSample - walked) / length));
                    nextSample += 1.f;
                }
                walked += length;
            }
            cv::Point2f direction(0.f, 0.f), origin(0.f, 0.f);
            const int count = static_cast<int>(capDbg.axisSamples.size());
            if (count >= 3) {
                for (const auto& sample : capDbg.axisSamples) origin += sample;
                origin *= 1.f / count;
                const float meanIndex = 0.5f * (count - 1);
                for (int j = 0; j < count; ++j)
                    direction += (meanIndex - j) * (capDbg.axisSamples[j] - origin);
            }
            const float norm = cv::norm(direction);
            capDbg.hasAxis = std::isfinite(norm) && norm > 1e-3f &&
                cv::pointPolygonTest(contour, origin, false) > 0;
            if (capDbg.hasAxis) {
                direction *= 1.f / norm;
                capDbg.axisOrigin = origin;
                capDbg.outwardDir = direction;
            }
            // Intersect the ray with contour SEGMENTS, not sampled vertices.
            // The first forward exit from this interior origin is the cap.
            float nearest = std::numeric_limits<float>::infinity();
            auto cross = [](cv::Point2f a, cv::Point2f b) { return a.x*b.y - a.y*b.x; };
            if (capDbg.hasAxis) {
                for (int j = 0; j < nContour; ++j) {
                    const cv::Point2f a = cv::Point2f(contour[j]);
                    const cv::Point2f edge = cv::Point2f(contour[(j + 1) % nContour]) - a;
                    const float denominator = cross(direction, edge);
                    if (std::abs(denominator) < 1e-6f) continue;
                    const float distance = cross(a - origin, edge) / denominator;
                    const float fraction = cross(a - origin, direction) / denominator;
                    if (distance >= 0.f && fraction >= -1e-5f && fraction <= 1.f + 1e-5f)
                        nearest = std::min(nearest, distance);
                }
            }
            // Keep a degenerate fit local; never route a ray across the body.
            const float limit = cv::norm(origin - epWorld) + maxForward;
            t.selectedAxis = capDbg.hasAxis && std::isfinite(nearest) && nearest <= limit;
            if (t.selectedAxis) {
                t.point = origin + direction * nearest;
                capDbg.selectedEstimator = QStringLiteral("axis boundary");
                capDbg.selectionReason = QStringLiteral("first contour exit along fitted terminal axis");
            } else {
                // Geometric fallback: closest point on a segment, no curvature score.
                float best = std::numeric_limits<float>::infinity();
                for (int j = 0; j < nContour; ++j) {
                    const cv::Point2f a = cv::Point2f(contour[j]);
                    const cv::Point2f edge = cv::Point2f(contour[(j + 1) % nContour]) - a;
                    const float lengthSq = edge.dot(edge);
                    if (lengthSq <= 0.f) continue;
                    const float fraction = std::clamp((epWorld-a).dot(edge)/lengthSq, 0.f, 1.f);
                    const cv::Point2f candidate = a + edge * fraction;
                    const float distance = cv::norm(candidate - epWorld);
                    if (distance < best) { best = distance; t.point = candidate; }
                }
                capDbg.selectedEstimator = QStringLiteral("nearest boundary");
                capDbg.selectionReason = QStringLiteral("terminal axis unavailable or no local forward exit");
            }
            // Features remain contour-derived, but never select the clean tip.
            const int featureIdx = nearestContourIdx(t.point - originOffset);
            t.curvature = curvature[featureIdx];
            t.width = widthAt(contourLocal[featureIdx], featureIdx);
            t.extended = false;
        }
        capDbg.selectedPoint = t.point;
        if (debugOut) debugOut->tipCapDebug.push_back(capDbg);
        r.tips.push_back(t);
        endpointDbg.finalTipIdx = static_cast<int>(r.tips.size()) - 1;
        endpointDbg.finalTipVideo = t.point;
        endpointDbg.finalExtended = t.extended;
        endpointDbg.finalCurvature = t.curvature;
        endpointDbg.finalWidth = t.width;
        endpointDbg.finalHasAxis = t.selectedAxis;
        if (debugOut) debugOut->endpointCandidateDebug.push_back(endpointDbg);
        if (t.extended) {
            usedPeakIndices.push_back(bestPeak);
        }
        usedFinalTipPoints.push_back(t.point);
    }

    // (f) ── Topology classification ───────────────────────────────────────
    if (inMergeGroup) {
        r.topology = Tracking::TopologyState::Merged;
    } else {
        const bool hasRing = !blob.holeContourPoints.empty();
        r.topology = (hasRing || r.tips.size() < 2)
                         ? Tracking::TopologyState::SelfCrossed
                         : Tracking::TopologyState::Clean;
    }

    // (g) ── Head/tail assignment ──────────────────────────────────────────
    // Each role is predicted from its last position, with a spread that grows
    // with the number of frames since it was actually observed, so a stale or
    // hypothesised position carries less weight than a fresh observation.
    // Predictor-less (keyframe) calls return (-1, -1); the caller bootstraps.
    if (!r.tips.empty() && predictor.hasPrev) {
        const float bodyLength = 2.f * predictor.refDistance;
        const Centerline::RolePrediction headPred = Centerline::predictRole(
            predictor.headKnown, predictor.lastHeadPos, predictor.velHead, predictor.headAge);
        const Centerline::RolePrediction tailPred = Centerline::predictRole(
            predictor.tailKnown, predictor.lastTailPos, predictor.velTail, predictor.tailAge);
        auto cost = [&](const cv::Point2f& p, const Centerline::RolePrediction& rp) {
            return Centerline::roleCost(p, rp, bodyLength);
        };

        if (r.tips.size() == 1) {
            // One visible tip; the other role stays -1 for route selection.
            if (cost(r.tips[0].point, headPred) <= cost(r.tips[0].point, tailPred)) r.headIdx = 0;
            else r.tailIdx = 0;
        } else {
            const float c01 = cost(r.tips[0].point, headPred) + cost(r.tips[1].point, tailPred);
            const float c10 = cost(r.tips[1].point, headPred) + cost(r.tips[0].point, tailPred);
            r.headIdx = c01 <= c10 ? 0 : 1;
            r.tailIdx = c01 <= c10 ? 1 : 0;
        }
    }

    // ── Debug log ───────────────────────────────────────────────────────────
    if (lcDataCommon().isDebugEnabled()) {
        YAWT_DEBUG(lcDataCommon) << QString::asprintf(
            "detectEndpoints: rawEnds=%d  prunedEnds=%d  tips=%d  peaks=%d  "
            "topo=%s  head=%d  tail=%d",
            rawEndpointCount,
            static_cast<int>(r.skeleton.endpointIndices.size()),
            static_cast<int>(r.tips.size()),
            static_cast<int>(curvaturePeakIdx.size()),
            qUtf8Printable(Tracking::topologyStateToString(r.topology)),
            r.headIdx, r.tailIdx);
        for (size_t i = 0; i < r.tips.size(); ++i) {
            const TrueTip& t = r.tips[i];
            YAWT_DEBUG(lcDataCommon) << QString::asprintf(
                "  tip[%zu] %s  point=(%.1f,%.1f)  snap=(%.1f,%.1f)  "
                "k=%+.4f  w=%.2f",
                i, t.selectedAxis ? "axis" : t.extended ? "peak" : "snap",
                static_cast<double>(t.point.x),
                static_cast<double>(t.point.y),
                static_cast<double>(t.skelPoint.x),
                static_cast<double>(t.skelPoint.y),
                static_cast<double>(t.curvature),
                static_cast<double>(t.width));
        }
    }

    return r;
}

} // namespace Centerline

// ── geometry helpers ────────────────────────────────────────────────────────

// Return Euclidean distance between two image-space points.
static float ptDist(const cv::Point2f& a, const cv::Point2f& b)
{
    float dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Measure the total length of an ordered polyline.
static float arcLen(const std::vector<cv::Point2f>& pts)
{
    float len = 0.f;
    for (size_t i = 1; i < pts.size(); ++i)
        len += ptDist(pts[i - 1], pts[i]);
    return len;
}

// Find the contour vertex closest to a target point.
static int nearestContourIdx(const std::vector<cv::Point>& contour,
                             const cv::Point2f& target)
{
    int best = 0;
    float bestD = std::numeric_limits<float>::max();
    for (int i = 0; i < static_cast<int>(contour.size()); ++i) {
        float dx = contour[i].x - target.x;
        float dy = contour[i].y - target.y;
        float d  = dx * dx + dy * dy;
        if (d < bestD) { bestD = d; best = i; }
    }
    return best;
}

// Return the contour point closest to a target point.
static cv::Point2f nearestContourPoint(const std::vector<cv::Point>& contour,
                                       const cv::Point2f& target)
{
    const int idx = nearestContourIdx(contour, target);
    return cv::Point2f(static_cast<float>(contour[idx].x),
                       static_cast<float>(contour[idx].y));
}

// Resample an ordered polyline to a fixed number of evenly spaced points.
static std::vector<cv::Point2f> resample(const std::vector<cv::Point2f>& pts, int nPoints)
{
    if (static_cast<int>(pts.size()) <= 1 || nPoints < 2) return pts;
    std::vector<float> cum(pts.size(), 0.f);
    for (size_t i = 1; i < pts.size(); ++i)
        cum[i] = cum[i - 1] + ptDist(pts[i - 1], pts[i]);
    float total = cum.back();
    if (total < 1e-6f) return pts;

    std::vector<cv::Point2f> out(nPoints);
    out.front() = pts.front();
    out.back()  = pts.back();
    for (int k = 1; k < nPoints - 1; ++k) {
        float target = total * k / (nPoints - 1);
        auto it  = std::lower_bound(cum.begin(), cum.end(), target);
        size_t j = std::min<size_t>(std::distance(cum.begin(), it), pts.size() - 1);
        if (j == 0) { out[k] = pts.front(); continue; }
        float t = (target - cum[j - 1]) / (cum[j] - cum[j - 1] + 1e-9f);
        out[k]  = pts[j - 1] + t * (pts[j] - pts[j - 1]);
    }
    return out;
}

// Sum of consecutive 2D segment cross products along a polyline:
//   vi = p[i+1] - p[i]
//   sum += vi.x * v(i+1).y - v(i+1).x * vi.y
// The sign captures clockwise/counterclockwise bend sense and flips when the
// traversal direction is reversed. Returns 0 for fewer than 3 points.
static float centerlineCrossSum(const std::vector<cv::Point2f>& pts)
{
    float total = 0.f;
    const int n = static_cast<int>(pts.size());
    for (int i = 1; i < n - 1; ++i) {
        const cv::Point2f v1 = pts[i]     - pts[i - 1];
        const cv::Point2f v2 = pts[i + 1] - pts[i];
        total += v1.x * v2.y - v2.x * v1.y;
    }
    return total;
}

// Convert a blob's QPointF centroid into OpenCV point coordinates.
static cv::Point2f blobCentroid(const Tracking::DetectedBlob& blob)
{
    return cv::Point2f(static_cast<float>(blob.centroid.x()),
                       static_cast<float>(blob.centroid.y()));
}

// Rasterize a blob's outer contour and holes into a local binary mask.
static void fillBlobMask(cv::Mat& mask,
                         const Tracking::DetectedBlob& blob,
                         const cv::Rect& bounds,
                         const cv::Point2f& offset)
{
    std::vector<std::vector<cv::Point>> outerContours(1);
    outerContours.front().reserve(blob.contourPoints.size());
    for (const cv::Point& pt : blob.contourPoints) {
        outerContours.front().push_back(cv::Point(
            static_cast<int>(std::lround(static_cast<float>(pt.x) + offset.x - bounds.x)),
            static_cast<int>(std::lround(static_cast<float>(pt.y) + offset.y - bounds.y))));
    }
    cv::fillPoly(mask, outerContours, cv::Scalar(255));

    for (const std::vector<cv::Point>& hole : blob.holeContourPoints) {
        std::vector<std::vector<cv::Point>> holeContour(1);
        holeContour.front().reserve(hole.size());
        for (const cv::Point& pt : hole) {
            holeContour.front().push_back(cv::Point(
                static_cast<int>(std::lround(static_cast<float>(pt.x) + offset.x - bounds.x)),
                static_cast<int>(std::lround(static_cast<float>(pt.y) + offset.y - bounds.y))));
        }
        cv::fillPoly(mask, holeContour, cv::Scalar(0));
    }
}

// Reassign two tip candidates by comparing them with previous centerline order.
static bool enforceTwoTipCenterlineOrderRoles(
    Tracking::DetectedBlob& blob,
    const std::vector<cv::Point2f>& previousCenterline,
    const cv::Point2f& previousCentroid,
    QStringList* diagnostics)
{
    if (blob.centerline.tipCandidates.size() != 2 || previousCenterline.size() < 2) {
        return false;
    }

    const cv::Point2f delta = blobCentroid(blob) - previousCentroid;
    std::vector<cv::Point2f> translatedPrev;
    translatedPrev.reserve(previousCenterline.size());
    for (const cv::Point2f& p : previousCenterline) {
        translatedPrev.push_back(p + delta);
    }

    std::vector<float> cumulative(translatedPrev.size(), 0.f);
    for (int i = 1; i < static_cast<int>(translatedPrev.size()); ++i) {
        cumulative[i] = cumulative[i - 1] + ptDist(translatedPrev[i - 1], translatedPrev[i]);
    }
    const float total = cumulative.back();
    if (total <= 1e-6f) {
        return false;
    }

    struct Projection {
        float fraction = 0.f;
        float distSq = std::numeric_limits<float>::max();
    };

    auto projectOntoPreviousOrder = [&](const cv::Point2f& q) -> Projection {
        Projection best;
        for (int i = 1; i < static_cast<int>(translatedPrev.size()); ++i) {
            const cv::Point2f a = translatedPrev[i - 1];
            const cv::Point2f b = translatedPrev[i];
            const cv::Point2f ab = b - a;
            const float lenSq = ab.x * ab.x + ab.y * ab.y;
            float t = 0.f;
            if (lenSq > 1e-6f) {
                const cv::Point2f aq = q - a;
                t = std::clamp((aq.x * ab.x + aq.y * ab.y) / lenSq, 0.f, 1.f);
            }
            const cv::Point2f proj = a + t * ab;
            const cv::Point2f d = q - proj;
            const float distSq = d.x * d.x + d.y * d.y;
            if (distSq < best.distSq) {
                best.distSq = distSq;
                const float along = cumulative[i - 1] +
                                    t * (cumulative[i] - cumulative[i - 1]);
                best.fraction = along / total;
            }
        }
        return best;
    };

    const Projection p0 = projectOntoPreviousOrder(blob.centerline.tipCandidates[0].point);
    const Projection p1 = projectOntoPreviousOrder(blob.centerline.tipCandidates[1].point);
    const float cost01 = p0.fraction * p0.fraction +
                         (1.f - p1.fraction) * (1.f - p1.fraction);
    const float cost10 = p1.fraction * p1.fraction +
                         (1.f - p0.fraction) * (1.f - p0.fraction);
    const int oldHead = blob.centerline.headTipIdx;
    const int oldTail = blob.centerline.tailTipIdx;

    if (cost01 <= cost10) {
        blob.centerline.headTipIdx = 0;
        blob.centerline.tailTipIdx = 1;
    } else {
        blob.centerline.headTipIdx = 1;
        blob.centerline.tailTipIdx = 0;
    }

    const bool changed =
        oldHead != blob.centerline.headTipIdx ||
        oldTail != blob.centerline.tailTipIdx;
    if (diagnostics) {
        diagnostics->append(
            QStringLiteral("two-tip centerline-order role check s0=%1 d0=%2 s1=%3 d1=%4 cost01=%5 cost10=%6 selected headIdx=%7 tailIdx=%8%9")
                .arg(p0.fraction, 0, 'f', 3)
                .arg(std::sqrt(p0.distSq), 0, 'f', 2)
                .arg(p1.fraction, 0, 'f', 3)
                .arg(std::sqrt(p1.distSq), 0, 'f', 2)
                .arg(cost01, 0, 'f', 4)
                .arg(cost10, 0, 'f', 4)
                .arg(blob.centerline.headTipIdx)
                .arg(blob.centerline.tailTipIdx)
                .arg(changed ? QStringLiteral(" reassigned") : QString()));
    }
    return changed;
}

// ── Skeleton-graph shortest path (Clean centerline branch) ────────────────
//
// Run Dijkstra on a prepared skeleton graph and return a video-coordinate
// start-to-goal path for the clean centerline branch.
static bool skeletonGraphPath(const Centerline::SkeletonGraph& graph,
                              int startIdx, int goalIdx,
                              const cv::Point2f& originOffset,
                              std::vector<cv::Point2f>& outPath)
{
    outPath.clear();
    if (startIdx < 0 || goalIdx < 0 ||
        startIdx >= static_cast<int>(graph.points.size()) ||
        goalIdx  >= static_cast<int>(graph.points.size())) return false;
    if (startIdx == goalIdx) return false;

    const Centerline::GraphSearchResult search =
        Centerline::dijkstraSkeleton(graph.points, graph.adjacency, startIdx);
    if (!std::isfinite(search.distances[goalIdx])) return false;

    outPath = Centerline::reconstructCenterlinePath(graph.points, search.parents,
                                                    startIdx, goalIdx, originOffset);
    return outPath.size() >= 2;
}

// Build a copy of a blob with a synthetic circular hole punched at the
// distance-transform maximum.  Used when a Clean-topology D-1 path is
// suspiciously short (worm tightly self-coiled but no hole visible in the
// mask yet).  The hole radius is 80% of the DT value at
// the peak (≈ the local body half-width).
static bool addSyntheticHoleAtDTMax(const Tracking::DetectedBlob& src,
                                     const cv::Mat& distTransform,
                                     const cv::Rect& localBounds,
                                     Tracking::DetectedBlob& out)
{
    if (distTransform.empty()) return false;
    double maxVal = 0.0;
    cv::Point maxLoc;
    cv::minMaxLoc(distTransform, nullptr, &maxVal, nullptr, &maxLoc);
    const float radius = static_cast<float>(maxVal * 0.8);
    if (radius < 3.f) return false;

    const float cx = static_cast<float>(maxLoc.x + localBounds.x);
    const float cy = static_cast<float>(maxLoc.y + localBounds.y);
    const int nPts = std::max(8, static_cast<int>(2.f * float(CV_PI) * radius));

    out = src;
    std::vector<cv::Point> hole;
    hole.reserve(nPts);
    for (int i = 0; i < nPts; ++i) {
        const float ang = 2.f * float(CV_PI) * i / nPts;
        hole.push_back({static_cast<int>(std::lround(cx + radius * std::cos(ang))),
                        static_cast<int>(std::lround(cy + radius * std::sin(ang)))});
    }
    out.holeContourPoints.push_back(std::move(hole));
    return true;
}

// ── Active-contour ("snake") refinement ─────────────────────────────────────
//
// For ring/coiled frames, skeletonization can never produce a self-intersecting
// centerline because the skeleton of a planar mask is a planar tree. When a
// worm physically crosses over itself, the true 2D centerline IS self-intersecting.
//
// This helper evolves a parametric polyline (which has no topological constraint
// against self-intersection) under three forces:
//   - tension  (alpha) → discrete Laplacian: V[i-1] - 2V[i] + V[i+1]
//   - rigidity (beta)  → discrete biharmonic: -(V[i-2] - 4V[i-1] + 6V[i] - 4V[i+1] + V[i+2])
//   - image    (lambda)→ ∇D where D is the distance transform of the blob mask.
//                        D's ridge IS the medial axis, so following ∇D pulls the
//                        snake onto the body axis.
//
// Snake core takes a pre-built mask, an explicit init polyline, and explicit
// pinned head/tail positions. The active pipeline uses it to lightly refine
// Clean-frame skeleton paths while keeping the DT, gradient, Euler loop, and
// overlap detection in one place.
//
// `v` is mutated in place: caller passes the init polyline; on success it
// contains the refined, nPoint-resampled centerline.
static bool refineSnakeCore(const Tracking::DetectedBlob& blob,
                            const cv::Mat& mask,
                            const cv::Rect& bounds,
                            std::vector<cv::Point2f>& v,
                            const cv::Point2f& pinHead,
                            const cv::Point2f& pinTail,
                            int nPoints,
                            const Centerline::CenterlineSnakeParams& params,
                            cv::Point2f& outOverlapCenter,
                            bool& outHasOverlap,
                            const cv::Point2f* midpointTarget = nullptr)
{
    outHasOverlap = false;
    if (mask.empty() || v.empty() || blob.contourPoints.empty()) return false;

    // Resample to canonical nPoint count.
    if (static_cast<int>(v.size()) != nPoints) v = resample(v, nPoints);
    if (static_cast<int>(v.size()) < 4) return false;

    // Distance transform → its ridges ARE the medial axis. Smooth a little so
    // ∇D is well-defined off-ridge. Sobel gives the gradient field that
    // attracts the snake toward the ridge (highest-D pixels).
    cv::Mat dt;
    cv::distanceTransform(mask, dt, cv::DIST_L2, 3);
    cv::GaussianBlur(dt, dt, cv::Size(0, 0), 1.0);
    cv::Mat gx, gy;
    cv::Sobel(dt, gx, CV_32F, 1, 0, 3);
    cv::Sobel(dt, gy, CV_32F, 0, 1, 3);

    // Normalize gradient magnitude scale so lambda has roughly mask-size-
    // independent meaning.
    double gxMin = 0.0, gxMax = 0.0, gyMin = 0.0, gyMax = 0.0;
    cv::minMaxLoc(gx, &gxMin, &gxMax);
    cv::minMaxLoc(gy, &gyMin, &gyMax);
    const double gradScale = std::max({std::abs(gxMin), std::abs(gxMax),
                                       std::abs(gyMin), std::abs(gyMax), 1.0});
    gx /= static_cast<float>(gradScale);
    gy /= static_cast<float>(gradScale);

    auto sampleGradient = [&](const cv::Point2f& video) -> cv::Point2f {
        const int lx = std::clamp(static_cast<int>(std::lround(video.x - bounds.x)),
                                  0, gx.cols - 1);
        const int ly = std::clamp(static_cast<int>(std::lround(video.y - bounds.y)),
                                  0, gx.rows - 1);
        return cv::Point2f(gx.at<float>(ly, lx), gy.at<float>(ly, lx));
    };
    auto isInsideMask = [&](const cv::Point2f& video) -> bool {
        const int lx = static_cast<int>(std::lround(video.x - bounds.x));
        const int ly = static_cast<int>(std::lround(video.y - bounds.y));
        if (lx < 0 || ly < 0 || lx >= mask.cols || ly >= mask.rows) return false;
        return mask.at<uchar>(ly, lx) != 0;
    };

    // Pin endpoints. Caller supplies the positions (e.g. assigned head/tail
    // tip points, or prev-frame endpoints snapped to current contour).
    v.front() = pinHead;
    v.back()  = pinTail;

    // Explicit-Euler gradient descent on the discretized energy.
    const float alpha  = static_cast<float>(params.alpha);
    const float beta   = static_cast<float>(params.beta);
    const float lambda = static_cast<float>(params.lambda);
    const float tau    = static_cast<float>(std::max(1e-3, params.stepSize));
    const int n = static_cast<int>(v.size());

    std::vector<cv::Point2f> next(v.size());
    for (int iter = 0; iter < std::max(1, params.iterations); ++iter) {
        next.front() = v.front();
        next.back()  = v.back();

        for (int i = 1; i < n - 1; ++i) {
            const cv::Point2f tens = v[i - 1] - 2.f * v[i] + v[i + 1];
            cv::Point2f rig(0.f, 0.f);
            if (i >= 2 && i <= n - 3) {
                rig = v[i - 2] - 4.f * v[i - 1] + 6.f * v[i] - 4.f * v[i + 1] + v[i + 2];
            }
            const cv::Point2f img = sampleGradient(v[i]);
            cv::Point2f force = alpha * tens - beta * rig + lambda * img;
            if (midpointTarget && i == n / 2) {
                // Keep the trace midpoint near its temporally smoothed position
                // while the image force still draws it toward the medial axis.
                constexpr float kMidpointGuideWeight = 1.0f;
                force += kMidpointGuideWeight * (*midpointTarget - v[i]);
            }
            cv::Point2f candidate = v[i] + tau * force;
            if (!isInsideMask(candidate)) {
                candidate = nearestContourPoint(blob.contourPoints, candidate);
            }
            next[i] = candidate;
        }
        v.swap(next);
    }

    if (static_cast<int>(v.size()) != nPoints) v = resample(v, nPoints);
    if (v.size() < 2) return false;

    // Self-intersection detection for debug overlay.
    auto segmentsIntersect = [](const cv::Point2f& a, const cv::Point2f& b,
                                const cv::Point2f& c, const cv::Point2f& d,
                                cv::Point2f& crossing) -> bool {
        const cv::Point2f r = b - a;
        const cv::Point2f s = d - c;
        const float denom = r.x * s.y - r.y * s.x;
        if (std::abs(denom) < 1e-6f) return false;
        const float t = ((c.x - a.x) * s.y - (c.y - a.y) * s.x) / denom;
        const float u = ((c.x - a.x) * r.y - (c.y - a.y) * r.x) / denom;
        if (t > 0.f && t < 1.f && u > 0.f && u < 1.f) {
            crossing = a + t * r;
            return true;
        }
        return false;
    };
    for (int i = 0; i < static_cast<int>(v.size()) - 1 && !outHasOverlap; ++i) {
        for (int j = i + 2; j < static_cast<int>(v.size()) - 1 && !outHasOverlap; ++j) {
            cv::Point2f x;
            if (segmentsIntersect(v[i], v[i + 1], v[j], v[j + 1], x)) {
                outOverlapCenter = x;
                outHasOverlap = true;
            }
        }
    }
    return true;
}

// Helper: build the local mask + padded bounds for a blob.
static bool buildSnakeMask(const Tracking::DetectedBlob& blob,
                           cv::Mat& outMask, cv::Rect& outBounds)
{
    if (blob.contourPoints.empty()) return false;
    cv::Rect bounds = cv::boundingRect(blob.contourPoints);
    constexpr int kPad = 8;
    bounds.x -= kPad;
    bounds.y -= kPad;
    bounds.width += 2 * kPad;
    bounds.height += 2 * kPad;
    if (bounds.width <= 1 || bounds.height <= 1) return false;
    cv::Mat mask = cv::Mat::zeros(bounds.height, bounds.width, CV_8UC1);
    fillBlobMask(mask, blob, bounds, cv::Point2f(0.f, 0.f));
    if (cv::countNonZero(mask) < 4) return false;
    outMask = mask;
    outBounds = bounds;
    return true;
}

// Copy endpoint-detection internals into a frame debug record for export.
static void captureEndpointDebug(const Centerline::EndpointResult& er,
                                 const Debug::EndpointDebug& dbg,
                                 Debug::CenterlineFrameDebug& record)
{
    record.endpointLocalBounds = er.localBounds;
    record.skeletonPixels.clear();
    record.rawSkeletonEndpointPoints.clear();
    record.rawSkeletonEndpointGraphIndices = dbg.rawSkeletonEndpointIndices;
    record.prunedSkeletonEndpointGraphIndices = er.skeleton.endpointIndices;
    record.skeletonEndpointPoints.clear();
    record.contourCurvaturePoints = dbg.contourPoints;
    record.contourCurvatures = dbg.contourCurvatures;
    record.contourCurvaturePeaks = dbg.contourCurvaturePeaks;
    record.endpointCandidateDebug = dbg.endpointCandidateDebug;

    const cv::Point2f origin(static_cast<float>(er.localBounds.x),
                             static_cast<float>(er.localBounds.y));
    if (!er.skeleton.skeleton.empty()) {
        for (int y = 0; y < er.skeleton.skeleton.rows; ++y) {
            const uchar* row = er.skeleton.skeleton.ptr<uchar>(y);
            for (int x = 0; x < er.skeleton.skeleton.cols; ++x) {
                if (!row[x]) {
                    continue;
                }
                record.skeletonPixels.push_back(
                    cv::Point2f(static_cast<float>(x) + origin.x,
                                static_cast<float>(y) + origin.y));
            }
        }
    }

    for (int idx : dbg.rawSkeletonEndpointIndices) {
        if (idx < 0 || idx >= static_cast<int>(er.skeleton.points.size())) {
            continue;
        }
        const cv::Point& point = er.skeleton.points[idx];
        record.rawSkeletonEndpointPoints.push_back(
            cv::Point2f(static_cast<float>(point.x) + origin.x,
                        static_cast<float>(point.y) + origin.y));
    }

    for (int idx : er.skeleton.endpointIndices) {
        if (idx < 0 || idx >= static_cast<int>(er.skeleton.points.size())) {
            continue;
        }
        const cv::Point& point = er.skeleton.points[idx];
        record.skeletonEndpointPoints.push_back(
            cv::Point2f(static_cast<float>(point.x) + origin.x,
                        static_cast<float>(point.y) + origin.y));
    }

    record.distanceTransform = Debug::DistanceTransformDebug{};
    record.distanceTransform.localBounds = er.localBounds;
    if (!er.distTransform.empty() && er.distTransform.type() == CV_32F) {
        record.distanceTransform.rows = er.distTransform.rows;
        record.distanceTransform.cols = er.distTransform.cols;
        record.distanceTransform.values.reserve(
            static_cast<size_t>(er.distTransform.rows * er.distTransform.cols));
        for (int y = 0; y < er.distTransform.rows; ++y) {
            const float* row = er.distTransform.ptr<float>(y);
            for (int x = 0; x < er.distTransform.cols; ++x) {
                record.distanceTransform.values.push_back(row[x]);
            }
        }
    }
}

namespace Centerline {

// Public wrapper for the processor's internal polyline length helper.
float arcLength(const std::vector<cv::Point2f>& points)
{
    return arcLen(points);
}

float resampledArcLength(const std::vector<cv::Point2f>& points, int nPoints)
{
    return arcLen(resample(points, nPoints));
}

// Run one frame of the centerline pipeline and update predictor/previous-frame state.
CenterlineFrameResult processFrame(const CenterlineFrameContext& ctx,
                                   const CenterlineFrameRequest& req,
                                   CenterlineSweepState& state,
                                   CenterlineFrameIo& io)
{
    CenterlineFrameResult result;
    if (!ctx.sortedPoints) {
        return result;
    }

    auto& predictor = state.predictor;
    auto& prevState = state.prevState;
    const Tracking::Track& points = *ctx.sortedPoints;
    const int i = req.pointIndex;

    auto nearestCandidateIdx = [](const Tracking::DetectedBlob& b,
                                  const cv::Point2f& target) -> int {
        int bestIdx = -1;
        float bestDistSq = std::numeric_limits<float>::max();
        for (size_t idx = 0; idx < b.centerline.tipCandidates.size(); ++idx) {
            const cv::Point2f d = b.centerline.tipCandidates[idx].point - target;
            const float dsq = d.x * d.x + d.y * d.y;
            if (dsq < bestDistSq) { bestDistSq = dsq; bestIdx = static_cast<int>(idx); }
        }
        return bestIdx;
    };

    // Rebuild the predictor from stored frames in sweep order. Each end keeps
    // its most recent position (observed or hypothesised) plus the number of
    // frames since it was last actually observed; velocity is carried only for
    // an end observed in both of the two preceding frames.
    auto loadPreviousFrameContext =
        [&](int frameNumber, int frameStep,
            Centerline::HeadTailPredictor& outPredictor,
            CenterlineState& outPrevState) -> bool {
        auto blobAt = [&](int f, Tracking::DetectedBlob& out) -> bool {
            const QMap<int, Tracking::DetectedBlob> blobs = io.getDetectedBlobsForFrame(f);
            if (!blobs.contains(ctx.wormId)) return false;
            out = blobs[ctx.wormId];
            return out.isValid && !out.contourPoints.empty();
        };
        struct RoleSample { bool present = false; bool observed = false; cv::Point2f point; };
        auto roleSample = [](const Tracking::DetectedBlob& b, int idx) {
            RoleSample r;
            if (idx < 0 || idx >= static_cast<int>(b.centerline.tipCandidates.size())) return r;
            const auto& tc = b.centerline.tipCandidates[idx];
            r.present = true;
            r.observed = tc.source != Tracking::TipCandidate::Source::HypothesizedHidden;
            r.point = tc.point;
            return r;
        };

        Tracking::DetectedBlob prevBlob;
        if (!blobAt(frameNumber - frameStep, prevBlob)) {
            return false;
        }

        outPredictor = Centerline::HeadTailPredictor{};
        struct RoleTrack {
            bool known = false; cv::Point2f estimate; int age = kMaxTipAge;
            bool foundObserved = false; RoleSample first, second;
        } head, tail;
        constexpr int kLookback = kMaxTipAge + 1;
        for (int k = 1; k <= kLookback; ++k) {
            Tracking::DetectedBlob b;
            if (k == 1) b = prevBlob;
            else if (!blobAt(frameNumber - k * frameStep, b)) continue;
            for (auto [track, idx] : {std::pair<RoleTrack*, int>{&head, b.centerline.headTipIdx},
                                      std::pair<RoleTrack*, int>{&tail, b.centerline.tailTipIdx}}) {
                const RoleSample sample = roleSample(b, idx);
                if (k == 1) track->first = sample;
                if (k == 2) track->second = sample;
                if (!sample.present || track->foundObserved) continue;
                if (!track->known) { track->known = true; track->estimate = sample.point; }
                if (sample.observed) { track->foundObserved = true; track->age = k - 1; }
            }
            if (head.foundObserved && tail.foundObserved) break;
        }
        auto finish = [](const RoleTrack& t, cv::Point2f& pos, cv::Point2f& vel, bool& known, int& age) {
            known = t.known;
            pos = t.known ? t.estimate : cv::Point2f(0.f, 0.f);
            age = t.known ? t.age : kMaxTipAge;
            vel = (t.first.observed && t.second.observed) ? t.first.point - t.second.point
                                                          : cv::Point2f(0.f, 0.f);
        };
        finish(head, outPredictor.lastHeadPos, outPredictor.velHead,
               outPredictor.headKnown, outPredictor.headAge);
        finish(tail, outPredictor.lastTailPos, outPredictor.velTail,
               outPredictor.tailKnown, outPredictor.tailAge);
        outPredictor.hasPrev = outPredictor.headKnown || outPredictor.tailKnown;

        Tracking::DetectedBlob prevPrevBlob;
        const bool havePrevPrev = blobAt(frameNumber - (2 * frameStep), prevPrevBlob);
        const bool havePrevPrevCenter = havePrevPrev &&
            prevPrevBlob.centerline.points.size() >= 2 &&
            prevBlob.centerline.points.size() >= 2;
        if (havePrevPrevCenter) {
            outPredictor.velCenter =
                prevBlob.centerline.points[prevBlob.centerline.points.size() / 2] -
                prevPrevBlob.centerline.points[prevPrevBlob.centerline.points.size() / 2];
        }
        outPredictor.hasVelocity = havePrevPrevCenter ||
            (head.first.observed && head.second.observed) ||
            (tail.first.observed && tail.second.observed);

        outPrevState = CenterlineState{};
        if (prevBlob.centerline.points.size() >= 2) {
            outPrevState.points.assign(prevBlob.centerline.points.begin(),
                                       prevBlob.centerline.points.end());
            outPrevState.blobCentroid = blobCentroid(prevBlob);
            outPrevState.blob = prevBlob;
            outPrevState.valid = true;
            outPrevState.turningAngle = centerlineCrossSum(outPrevState.points);
            outPredictor.lastCenterPos =
                outPrevState.points[outPrevState.points.size() / 2];
        }
        return outPredictor.hasPrev || outPrevState.valid;
    };

    if (i < 0 || i >= static_cast<int>(points.size())) return result;
    const Tracking::TrackPoint& tp = points[i];

    if (tp.quality == Tracking::TrackPointQuality::Lost) {
        predictor = Centerline::HeadTailPredictor{};
        prevState = CenterlineState{};
        return result;
    }

    QMap<int, Tracking::DetectedBlob> frameBlobs =
        io.getDetectedBlobsForFrame(tp.frameNumber);
    if (!frameBlobs.contains(ctx.wormId)) return result;
    Tracking::DetectedBlob blob = frameBlobs[ctx.wormId];
    if (!blob.isValid || blob.contourPoints.empty()) return result;
    blob.centerline.hasCutPoint = false;

    const bool inMerge = tp.quality == Tracking::TrackPointQuality::Merged;

    if (inMerge && req.skipIfMerged) {
        predictor = Centerline::HeadTailPredictor{};
        prevState = CenterlineState{};
        blob.centerline.points.clear();
        blob.centerline.hasCutPoint = false;
        io.setDetectedBlobForFrame(tp.frameNumber, ctx.wormId, blob);
        result.wroteBlob = true;
        result.blob = blob;
        return result; // result.processed = false; doWork bootstraps the next non-merged frame.
    }

    Centerline::HeadTailPredictor framePredictor = predictor;
    CenterlineState framePrevState = prevState;
    if (!req.isKeyframeBootstrap) {
        loadPreviousFrameContext(tp.frameNumber, req.step, framePredictor, framePrevState);
    }

// ── STEP 1: detect endpoints, write back tip data ───────────
const Centerline::TipFeatureBaseline baseline =
    io.getTipBaseline(ctx.wormId);
// Expected body length: the clean-frame baseline once it is established,
// otherwise the Sweep 0 median. Never the previous frame's own output, so one
// bad frame cannot shrink the reference for the next.
const float frameRefLength =
    baseline.lengthSamples >= kMinBodyLengthSamples ? baseline.meanBodyLength : ctx.refLength;
if (frameRefLength > 0.f) {
    framePredictor.refDistance = std::max(8.f, 0.5f * frameRefLength);
}
const bool captureDebug =
    ctx.captureDebug;
Debug::CenterlineFrameDebug debugRecord;
debugRecord.wormId = ctx.wormId;
debugRecord.frameNumber = tp.frameNumber;
debugRecord.sweepStep = req.step;
debugRecord.keyframeBootstrap = req.isKeyframeBootstrap;
debugRecord.inMergeGroup = inMerge;
debugRecord.predictorBefore = framePredictor;
debugRecord.baselineBefore = baseline;
debugRecord.refLength = frameRefLength;
debugRecord.previousTurningAngle = framePrevState.turningAngle;
debugRecord.predictedHead = framePredictor.hasVelocity
    ? framePredictor.lastHeadPos + framePredictor.velHead
    : framePredictor.lastHeadPos;
debugRecord.predictedTail = framePredictor.hasVelocity
    ? framePredictor.lastTailPos + framePredictor.velTail
    : framePredictor.lastTailPos;
debugRecord.predictedCenter = framePredictor.hasVelocity
    ? framePredictor.lastCenterPos + framePredictor.velCenter
    : framePredictor.lastCenterPos;
debugRecord.decisions << QStringLiteral("loaded live predictor and previous-frame state");

Debug::EndpointDebug epDebug;   // filled only when captureDebug
Centerline::EndpointResult er = Centerline::detectEndpoints(
        blob, framePredictor, baseline, inMerge, captureDebug ? &epDebug : nullptr);

// ── OMEGA UNZIPPER START ──────────────────────────────────────────
if (er.topology == Tracking::TopologyState::SelfCrossed && framePredictor.hasVelocity) {
    const cv::Point2f predictedHead = framePredictor.lastHeadPos + framePredictor.velHead;
    const cv::Point2f predictedTail = framePredictor.lastTailPos + framePredictor.velTail;
    float tipDist = ptDist(predictedHead, predictedTail);

    // 1. Proximity Check
    if (baseline.isReliable() && tipDist < baseline.meanWidth * 2.5f) {
        cv::Point2f midPoint = (predictedHead + predictedTail) * 0.5f;
        int localX = static_cast<int>(std::lround(midPoint.x - er.localBounds.x));
        int localY = static_cast<int>(std::lround(midPoint.y - er.localBounds.y));

        if (localX >= 0 && localY >= 0 &&
            localX < er.distTransform.cols && localY < er.distTransform.rows) {

            // 2. Width Check
            float localRadius = er.distTransform.at<float>(localY, localX);
            float normalRadius = baseline.meanWidth / 2.0f;

            if (localRadius > normalRadius * 1.4f) {
                Tracking::DetectedBlob splitBlob = blob;

                // 3. The Cut (Draw black line between predicted tips)
                cv::Point2f cutA = predictedHead - cv::Point2f(static_cast<float>(er.localBounds.x), static_cast<float>(er.localBounds.y));
                cv::Point2f cutB = predictedTail - cv::Point2f(static_cast<float>(er.localBounds.x), static_cast<float>(er.localBounds.y));

                if (Centerline::populateCenterlineFromContourWithCut(splitBlob, cutA, cutB, 2)) {

                    // 4. Re-evaluate topology
                    Debug::EndpointDebug splitDebug;
                    Centerline::EndpointResult splitEr =
                        Centerline::detectEndpoints(splitBlob, framePredictor, baseline, inMerge,
                                                    captureDebug ? &splitDebug : nullptr);

                    if (splitEr.topology == Tracking::TopologyState::Clean) {
                        blob = splitBlob;
                        er = splitEr;
                        epDebug = splitDebug;
                        debugRecord.decisions << QStringLiteral("Omega handle detected and unzipped via predictive DT");
                    }
                }
            }
        }
    }
}
// ── OMEGA UNZIPPER END ────────────────────────────────────────────

if (captureDebug) {
    captureEndpointDebug(er, epDebug, debugRecord);
}

// Store the authoritative selected position and its estimator.
blob.centerline.tipCandidates.clear();
for (const Centerline::TrueTip& t : er.tips) {
    Tracking::TipCandidate tc;
    tc.point     = t.point;
    tc.curvature = t.curvature;
    tc.width     = t.width;
    tc.source    = t.selectedAxis ? Tracking::TipCandidate::Source::AxisBoundary
                              : t.extended ? Tracking::TipCandidate::Source::CurvaturePeak
                              : Tracking::TipCandidate::Source::SkeletonEndpoint;
    blob.centerline.tipCandidates.push_back(tc);
}
blob.centerline.headTipIdx = er.headIdx;
blob.centerline.tailTipIdx = er.tailIdx;
blob.centerline.topology      = er.topology;
debugRecord.decisions << QStringLiteral("detectEndpoints topology=%1 tips=%2 headIdx=%3 tailIdx=%4")
                             .arg(Tracking::topologyStateToString(er.topology))
                             .arg(static_cast<int>(blob.centerline.tipCandidates.size()))
                             .arg(blob.centerline.headTipIdx)
                             .arg(blob.centerline.tailTipIdx);
// Self-crossed roles are chosen together with the route in Step 2. For a
// clean frame following a clean frame, the previous centerline's order is a
// reliable second opinion on the two visible tips.
if (er.topology == Tracking::TopologyState::Clean && framePrevState.valid &&
    framePrevState.blob.centerline.topology == Tracking::TopologyState::Clean) {
    enforceTwoTipCenterlineOrderRoles(blob,
                                      framePrevState.points,
                                      framePrevState.blobCentroid,
                                      &debugRecord.decisions);
}
debugRecord.topology = er.topology;
debugRecord.assignedHeadTipIdx = blob.centerline.headTipIdx;
debugRecord.assignedTailTipIdx = blob.centerline.tailTipIdx;
debugRecord.tipCandidates = blob.centerline.tipCandidates;

// Copy terminal-axis debug — parallel to tipCandidates, with role labels.
debugRecord.tipCapDebug = epDebug.tipCapDebug;
debugRecord.tipCapRoles.resize(epDebug.tipCapDebug.size());
for (int ci = 0; ci < static_cast<int>(epDebug.tipCapDebug.size()); ++ci) {
    if (ci == blob.centerline.headTipIdx)      debugRecord.tipCapRoles[ci] = QStringLiteral("head");
    else if (ci == blob.centerline.tailTipIdx) debugRecord.tipCapRoles[ci] = QStringLiteral("tail");
    else                                    debugRecord.tipCapRoles[ci] = QString();
}

// Sample baseline (curvature + width) on Clean frames with two
// tips. Body-length sample is taken AFTER req.step 3 below using
// the resampled centerline.
const bool cleanWithTwoTips =
    er.topology == Tracking::TopologyState::Clean &&
    er.tips.size() == 2;
if (cleanWithTwoTips) {
    for (const Centerline::TrueTip& t : er.tips) {
        io.recordTipFeatureSample(
            ctx.wormId, std::abs(t.curvature), t.width);
    }
}

// Keyframe bootstrap: detectEndpoints leaves head/tail at -1
// because predictor.hasPrev is false on the first call. With
// ≥2 tips we arbitrarily pick (0, 1) so Step 2's Clean branch
// can build a centerline; head/tail will be re-derived from
// the centerline orientation after Step 3.
if (req.isKeyframeBootstrap && !er.tips.empty()) {
    if (er.tips.size() >= 2 && blob.centerline.headTipIdx < 0) {
        blob.centerline.headTipIdx = 0;
        blob.centerline.tailTipIdx = 1;
        debugRecord.decisions << QStringLiteral("keyframe bootstrap assigned head/tail candidate indices 0/1");
    } else if (blob.centerline.headTipIdx < 0 &&
               blob.centerline.tailTipIdx < 0) {
        blob.centerline.headTipIdx = 0;
        debugRecord.decisions << QStringLiteral("keyframe bootstrap assigned single head candidate index 0");
    }
    debugRecord.assignedHeadTipIdx = blob.centerline.headTipIdx;
    debugRecord.assignedTailTipIdx = blob.centerline.tailTipIdx;
}

// ── STEP 2: build centerline based on topology ──────────────
std::vector<cv::Point2f> centerline;
cv::Point2f overlapCenter(0.f, 0.f);
bool hasOverlap = false;
bool snakeRan = false;
const cv::Point2f originOffset(static_cast<float>(er.localBounds.x),
                               static_cast<float>(er.localBounds.y));
// Route selection for self-crossed skeletons (see centerlineroutes.h). Writes
// the centerline head→tail into `centerline`, registers hidden ends as
// hypothesised tips on `target`, and sets its head/tail roles. Returns false
// when no route fits the body-length window.
bool routeTrusted = false;
auto runRouteSelection = [&](Tracking::DetectedBlob& target,
                             const Centerline::EndpointResult& dispEr) -> bool {
    Centerline::RouteSelectionInput in;
    in.graph = &dispEr.skeleton;
    in.distTransform = dispEr.distTransform;
    in.localBounds = dispEr.localBounds;
    for (size_t t = 0; t < dispEr.tips.size() && t < dispEr.skeleton.endpointIndices.size(); ++t)
        in.observedTips.push_back({dispEr.skeleton.endpointIndices[t], dispEr.tips[t].point});
    in.head = Centerline::predictRole(framePredictor.headKnown, framePredictor.lastHeadPos,
                                      framePredictor.velHead, framePredictor.headAge);
    in.tail = Centerline::predictRole(framePredictor.tailKnown, framePredictor.lastTailPos,
                                      framePredictor.velTail, framePredictor.tailAge);
    in.bodyLength = frameRefLength;
    in.nPoints = ctx.nPts;
    in.hasOrientationReference = state.hasOrientationReference && !req.isKeyframeBootstrap;
    in.orientationReference = state.orientationReference;
    debugRecord.decisions << QStringLiteral("route selection predictions: head=%1 (%2,%3) age=%4  tail=%5 (%6,%7) age=%8")
        .arg(in.head.valid ? "Y" : "N")
        .arg(in.head.position.x, 0, 'f', 1).arg(in.head.position.y, 0, 'f', 1).arg(in.head.age)
        .arg(in.tail.valid ? "Y" : "N")
        .arg(in.tail.position.x, 0, 'f', 1).arg(in.tail.position.y, 0, 'f', 1).arg(in.tail.age);

    const Centerline::RouteSelectionResult sel = Centerline::selectSelfCrossedRoute(in);
    debugRecord.decisions << sel.decisions;
    debugRecord.d3RouteDebugAvailable = true;
    debugRecord.d3CandidatePaths.clear();
    for (size_t k = 0; k < sel.ranked.size() && k < 4; ++k)
        debugRecord.d3CandidatePaths.push_back(sel.ranked[k].points);
    debugRecord.d3SelectedCandidate = sel.found ? 0 : -1;
    debugRecord.d3RouteJunction = cv::Point2f(-1.f, -1.f);
    debugRecord.d3RouteCenter = cv::Point2f(-1.f, -1.f);
    if (!sel.found) return false;

    centerline = sel.best.points;
    debugRecord.d3RouteStartIsHead = true;
    debugRecord.d3RouteStart = centerline.front();
    debugRecord.d3RouteEnd = centerline.back();

    auto registerEnd = [&](const cv::Point2f& point, Centerline::RouteEndKind kind) -> int {
        auto& candidates = target.centerline.tipCandidates;
        if (kind != Centerline::RouteEndKind::Hidden) {
            for (int idx = 0; idx < static_cast<int>(candidates.size()); ++idx) {
                if (candidates[idx].source != Tracking::TipCandidate::Source::HypothesizedHidden &&
                    ptDist(candidates[idx].point, point) < 0.5f)
                    return idx;
            }
        }
        Tracking::TipCandidate tc;
        tc.point = point;
        tc.source = kind == Centerline::RouteEndKind::Hidden
            ? Tracking::TipCandidate::Source::HypothesizedHidden
            : Tracking::TipCandidate::Source::SkeletonEndpoint;
        candidates.push_back(tc);
        if (kind == Centerline::RouteEndKind::Hidden) {
            debugRecord.hiddenTipHypothesized = true;
            debugRecord.hiddenTipFinal = point;
        }
        return static_cast<int>(candidates.size()) - 1;
    };
    target.centerline.headTipIdx = registerEnd(centerline.front(), sel.best.headKind);
    target.centerline.tailTipIdx = registerEnd(centerline.back(), sel.best.tailKind);
    routeTrusted = sel.best.headKind != Centerline::RouteEndKind::Hidden &&
                   sel.best.tailKind != Centerline::RouteEndKind::Hidden &&
                   std::abs(sel.best.length - frameRefLength) <= 0.08f * frameRefLength;
    return true;
};

if (er.topology == Tracking::TopologyState::Clean &&
    blob.centerline.headTipIdx >= 0 &&
    blob.centerline.tailTipIdx >= 0 &&
    static_cast<int>(er.skeleton.endpointIndices.size()) >
        std::max(blob.centerline.headTipIdx, blob.centerline.tailTipIdx)) {

    const int hGraphIdx =
        er.skeleton.endpointIndices[blob.centerline.headTipIdx];
    const int tGraphIdx =
        er.skeleton.endpointIndices[blob.centerline.tailTipIdx];

    std::vector<cv::Point2f> graphPath;
    if (skeletonGraphPath(er.skeleton, hGraphIdx, tGraphIdx,
                          originOffset, graphPath)) {
        centerline = std::move(graphPath);
        debugRecord.branch = Debug::CenterlineBranch::D1CleanGraphPath;
        debugRecord.decisions << QStringLiteral("D-1 clean skeleton graph path selected");
        if (!centerline.empty())
            centerline.front() = er.tips[blob.centerline.headTipIdx].point;
        if (!centerline.empty())
            centerline.back() = er.tips[blob.centerline.tailTipIdx].point;

        // If D-1 result is suspiciously short the worm is
        // tightly self-coiled but had no visible hole in the
        // mask. Punch a synthetic hole at the DT maximum and
        // choose a route through the resulting loop.
        if (frameRefLength > 0.f &&
            arcLen(centerline) < 0.5f * frameRefLength) {
            Tracking::DetectedBlob holeBlob;
            if (addSyntheticHoleAtDTMax(blob, er.distTransform,
                                         er.localBounds, holeBlob)) {
                debugRecord.syntheticHoleUsed = true;
                debugRecord.decisions << QStringLiteral("D-1 path was short; synthetic hole retry attempted");
                const Centerline::EndpointResult er2 =
                    Centerline::detectEndpoints(
                        holeBlob, framePredictor, baseline, inMerge);
                std::vector<cv::Point2f> prevCl = centerline;
                Tracking::DetectedBlob savedBlob = blob;
                blob = holeBlob;
                blob.centerline.tipCandidates.clear();
                for (const Centerline::TrueTip& t : er2.tips) {
                    Tracking::TipCandidate tc;
                    tc.point     = t.point;
                    tc.curvature = t.curvature;
                    tc.width     = t.width;
                    tc.source    = t.selectedAxis ? Tracking::TipCandidate::Source::AxisBoundary
                              : t.extended ? Tracking::TipCandidate::Source::CurvaturePeak
                                              : Tracking::TipCandidate::Source::SkeletonEndpoint;
                    blob.centerline.tipCandidates.push_back(tc);
                }
                blob.centerline.headTipIdx = er2.headIdx;
                blob.centerline.tailTipIdx = er2.tailIdx;
                blob.centerline.topology      = er2.topology;
                centerline.clear();
                if (!runRouteSelection(blob, er2)) {
                    blob = savedBlob;
                    centerline = prevCl;
                    debugRecord.decisions << QStringLiteral("synthetic hole retry failed; restored D-1 path");
                }
                else {
                    debugRecord.branch = Debug::CenterlineBranch::D1SyntheticHoleRetry;
                    debugRecord.decisions << QStringLiteral("synthetic hole retry supplied centerline");
                }
            }
        }
    }
}
else if (er.topology == Tracking::TopologyState::SelfCrossed) {
    // One route selector handles every self-crossed case (two, one, or no
    // visible tips). A frame with no route inside the body-length window is
    // left unresolved rather than given an implausible centerline.
    if (runRouteSelection(blob, er)) {
        debugRecord.branch = Debug::CenterlineBranch::SelfCrossedRouteSelection;
    } else {
        debugRecord.branch = Debug::CenterlineBranch::SelfCrossedUnresolved;
    }
}

if (centerline.empty() && debugRecord.branch != Debug::CenterlineBranch::SelfCrossedUnresolved) {
    // D-4 fallback: legacy contour-skeleton path.
    debugRecord.fallbackUsed = true;
    debugRecord.branch = Debug::CenterlineBranch::D4FallbackContourSkeleton;
    Tracking::DetectedBlob fallback = blob;
    if (Centerline::populateCenterlineFromContour(fallback) &&
        fallback.centerline.points.size() >= 2) {
        centerline.assign(fallback.centerline.points.begin(),
                          fallback.centerline.points.end());
        debugRecord.decisions << QStringLiteral("D-4 fallback contour skeleton supplied centerline");
    }
}

debugRecord.initialCenterline = centerline;
debugRecord.initialArcLength = centerline.size() >= 2 ? arcLen(centerline) : 0.f;
debugRecord.tipCandidates = blob.centerline.tipCandidates;
debugRecord.assignedHeadTipIdx = blob.centerline.headTipIdx;
debugRecord.assignedTailTipIdx = blob.centerline.tailTipIdx;

if (centerline.size() < 2) {
    // No centerline producible. Persist what we DID compute
    // (tip data) so the renderer still shows green dots.
    io.setDetectedBlobForFrame(tp.frameNumber, ctx.wormId, blob);
    result.wroteBlob = true;
    result.blob = blob;
    debugRecord.decisions << QStringLiteral("no centerline producible; persisted endpoint/debug blob state only");
    if (captureDebug) {
        io.setCenterlineDebugFrame(debugRecord);
    }
    result.processed = true;
    result.debugRecord = debugRecord;
    prevState.valid = false;
    return result;
}

// ── STEP 3: resample to nPoints ─────────────────────────────
if (static_cast<int>(centerline.size()) != ctx.nPts)
    centerline = resample(centerline, ctx.nPts);
debugRecord.resampledCenterline = centerline;

if (cleanWithTwoTips) {
    io.recordBodyLengthSample(ctx.wormId, arcLen(centerline));
}

// ── STEP 4: snake refinement (Clean only) ──────────────────
// SelfCrossed centerlines come from S-1 route selection, which traces the
// skeleton directly; no snake pass.
if (er.topology == Tracking::TopologyState::Clean) {
    cv::Mat mask;
    cv::Rect bounds;
    if (buildSnakeMask(blob, mask, bounds)) {
        const cv::Point2f pinH = (blob.centerline.headTipIdx >= 0 &&
                                  blob.centerline.headTipIdx <
                                    static_cast<int>(blob.centerline.tipCandidates.size()))
            ? blob.centerline.tipCandidates[blob.centerline.headTipIdx].point
            : centerline.front();
        const cv::Point2f pinT = (blob.centerline.tailTipIdx >= 0 &&
                                  blob.centerline.tailTipIdx <
                                    static_cast<int>(blob.centerline.tipCandidates.size()))
            ? blob.centerline.tipCandidates[blob.centerline.tailTipIdx].point
            : centerline.back();
        cv::Point2f tmpOverlap(0.f, 0.f);
        bool tmpHasOverlap = false;
        refineSnakeCore(blob, mask, bounds, centerline,
                        pinH, pinT, ctx.nPts, ctx.snakeParams,
                        tmpOverlap, tmpHasOverlap);
        snakeRan = true;
        debugRecord.decisions << QStringLiteral("snake refinement ran on clean topology frame");
    }
}

// Orientation is enforced while choosing self-crossed routes; a finished
// centerline is never reversed afterwards.
const float curTurning = centerlineCrossSum(centerline);
const bool flipped = false;

// Keyframe bootstrap re-derivation: now that we have a
// centerline, set head/tail from its natural orientation
// (front = head). This stabilises the convention regardless
// of which tip arbitrarily got tips[0] in detectEndpoints.
if (req.isKeyframeBootstrap && centerline.size() >= 2 &&
    !blob.centerline.tipCandidates.empty()) {
    const int headIdx = nearestCandidateIdx(blob, centerline.front());
    int       tailIdx = nearestCandidateIdx(blob, centerline.back());
    if (tailIdx == headIdx) tailIdx = -1;
    blob.centerline.headTipIdx = headIdx;
    blob.centerline.tailTipIdx = tailIdx;
    debugRecord.decisions << QStringLiteral("keyframe bootstrap re-derived head/tail from centerline orientation");
}

// Persist centerline + cut/overlap marker on the blob.
blob.centerline.points.assign(centerline.begin(), centerline.end());
if (hasOverlap) {
    blob.centerline.cutPoint = overlapCenter;
    blob.centerline.hasCutPoint = true;
}

io.setDetectedBlobForFrame(tp.frameNumber, ctx.wormId, blob);
result.wroteBlob = true;
result.blob = blob;

debugRecord.snakeRan = snakeRan;
debugRecord.rhrFlipped = flipped;
debugRecord.finalCenterline = centerline;
debugRecord.finalArcLength = arcLen(centerline);
debugRecord.finalTurningAngle = flipped ? -curTurning : curTurning;
debugRecord.tipCandidates = blob.centerline.tipCandidates;
debugRecord.assignedHeadTipIdx = blob.centerline.headTipIdx;
debugRecord.assignedTailTipIdx = blob.centerline.tailTipIdx;
debugRecord.topology = blob.centerline.topology;
// Roles can change after bootstrap or routing. Label cap diagnostics only when
// their selected detection still matches a final assigned candidate.
for (size_t ci = 0; ci < debugRecord.tipCapDebug.size(); ++ci) {
    auto& role = debugRecord.tipCapRoles[ci];
    role.clear();
    const auto& selected = debugRecord.tipCapDebug[ci].selectedPoint;
    for (int idx : {blob.centerline.headTipIdx, blob.centerline.tailTipIdx}) {
        if (idx >= 0 && idx < static_cast<int>(blob.centerline.tipCandidates.size()) &&
            ptDist(selected, blob.centerline.tipCandidates[idx].point) < 1e-4f) {
            role = idx == blob.centerline.headTipIdx ? QStringLiteral("head") : QStringLiteral("tail");
        }
    }
}
if (captureDebug) {
    io.setCenterlineDebugFrame(debugRecord);
}

// ── STEP 5: predictor update ────────────────────────────────
// Mirrors loadPreviousFrameContext for the carried state: a hypothesised end
// updates the position estimate but not the observation age or velocity.
const int hIdx = blob.centerline.headTipIdx;
const int tIdx = blob.centerline.tailTipIdx;
auto updateRole = [&](int idx, cv::Point2f& last, cv::Point2f& vel, bool& known, int& age) {
    if (idx < 0 || idx >= static_cast<int>(blob.centerline.tipCandidates.size())) {
        age = std::min(age + 1, kMaxTipAge);
        vel = cv::Point2f(0.f, 0.f);
        return;
    }
    const auto& tc = blob.centerline.tipCandidates[idx];
    const bool observed = tc.source != Tracking::TipCandidate::Source::HypothesizedHidden;
    vel = (observed && known && age == 0) ? tc.point - last : cv::Point2f(0.f, 0.f);
    age = observed ? 0 : std::min(age + 1, kMaxTipAge);
    last = tc.point;
    known = true;
};
updateRole(hIdx, predictor.lastHeadPos, predictor.velHead, predictor.headKnown, predictor.headAge);
updateRole(tIdx, predictor.lastTailPos, predictor.velTail, predictor.tailKnown, predictor.tailAge);
if (centerline.size() >= 2) {
    const cv::Point2f newC = centerline[centerline.size() / 2];
    if (predictor.hasPrev)
        predictor.velCenter = newC - predictor.lastCenterPos;
    predictor.lastCenterPos = newC;
}
predictor.hasVelocity = predictor.hasPrev &&
                        (hIdx >= 0 || tIdx >= 0 || centerline.size() >= 2);
predictor.hasPrev = true;
if (frameRefLength > 0.f)
    predictor.refDistance = std::max(8.f, 0.5f * frameRefLength);

// Keep the loop orientation of the latest trusted centerline: a clean frame
// with two visible tips, or a self-crossed route whose ends are both visible
// and whose length matches the body. Frames with hypothesised ends leave it.
if ((cleanWithTwoTips && debugRecord.branch == Debug::CenterlineBranch::D1CleanGraphPath) ||
    routeTrusted) {
    state.hasOrientationReference = true;
    state.orientationReference = Centerline::signedTurning(centerline);
    debugRecord.decisions << QStringLiteral("orientation reference updated to %1")
                                 .arg(state.orientationReference, 0, 'f', 2);
}

// Carried previous-frame state.
prevState.points       = centerline;
prevState.blobCentroid = blobCentroid(blob);
prevState.blob         = blob;
prevState.valid        = true;
prevState.turningAngle = flipped ? -curTurning : curTurning;

    result.processed = true;
    result.debugRecord = debugRecord;
    return result;

}

bool relaxCenterlineToSmoothedMidpoint(Tracking::DetectedBlob& blob,
                                      const cv::Point2f& midpointTarget,
                                      int nPoints,
                                      const CenterlineSnakeParams& params)
{
    if (blob.contourPoints.empty()) return false;
    if (blob.centerline.points.empty()) return false;
    if (blob.centerline.topology != Tracking::TopologyState::Clean) return false;

    const int hIdx = blob.centerline.headTipIdx;
    const int tIdx = blob.centerline.tailTipIdx;
    if (hIdx < 0 || tIdx < 0) return false;
    if (hIdx >= static_cast<int>(blob.centerline.tipCandidates.size())) return false;
    if (tIdx >= static_cast<int>(blob.centerline.tipCandidates.size())) return false;

    cv::Mat mask;
    cv::Rect bounds;
    if (!buildSnakeMask(blob, mask, bounds)) return false;
    const int mx = static_cast<int>(std::lround(midpointTarget.x - bounds.x));
    const int my = static_cast<int>(std::lround(midpointTarget.y - bounds.y));
    if (mx < 0 || my < 0 || mx >= mask.cols || my >= mask.rows ||
        mask.at<uchar>(my, mx) == 0) return false;

    const cv::Point2f pinH = blob.centerline.tipCandidates[hIdx].point;
    const cv::Point2f pinT = blob.centerline.tipCandidates[tIdx].point;

    std::vector<cv::Point2f> centerline(blob.centerline.points.begin(),
                                        blob.centerline.points.end());
    cv::Point2f overlapCenter(0.f, 0.f);
    bool hasOverlap = false;
    if (!refineSnakeCore(blob, mask, bounds, centerline, pinH, pinT,
                         nPoints, params, overlapCenter, hasOverlap,
                         &midpointTarget))
        return false;

    blob.centerline.points.assign(centerline.begin(), centerline.end());
    return true;
}

} // namespace Centerline
