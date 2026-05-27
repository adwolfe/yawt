
#include "trackingcommon.h" // Lowercase include
#include "../core/centerlineprocessor.h"
#include <QtMath>       // For qSqrt, qPow
#include <QDebug>       // For qWarning/qDebug
#include "../utils/loggingcategories.h"

#include <algorithm>
#include <cmath>
#include <limits>


namespace Tracking {

namespace {

constexpr int kCenterlinePaddingPixels = 2;

cv::Rect buildCenterlineMask(const DetectedBlob& blob, cv::Mat& mask)
{
    cv::Rect localBounds = cv::boundingRect(blob.contourPoints);
    localBounds.x -= kCenterlinePaddingPixels;
    localBounds.y -= kCenterlinePaddingPixels;
    localBounds.width += 2 * kCenterlinePaddingPixels;
    localBounds.height += 2 * kCenterlinePaddingPixels;

    if (localBounds.width <= 1 || localBounds.height <= 1) {
        mask.release();
        return localBounds;
    }

    mask = cv::Mat::zeros(localBounds.height, localBounds.width, CV_8UC1);

    std::vector<std::vector<cv::Point>> outerContours(1);
    outerContours.front().reserve(blob.contourPoints.size());
    for (const cv::Point& pt : blob.contourPoints) {
        outerContours.front().push_back(cv::Point(pt.x - localBounds.x, pt.y - localBounds.y));
    }
    cv::fillPoly(mask, outerContours, cv::Scalar(255));

    for (const std::vector<cv::Point>& hole : blob.holeContourPoints) {
        std::vector<std::vector<cv::Point>> holeContour(1);
        holeContour.front().reserve(hole.size());
        for (const cv::Point& pt : hole) {
            holeContour.front().push_back(cv::Point(pt.x - localBounds.x, pt.y - localBounds.y));
        }
        cv::fillPoly(mask, holeContour, cv::Scalar(0));
    }

    if (!blob.holeContourPoints.empty()) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    }

    return localBounds;
}


} // namespace

// ── Single-pass endpoint detector ───────────────────────────────────────────
//
// Used by the centerline pipeline (centerlineworker doWork). Builds the
// skeleton once, then derives every downstream quantity from it.
//
// Design notes:
//
// * Curvature math + width probe are kept stable so baseline samples stay
//   commensurable across frames.
//
// * Mask construction reuses buildCenterlineMask, including the MORPH_CLOSE
//   applied for ring blobs.
//
// * Endpoint pruning preserves the longest-path pair when more than two
//   degree-1 nodes are present (skeleton noise spurs). Never drops below two
//   if two existed; never drops the only one if only one exists.
//
// * Each surviving skeleton endpoint is "extended" to the strongest curvature
//   peak within DT(endpoint) * 1.5 pixels (search radius scales with local
//   body thickness). Strongest, not nearest — picking the nearest peak snaps
//   onto body kinks for tightly-coiled worms.
EndpointResult detectEndpoints(const DetectedBlob& blob,
                               const HeadTailPredictor& predictor,
                               const TipFeatureBaseline& baseline,
                               bool inMergeGroup)
{
    EndpointResult r;

    if (!blob.isValid) {
        r.topology = TopologyState::Lost;
        return r;
    }
    // Merged blobs still get their skeleton + tips harvested when possible —
    // the topology label is set to Merged at the end. centerlineworker treats
    // Merged as a no-op for centerline computation but the tip data is still
    // useful for the renderer.
    if (blob.contourPoints.size() < 8) {
        r.topology = inMergeGroup ? TopologyState::Merged
                                  : TopologyState::SelfCrossed;
        return r;
    }

    // (a) ── Mask + DT ──────────────────────────────────────────────────────
    cv::Mat mask;
    r.localBounds = buildCenterlineMask(blob, mask);
    if (mask.empty()) {
        r.topology = inMergeGroup ? TopologyState::Merged
                                  : TopologyState::SelfCrossed;
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
        r.topology = inMergeGroup ? TopologyState::Merged
                                  : TopologyState::SelfCrossed;
        return r;
    }
    std::vector<int> rawEndpoints = r.skeleton.endpointIndices;

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
    r.contourPoints.reserve(nContour);
    r.contourCurvatures = curvature;
    r.contourCurvaturePeaks = curvaturePeakIdx;
    for (const cv::Point& p : contour) {
        r.contourPoints.emplace_back(static_cast<float>(p.x),
                                     static_cast<float>(p.y));
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

        int prev = epIdx;
        int cur = r.skeleton.adjacency[epIdx].front();
        cv::Point inner = r.skeleton.points[cur];
        constexpr int kEndpointDirectionSteps = 6;
        for (int step = 1; step < kEndpointDirectionSteps; ++step) {
            int next = -1;
            for (int candidate : r.skeleton.adjacency[cur]) {
                if (candidate != prev) {
                    next = candidate;
                    break;
                }
            }
            if (next < 0) {
                break;
            }
            prev = cur;
            cur = next;
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

    auto projectedEndpointContourIdx = [&](const cv::Point2f& epLocalF,
                                           const cv::Point2f& outwardDir,
                                           float dtAtEp) -> int {
        if (std::hypot(outwardDir.x, outwardDir.y) < 1e-3f) {
            return nearestContourIdx(epLocalF);
        }

        const float maxForward = std::clamp(6.f + 4.f * dtAtEp, 8.f, 18.f);
        const float maxSide = std::clamp(2.f + 2.5f * dtAtEp, 4.f, 10.f);
        int bestIdx = -1;
        float bestScore = -std::numeric_limits<float>::max();
        for (int i = 0; i < nContour; ++i) {
            const cv::Point2f rel = contourLocal[i] - epLocalF;
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
            const float score = forward - 0.25f * side + curvatureBonus;
            if (score > bestScore) {
                bestScore = score;
                bestIdx = i;
            }
        }

        return bestIdx >= 0 ? bestIdx : nearestContourIdx(epLocalF);
    };

    // (e) ── Extend each skeleton endpoint to the strongest reachable peak ──
    for (int epIdx : r.skeleton.endpointIndices) {
        const cv::Point& epLocal = r.skeleton.points[epIdx];
        const cv::Point2f epLocalF(static_cast<float>(epLocal.x),
                                   static_cast<float>(epLocal.y));
        const cv::Point2f epWorld(epLocalF.x + originOffset.x,
                                  epLocalF.y + originOffset.y);

        const float dtAtEp = r.distTransform.at<float>(epLocal.y, epLocal.x);
        const cv::Point2f outwardDir = endpointOutwardDirection(epIdx);

        // Skeleton endpoint projected outward to the cap contour. A nearest
        // contour snap often lands on the side of a rounded tip instead of at
        // the end-cap apex.
        const int snapIdx = projectedEndpointContourIdx(epLocalF, outwardDir, dtAtEp);
        cv::Point2f snapLocal(0.f, 0.f);
        cv::Point2f snapWorld = epWorld;
        if (snapIdx >= 0) {
            snapLocal = contourLocal[snapIdx];
            snapWorld = cv::Point2f(snapLocal.x + originOffset.x,
                                    snapLocal.y + originOffset.y);
        }

        // Find a strong curvature peak in the local end-cap region defined by
        // the skeleton endpoint and its outward direction. This avoids making
        // curvature depend on a possibly side-biased contour snap.
        const float maxForward = std::clamp(6.f + 4.f * dtAtEp, 8.f, 18.f);
        const float maxSide = std::clamp(2.f + 2.5f * dtAtEp, 4.f, 10.f);
        int bestPeak = -1;
        float bestScore = -std::numeric_limits<float>::max();
        for (int peakIdx : curvaturePeakIdx) {
            const cv::Point2f& peakLocal = contourLocal[peakIdx];
            const cv::Point2f rel = peakLocal - epLocalF;
            float forward = 0.f;
            float side = std::sqrt(rel.x * rel.x + rel.y * rel.y);
            if (std::hypot(outwardDir.x, outwardDir.y) >= 1e-3f) {
                forward = rel.x * outwardDir.x + rel.y * outwardDir.y;
                const float cross = rel.x * outwardDir.y - rel.y * outwardDir.x;
                side = std::abs(cross);
            }
            if (forward < -1.f || forward > maxForward || side > maxSide) {
                continue;
            }

            const float score = forward - 0.25f * side + 4.f * std::abs(curvature[peakIdx]);
            if (score > bestScore) {
                bestScore = score;
                bestPeak = peakIdx;
            }
        }

        if (bestPeak >= 0) {
            const cv::Point2f peakDelta = contourLocal[bestPeak] - snapLocal;
            const float peakDist = std::hypot(peakDelta.x, peakDelta.y);
            const float maxPeakShift = std::clamp(1.0f + 0.75f * dtAtEp, 2.0f, 4.0f);
            if (peakDist > maxPeakShift) {
                bestPeak = -1;
            }
        }

        // ── (e2) Bilateral cap midpoint ───────────────────────────────────────
        //
        // Instead of relying on a single best-scored contour point (which can
        // jump 1–2 px between frames when the mask is imperfect), we split the
        // end-cap contour points into left-of-axis and right-of-axis halves,
        // then independently find the "apex" on each side — the point with the
        // greatest forward component along the outward direction. Taking the
        // midpoint of these two apexes gives a tip estimate that is robust to
        // one-sided mask noise because both sides' errors partially cancel.
        //
        // To further suppress single-pixel outliers we use a weighted centroid
        // of the top-forward fraction of cap points on each side rather than the
        // single furthest point. Points in the forward quartile (within
        // kFwdFraction of the side's own peak) contribute with weight
        // proportional to their forward depth, so genuine tip pixels dominate.
        //
        // A TipCapDebug is populated in parallel for the debug exporter.
        {
            constexpr float kFwdFraction = 0.25f; // include points within 25% of apex fwd

            const cv::Point2f perp(-outwardDir.y, outwardDir.x); // left of D

            // Debug snapshot — always populated so the exporter can show the
            // search window even when the bilateral computation falls through.
            TipCapDebug capDbg;
            capDbg.valid       = true;
            capDbg.skelEndpoint = snapWorld;
            capDbg.outwardDir  = outwardDir;
            capDbg.dtAtEp      = dtAtEp;
            capDbg.maxForward  = maxForward;
            capDbg.maxSide     = maxSide;
            capDbg.snapPoint       = snapWorld;
            capDbg.peakOrSnapPoint = snapWorld; // updated below if a peak is found
            capDbg.hadPeak         = false;

            float leftPeakFwd  = -std::numeric_limits<float>::max();
            float rightPeakFwd = -std::numeric_limits<float>::max();

            // First pass: find peak forward depth on each side and collect
            // all cap contour points for the debug snapshot.
            for (int i = 0; i < nContour; ++i) {
                const cv::Point2f rel = contourLocal[i] - epLocalF;
                const float fwd = rel.x * outwardDir.x + rel.y * outwardDir.y;
                if (fwd < -1.f || fwd > maxForward) continue;
                const float lat = rel.x * perp.x + rel.y * perp.y;
                if (std::abs(lat) > maxSide) continue;

                const cv::Point2f worldPt(contourLocal[i].x + originOffset.x,
                                          contourLocal[i].y + originOffset.y);
                if (lat >= 0.f) {
                    capDbg.leftCapPoints.push_back(worldPt);
                    if (fwd > leftPeakFwd) leftPeakFwd = fwd;
                }
                if (lat <= 0.f) {
                    capDbg.rightCapPoints.push_back(worldPt);
                    if (fwd > rightPeakFwd) rightPeakFwd = fwd;
                }
            }

            const bool hasLeft  = leftPeakFwd  > -std::numeric_limits<float>::max();
            const bool hasRight = rightPeakFwd > -std::numeric_limits<float>::max();
            capDbg.hasLeft       = hasLeft;
            capDbg.hasRight      = hasRight;
            capDbg.leftPeakFwd   = hasLeft  ? leftPeakFwd  : 0.f;
            capDbg.rightPeakFwd  = hasRight ? rightPeakFwd : 0.f;

            if (hasLeft && hasRight) {
                // Sanity: both sides' apexes should be at similar forward depths.
                // If one side is dramatically deeper the mask is degenerate; skip.
                const float fwdSpan = std::max(leftPeakFwd, rightPeakFwd) -
                                      std::min(leftPeakFwd, rightPeakFwd);
                const float fwdMean = 0.5f * (leftPeakFwd + rightPeakFwd);

                if (fwdSpan <= 0.6f * fwdMean + 2.f) {
                    capDbg.sanityPassed = true;

                    // Second pass: weighted centroid of points near each apex.
                    const float leftThresh  = leftPeakFwd  - kFwdFraction * leftPeakFwd;
                    const float rightThresh = rightPeakFwd - kFwdFraction * rightPeakFwd;

                    cv::Point2f leftSum(0.f, 0.f),  rightSum(0.f, 0.f);
                    float       leftWt = 0.f,        rightWt = 0.f;

                    for (int i = 0; i < nContour; ++i) {
                        const cv::Point2f rel = contourLocal[i] - epLocalF;
                        const float fwd = rel.x * outwardDir.x + rel.y * outwardDir.y;
                        if (fwd < -1.f || fwd > maxForward) continue;
                        const float lat = rel.x * perp.x + rel.y * perp.y;
                        if (std::abs(lat) > maxSide) continue;

                        if (lat >= 0.f && fwd >= leftThresh) {
                            const float w = fwd + 1.f; // weight by forward depth
                            leftSum += w * contourLocal[i];
                            leftWt  += w;
                        }
                        if (lat <= 0.f && fwd >= rightThresh) {
                            const float w = fwd + 1.f;
                            rightSum += w * contourLocal[i];
                            rightWt  += w;
                        }
                    }

                    if (leftWt > 0.f && rightWt > 0.f) {
                        const cv::Point2f leftCentroid  = leftSum  * (1.f / leftWt);
                        const cv::Point2f rightCentroid = rightSum * (1.f / rightWt);
                        const cv::Point2f midLocal = (leftCentroid + rightCentroid) * 0.5f;
                        const cv::Point2f bilateralWorld(midLocal.x + originOffset.x,
                                                         midLocal.y + originOffset.y);
                        const cv::Point2f leftApexWorld(leftCentroid.x + originOffset.x,
                                                        leftCentroid.y + originOffset.y);
                        const cv::Point2f rightApexWorld(rightCentroid.x + originOffset.x,
                                                         rightCentroid.y + originOffset.y);
                        capDbg.leftApex      = leftApexWorld;
                        capDbg.rightApex     = rightApexWorld;
                        capDbg.bilateralTip  = bilateralWorld;
                        capDbg.hasBilateral  = true;

                        TrueTip t;
                        t.skelPoint     = snapWorld;
                        t.bilateralTip  = bilateralWorld;
                        t.hasBilateral  = true;
                        if (bestPeak >= 0) {
                            const cv::Point2f peakLocal = contourLocal[bestPeak];
                            const cv::Point2f peakWorld(peakLocal.x + originOffset.x,
                                                        peakLocal.y + originOffset.y);
                            t.point     = peakWorld;
                            t.curvature = curvature[bestPeak];
                            t.width     = widthAt(peakLocal, bestPeak);
                            t.extended  = true;
                            capDbg.peakOrSnapPoint = peakWorld;
                            capDbg.hadPeak         = true;
                        } else {
                            t.point     = snapWorld;
                            t.curvature = (snapIdx >= 0) ? curvature[snapIdx] : 0.f;
                            t.width     = (snapIdx >= 0) ? widthAt(snapLocal, snapIdx) : 0.f;
                            t.extended  = false;
                        }
                        r.tips.push_back(t);
                        r.tipCapDebug.push_back(capDbg);
                        continue; // skip the fallback TrueTip construction below
                    }
                }
            }

            // Fallback path: bilateral not available. Fill in comparison fields.
            if (bestPeak >= 0) {
                const cv::Point2f peakWorld(contourLocal[bestPeak].x + originOffset.x,
                                            contourLocal[bestPeak].y + originOffset.y);
                capDbg.peakOrSnapPoint = peakWorld;
                capDbg.hadPeak         = true;
            }
            r.tipCapDebug.push_back(capDbg);
        }

        // Fallback (no valid bilateral): construct TrueTip with snap/peak only.
        TrueTip t;
        t.skelPoint = snapWorld;
        if (bestPeak >= 0) {
            const cv::Point2f peakLocal = contourLocal[bestPeak];
            t.point     = cv::Point2f(peakLocal.x + originOffset.x,
                                      peakLocal.y + originOffset.y);
            t.curvature = curvature[bestPeak];
            t.width     = widthAt(peakLocal, bestPeak);
            t.extended  = true;
        } else {
            t.point     = snapWorld;
            t.curvature = (snapIdx >= 0) ? curvature[snapIdx] : 0.f;
            t.width     = (snapIdx >= 0) ? widthAt(snapLocal, snapIdx) : 0.f;
            t.extended  = false;
        }
        r.tips.push_back(t);
    }

    // (f) ── Topology classification ───────────────────────────────────────
    if (inMergeGroup) {
        r.topology = TopologyState::Merged;
    } else {
        const bool hasRing = !blob.holeContourPoints.empty();
        r.topology = (hasRing || r.tips.size() < 2)
                         ? TopologyState::SelfCrossed
                         : TopologyState::Clean;
    }

    // (g) ── Head/tail assignment ──────────────────────────────────────────
    // Distance-only assignment, with velocity-extrapolated tiebreak when the
    // 2-tip cost spread is < 10%. Predictor-less (keyframe) calls return
    // (-1, -1); the caller bootstraps from the centerline orientation.
    auto distSq = [](const cv::Point2f& a, const cv::Point2f& b) -> float {
        const cv::Point2f d = a - b;
        return d.x * d.x + d.y * d.y;
    };

    if (r.tips.empty() || !predictor.hasPrev) {
        // Nothing to do — caller handles bootstrap.
    } else {
        const cv::Point2f predHead = predictor.hasVelocity
            ? predictor.lastHeadPos + predictor.velHead
            : predictor.lastHeadPos;
        const cv::Point2f predTail = predictor.hasVelocity
            ? predictor.lastTailPos + predictor.velTail
            : predictor.lastTailPos;

        if (r.tips.size() == 1) {
            // One visible tip — assign to whichever role's predicted point
            // is closer. The other role stays -1 so D-3 can fill it.
            const float dH = distSq(r.tips[0].point, predHead);
            const float dT = distSq(r.tips[0].point, predTail);
            if (dH <= dT) r.headIdx = 0;
            else          r.tailIdx = 0;
        } else {
            // Two tips — minimum-cost ordered assignment.
            const float c00 = distSq(r.tips[0].point, predHead) +
                              distSq(r.tips[1].point, predTail);
            const float c01 = distSq(r.tips[1].point, predHead) +
                              distSq(r.tips[0].point, predTail);
            const float winner = std::min(c00, c01);
            const float loser  = std::max(c00, c01);
            const bool spreadIsTight = (winner > 0.f) &&
                                       ((loser - winner) / winner < 0.10f);

            int headPick = (c00 <= c01) ? 0 : 1;
            int tailPick = (c00 <= c01) ? 1 : 0;

            // Velocity-extrapolated tiebreak for tight spreads.
            if (spreadIsTight && predictor.hasVelocity) {
                const cv::Point2f extrapHead = predictor.lastHeadPos +
                                               2.f * predictor.velHead;
                const cv::Point2f extrapTail = predictor.lastTailPos +
                                               2.f * predictor.velTail;
                const float c00x = distSq(r.tips[0].point, extrapHead) +
                                   distSq(r.tips[1].point, extrapTail);
                const float c01x = distSq(r.tips[1].point, extrapHead) +
                                   distSq(r.tips[0].point, extrapTail);
                if (c00x <= c01x) { headPick = 0; tailPick = 1; }
                else              { headPick = 1; tailPick = 0; }
            }

            r.headIdx = headPick;
            r.tailIdx = tailPick;
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
            qUtf8Printable(topologyStateToString(r.topology)),
            r.headIdx, r.tailIdx);
        for (size_t i = 0; i < r.tips.size(); ++i) {
            const TrueTip& t = r.tips[i];
            YAWT_DEBUG(lcDataCommon) << QString::asprintf(
                "  tip[%zu] %s  point=(%.1f,%.1f)  skel=(%.1f,%.1f)  "
                "k=%+.4f  w=%.2f",
                i, t.extended ? "ext " : "skel",
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

bool populateCenterlineFromContour(DetectedBlob& blob)
{
    blob.centerlinePoints.clear();

    if (!blob.isValid || blob.contourPoints.size() < 3) {
        return false;
    }

    cv::Mat mask;
    cv::Rect localBounds = buildCenterlineMask(blob, mask);
    if (mask.empty()) return false;

    blob.centerlinePoints = Centerline::extractCenterlineFromMask(
        mask,
        cv::Point2f(static_cast<float>(localBounds.x), static_cast<float>(localBounds.y)));

    return blob.centerlinePoints.size() >= 2;
}

bool populateCenterlineFromContourWithCut(DetectedBlob& blob,
                                          const cv::Point2f& cutStart,
                                          const cv::Point2f& cutEnd,
                                          int cutThickness)
{
    blob.centerlinePoints.clear();

    if (!blob.isValid || blob.contourPoints.size() < 3 || blob.holeContourPoints.empty()) {
        return false;
    }

    cv::Mat mask;
    cv::Rect localBounds = buildCenterlineMask(blob, mask);
    if (mask.empty()) return false;

    const cv::Point localStart(qRound(cutStart.x - localBounds.x),
                               qRound(cutStart.y - localBounds.y));
    const cv::Point localEnd(qRound(cutEnd.x - localBounds.x),
                             qRound(cutEnd.y - localBounds.y));
    const int thickness = std::max(1, cutThickness);
    cv::line(mask, localStart, localEnd, cv::Scalar(0), thickness, cv::LINE_8);

    blob.centerlinePoints = Centerline::extractCenterlineFromMask(
        mask,
        cv::Point2f(static_cast<float>(localBounds.x), static_cast<float>(localBounds.y)));

    return blob.centerlinePoints.size() >= 2;
}

QList<QPointF> extractOrderedCenterlinePoints(const DetectedBlob& blob)
{
    QList<QPointF> centerlinePoints;
    if (!blob.isValid) {
        return centerlinePoints;
    }

    DetectedBlob centerlineBlob = blob;
    if (centerlineBlob.centerlinePoints.empty() && !centerlineBlob.contourPoints.empty()) {
        populateCenterlineFromContour(centerlineBlob);
    }

    if (centerlineBlob.centerlinePoints.empty()) {
        return centerlinePoints;
    }

    centerlinePoints.reserve(static_cast<qsizetype>(centerlineBlob.centerlinePoints.size()));
    for (const cv::Point2f& point : centerlineBlob.centerlinePoints) {
        centerlinePoints.append(QPointF(point.x, point.y));
    }

    return centerlinePoints;
}

QList<QPointF> resampleCenterlinePoints(const QList<QPointF>& points, int pointCount)
{
    QList<QPointF> sampledPoints;
    if (points.isEmpty() || pointCount <= 0) {
        return sampledPoints;
    }

    sampledPoints.reserve(pointCount);
    if (points.size() == 1 || pointCount == 1) {
        for (int i = 0; i < pointCount; ++i) {
            sampledPoints.append(points.first());
        }
        return sampledPoints;
    }

    std::vector<double> cumulativeDistance(static_cast<size_t>(points.size()), 0.0);
    for (int i = 1; i < points.size(); ++i) {
        const QPointF delta = points.at(i) - points.at(i - 1);
        cumulativeDistance[static_cast<size_t>(i)] =
            cumulativeDistance[static_cast<size_t>(i - 1)] + std::hypot(delta.x(), delta.y());
    }

    const double totalLength = cumulativeDistance.back();
    if (qFuzzyIsNull(totalLength)) {
        for (int i = 0; i < pointCount; ++i) {
            sampledPoints.append(points.first());
        }
        return sampledPoints;
    }

    int segmentIndex = 1;
    for (int sampleIndex = 0; sampleIndex < pointCount; ++sampleIndex) {
        const double targetDistance =
            totalLength * static_cast<double>(sampleIndex) / static_cast<double>(pointCount - 1);

        while (segmentIndex < points.size() - 1 &&
               cumulativeDistance[static_cast<size_t>(segmentIndex)] < targetDistance) {
            ++segmentIndex;
        }

        const double previousDistance = cumulativeDistance[static_cast<size_t>(segmentIndex - 1)];
        const double nextDistance = cumulativeDistance[static_cast<size_t>(segmentIndex)];
        const double segmentLength = nextDistance - previousDistance;
        const double t = qFuzzyIsNull(segmentLength)
                             ? 0.0
                             : (targetDistance - previousDistance) / segmentLength;

        const QPointF a = points.at(segmentIndex - 1);
        const QPointF b = points.at(segmentIndex);
        sampledPoints.append(a + (b - a) * t);
    }

    return sampledPoints;
}

QList<QPointF> resampleCenterlinePoints(const std::vector<cv::Point2f>& points, int pointCount)
{
    QList<QPointF> convertedPoints;
    convertedPoints.reserve(static_cast<qsizetype>(points.size()));
    for (const cv::Point2f& point : points) {
        convertedPoints.append(QPointF(point.x, point.y));
    }
    return resampleCenterlinePoints(convertedPoints, pointCount);
}

QList<QPointF> extractResampledCenterlinePoints(const DetectedBlob& blob, int pointCount)
{
    return resampleCenterlinePoints(extractOrderedCenterlinePoints(blob), pointCount);
}

// Uses thresholded mat to find the nearest blob to a click and selects it.
DetectedBlob findClickedBlob(const cv::Mat& binaryImage,
                             const QPointF& clickPointVideoCoords,
                             double minArea,
                             double maxArea,
                             double maxDistanceForSelection) {
    DetectedBlob result;
    result.isValid = false;

    if (binaryImage.empty() || binaryImage.type() != CV_8UC1) {
        YAWT_WARN(lcDataCommon) << "findClickedBlob: Invalid input image (empty or not CV_8UC1).";
        return result;
    }

    cv::Point clickCvPoint(qRound(clickPointVideoCoords.x()), qRound(clickPointVideoCoords.y()));
    if (clickCvPoint.x < 0 || clickCvPoint.y < 0 ||
        clickCvPoint.x >= binaryImage.cols || clickCvPoint.y >= binaryImage.rows) {
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: Click out of bounds. Click:"
                                 << clickPointVideoCoords << "Image size:"
                                 << QSize(binaryImage.cols, binaryImage.rows);
        return result;
    }

    const int clickPixel = static_cast<int>(binaryImage.at<uchar>(clickCvPoint));
    YAWT_DEBUG(lcDataCommon) << "findClickedBlob: Click:"
                             << clickPointVideoCoords
                             << "Pixel:" << clickPixel
                             << "Image size:" << QSize(binaryImage.cols, binaryImage.rows)
                             << "Min/Max area:" << minArea << "/" << maxArea
                             << "Max dist:" << maxDistanceForSelection;

    std::vector<std::vector<cv::Point>> contours;
    // Use a copy of binaryImage for findContours if it modifies the input
    cv::findContours(binaryImage.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: No contours found.";
        return result;
    }

    int bestContourIdx = -1;
    double minDistanceSqToCentroid = std::numeric_limits<double>::max();
    bool clickInsideABlob = false;
    int areaPassedCount = 0;
    double minContourArea = std::numeric_limits<double>::max();
    double maxContourArea = 0.0;

    // Pass 1: Check for contours whose bounding box *contains* the click point.
    // Prioritize these. If multiple, could pick smallest area or closest centroid.
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < minContourArea) minContourArea = area;
        if (area > maxContourArea) maxContourArea = area;
        if (area < minArea || area > maxArea) { // Apply area filter
            continue;
        }
        areaPassedCount++;
        cv::Rect br = cv::boundingRect(contours[i]);
        if (br.contains(clickCvPoint)) {
            // This contour is a strong candidate.
            // If we find one, we can potentially stop and use this one.
            // For now, let's take the first valid one we find that contains the click.
            // A more refined approach might be to find the one with the smallest area that contains the click.
            cv::Moments mu = cv::moments(contours[i]);
            if (mu.m00 > 0) { // Check for valid moments
                double distSq = qPow( (mu.m10 / mu.m00) - clickPointVideoCoords.x(), 2) +
                                qPow( (mu.m01 / mu.m00) - clickPointVideoCoords.y(), 2);
                if (distSq < minDistanceSqToCentroid) { // Prefer the one whose centroid is closer if multiple contain click
                    minDistanceSqToCentroid = distSq;
                    bestContourIdx = static_cast<int>(i);
                    clickInsideABlob = true;
                }
            }
        }
    }

    // Pass 2: If click was not inside any blob's bounding box, find the blob with the closest centroid.
    if (!clickInsideABlob) {
        minDistanceSqToCentroid = std::numeric_limits<double>::max(); // Reset for this pass
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < minArea || area > maxArea) { // Apply area filter
                continue;
            }
            cv::Moments mu = cv::moments(contours[i]);
            if (mu.m00 > 0) { // Check for valid moments (non-zero area)
                cv::Point2f centroid(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
                double distSq = qPow(centroid.x - clickPointVideoCoords.x(), 2) +
                                qPow(centroid.y - clickPointVideoCoords.y(), 2);

                if (distSq < minDistanceSqToCentroid) {
                    minDistanceSqToCentroid = distSq;
                    bestContourIdx = static_cast<int>(i);
                }
            }
        }
        // Check if the closest one found is within the maxDistanceForSelection
        if (qSqrt(minDistanceSqToCentroid) > maxDistanceForSelection) {
            bestContourIdx = -1; // Too far, invalidate selection
        }
    }


    // If a suitable contour was found by either method
    if (bestContourIdx != -1) {
        const auto& bestContour = contours[bestContourIdx];
        cv::Moments mu = cv::moments(bestContour);
        // Double check mu.m00 > 0, though area filter should imply this
        if (mu.m00 > 0) {
            result.centroid = QPointF(static_cast<double>(mu.m10 / mu.m00), static_cast<double>(mu.m01 / mu.m00));
            cv::Rect brCv = cv::boundingRect(bestContour);
            result.boundingBox = QRectF(brCv.x, brCv.y, brCv.width, brCv.height);
            result.area = cv::contourArea(bestContour); // Already calculated, but store it
            result.contourPoints = bestContour; // These points are relative to binaryImage origin
            result.isValid = true;
            populateCenterlineFromContour(result);
            // touchesROIboundary is not relevant for findClickedBlob as it operates on the whole image or a pre-defined mask.
        }
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: Selected contour idx:"
                                 << bestContourIdx
                                 << "Centroid:" << result.centroid
                                 << "Area:" << result.area
                                 << "BBox:" << result.boundingBox;
    } else {
        const double minAreaForLog = (minContourArea == std::numeric_limits<double>::max()) ? 0.0 : minContourArea;
        const double dist = (minDistanceSqToCentroid == std::numeric_limits<double>::max())
                                ? -1.0
                                : qSqrt(minDistanceSqToCentroid);
        YAWT_DEBUG(lcDataCommon) << "findClickedBlob: No valid blob."
                                 << "Contours:" << contours.size()
                                 << "Area-passing:" << areaPassedCount
                                 << "Area min/max:" << minAreaForLog << "/" << maxContourArea
                                 << "Click inside bbox:" << clickInsideABlob
                                 << "Nearest centroid dist:" << dist;
    }

    return result;
}


QList<DetectedBlob> findAllPlausibleBlobsInRoi(const cv::Mat& binaryImage,
                                               const QRectF& roiToSearch, // This is in full image coordinates
                                               double minArea,
                                               double maxArea,
                                               double minAspectRatio,
                                               double maxAspectRatio) {
    QList<DetectedBlob> plausibleBlobs;

    if (binaryImage.empty() || binaryImage.type() != CV_8UC1 || roiToSearch.isEmpty() || roiToSearch.width() <=0 || roiToSearch.height() <=0) {
        YAWT_WARN(lcDataCommon) << "findAllPlausibleBlobsInRoi: Invalid input image or ROI.";
        return plausibleBlobs;
    }

    // Define the OpenCV ROI from QRectF (roiToSearch is in full image coordinates)
    cv::Rect roiCv(static_cast<int>(qRound(roiToSearch.x())),
                   static_cast<int>(qRound(roiToSearch.y())),
                   static_cast<int>(qRound(roiToSearch.width())),
                   static_cast<int>(qRound(roiToSearch.height())));

    // Ensure ROI is within the image boundaries
    // This creates the actual ROI that will be used on binaryImage
    cv::Rect actualRoiCv = roiCv & cv::Rect(0, 0, binaryImage.cols, binaryImage.rows);

    if (actualRoiCv.width <= 0 || actualRoiCv.height <= 0) {
        // qDebug() << "findAllPlausibleBlobsInRoi: ROI after clamping is invalid or outside image.";
        return plausibleBlobs; // ROI is outside image or has no area
    }

    cv::Mat roiImage = binaryImage(actualRoiCv); // Extract the sub-image for contour finding
    std::vector<std::vector<cv::Point>> contoursInSubImage;
    std::vector<cv::Vec4i> hierarchy;
    // RETR_CCOMP gives a 2-level hierarchy (outer contours + their holes).
    // This lets us subtract hole areas from outer contour areas, so a coiled worm
    // whose thresholded shape is a ring is measured by actual pixel area rather than
    // the much-larger disk area that RETR_EXTERNAL + contourArea would produce.
    cv::findContours(roiImage.clone(), contoursInSubImage, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contoursInSubImage.size(); ++i) {
        // Only process outer contours (parent index == -1 in RETR_CCOMP)
        if (hierarchy[i][3] != -1) continue;

        const auto& contourInSub = contoursInSubImage[i];
        double outerArea = cv::contourArea(contourInSub);

        // Subtract areas of direct-child hole contours to get true foreground pixel area.
        // This handles the ring topology produced by a self-touching coiled worm.
        double holeArea = 0.0;
        for (int childIdx = hierarchy[i][2]; childIdx != -1; childIdx = hierarchy[childIdx][0]) {
            holeArea += cv::contourArea(contoursInSubImage[childIdx]);
        }
        double area = outerArea - holeArea;

        // Calculate convex hull area using the outer boundary
        std::vector<cv::Point> hull;
        cv::convexHull(contourInSub, hull);
        double hullArea = cv::contourArea(hull);

        if (area < minArea || area > maxArea) {
            continue; // Filter by area
        }

        // Bounding box of the contour, relative to roiImage (the sub-image)
        cv::Rect brInSub = cv::boundingRect(contourInSub);
        if (brInSub.width == 0 || brInSub.height == 0) {
            continue; // Skip zero-dimension bounding boxes
        }

        // Aspect ratio (using dimensions from brInSub)
        double currentAspectRatio = static_cast<double>(brInSub.width) / static_cast<double>(brInSub.height);
        if (currentAspectRatio < 1.0) {
            currentAspectRatio = 1.0 / currentAspectRatio; // Ensure aspect ratio is >= 1
        }

        // Aspect ratio filter (currently commented out in your provided code)
        // if (currentAspectRatio < minAspectRatio || currentAspectRatio > maxAspectRatio) {
        //     continue;
        // }

        cv::Moments mu = cv::moments(contourInSub);
        if (mu.m00 > 0) { // Check for valid moments (non-zero area)
            DetectedBlob blob;
            blob.isValid = true;
            blob.area = area;
            blob.convexHullArea = hullArea;

            // Convert centroid and bounding box to full image coordinates
            // Centroid in sub-image: (mu.m10 / mu.m00), (mu.m01 / mu.m00)
            // Add actualRoiCv.x and actualRoiCv.y to convert to full image coordinates
            blob.centroid = QPointF(actualRoiCv.x + (mu.m10 / mu.m00),
                                    actualRoiCv.y + (mu.m01 / mu.m00));

            // Bounding box in sub-image: brInSub
            // Add actualRoiCv.x and actualRoiCv.y to convert to full image coordinates
            blob.boundingBox = QRectF(actualRoiCv.x + brInSub.x,
                                      actualRoiCv.y + brInSub.y,
                                      brInSub.width,
                                      brInSub.height);

            // Offset outer contour points to full frame coordinates
            blob.contourPoints.reserve(contourInSub.size());
            for(const cv::Point& ptInSub : contourInSub) {
                blob.contourPoints.push_back(cv::Point(ptInSub.x + actualRoiCv.x, ptInSub.y + actualRoiCv.y));
            }

            // Offset hole contour points to full frame coordinates
            for (int childIdx = hierarchy[i][2]; childIdx != -1; childIdx = hierarchy[childIdx][0]) {
                std::vector<cv::Point> holeInFullFrame;
                holeInFullFrame.reserve(contoursInSubImage[childIdx].size());
                for (const cv::Point& ptInSub : contoursInSubImage[childIdx]) {
                    holeInFullFrame.push_back(cv::Point(ptInSub.x + actualRoiCv.x, ptInSub.y + actualRoiCv.y));
                }
                blob.holeContourPoints.push_back(std::move(holeInFullFrame));
            }

            // --- Set touchesROIboundary flag ---
            // Check if the bounding box of the contour (brInSub, which is relative to roiImage)
            // touches the edges of roiImage.
            // roiImage has dimensions actualRoiCv.width and actualRoiCv.height.
            // Note: actualRoiCv.width and actualRoiCv.height are the dimensions of roiImage.
            if (brInSub.x <= 0 ||
                brInSub.y <= 0 ||
                (brInSub.x + brInSub.width) >= actualRoiCv.width ||
                (brInSub.y + brInSub.height) >= actualRoiCv.height) {
                blob.touchesROIboundary = true;
            } else {
                blob.touchesROIboundary = false;
            }
            // A more precise check could iterate over contour points if needed, but bounding box is usually sufficient.
            // For example, if any point in contourInSub has x=0, y=0, x=actualRoiCv.width-1, or y=actualRoiCv.height-1.
            // However, the bounding box check is simpler and often what's implied.

            populateCenterlineFromContour(blob);
            plausibleBlobs.append(blob);
        }
    }
    std::sort(plausibleBlobs.begin(), plausibleBlobs.end(), [](const DetectedBlob& a, const DetectedBlob& b) {
        return a.area > b.area; // For descending order; largest blob first
    });
    return plausibleBlobs;
}


} // namespace Tracking
