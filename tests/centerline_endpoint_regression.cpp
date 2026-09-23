#include "core/centerlinegeometry.h"
#include "core/centerlineprocessor.h"
#include "core/centerlineroutes.h"
#include "data/trackingdatastorage.h"
#include "debug/debugdatastore.h"
#include "debug/debugexporter.h"
#include <QCoreApplication>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <map>
#include <stdexcept>
#include "truncated_head_contours.h"
#include "visible_cap_contours.h"
#include "oblique_tail_contours.h"
#include "self_contact_sequence_contours.h"

static void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
static bool same(cv::Point2f a, cv::Point2f b) { return cv::norm(a - b) < 1e-4; }

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    try {
        // Rounded, oblique caps expose the difference between a single contour
        // pixel and a subpixel axis boundary estimate.
        cv::Mat mask = cv::Mat::zeros(120, 180, CV_8U);
        cv::ellipse(mask, {90, 60}, {55, 9}, 17, 0, 360, cv::Scalar(255), -1);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        Tracking::DetectedBlob blob;
        blob.isValid = true;
        blob.contourPoints = contours.front();
        blob.centroid = {90, 60};
        Debug::EndpointDebug debug;
        auto endpoints = Centerline::detectEndpoints(blob, {}, {}, false, &debug);
        require(endpoints.topology == Tracking::TopologyState::Clean, "fixture is not clean");
        require(endpoints.tips.size() == 2, "expected two tips");
        bool distinct = false;
        for (size_t i = 0; i < endpoints.tips.size(); ++i) {
            const auto& tip = endpoints.tips[i];
            const auto& cap = debug.tipCapDebug[i];
            require(tip.selectedAxis, "clean tip did not select axis boundary");
            require(std::abs(cv::pointPolygonTest(blob.contourPoints, tip.point, true)) < 1e-3, "selected tip is off boundary");
            require(same(cap.skelEndpoint, debug.endpointCandidateDebug[i].skeletonVideo), "debug search origin is not raw skeleton");
            require(same(cap.selectedPoint, tip.point), "debug selection differs from detector");
            require(cap.selectedEstimator == "axis boundary", "wrong selected estimator");
            distinct |= !same(tip.point, cap.peakOrSnapPoint);
        }
        require(distinct, "fixture must distinguish axis boundary from peak/snap");
        // Preserve the established non-clean endpoint behavior.
        Debug::EndpointDebug mergedDebug;
        auto merged = Centerline::detectEndpoints(blob, {}, {}, true, &mergedDebug);
        for (size_t i = 0; i < merged.tips.size(); ++i) {
            require(!merged.tips[i].selectedAxis, "merged tip changed estimator");
            require(same(merged.tips[i].point, mergedDebug.tipCapDebug[i].peakOrSnapPoint), "merged tip changed position");
        }
        Tracking::DetectedBlob crossed = blob;
        crossed.holeContourPoints.push_back({{86, 58}, {93, 59}, {92, 62}, {85, 61}});
        Debug::EndpointDebug crossedDebug;
        auto crossedEndpoints = Centerline::detectEndpoints(crossed, {}, {}, false, &crossedDebug);
        require(crossedEndpoints.topology == Tracking::TopologyState::SelfCrossed, "hole fixture is not self-crossed");
        require(!crossedEndpoints.tips.empty(), "hole fixture must retain a visible tip");
        for (size_t i = 0; i < crossedEndpoints.tips.size(); ++i) {
            require(!crossedEndpoints.tips[i].selectedAxis, "self-crossed tip changed estimator");
            require(same(crossedEndpoints.tips[i].point, crossedDebug.tipCapDebug[i].peakOrSnapPoint), "self-crossed tip changed position");
        }
        Tracking::Track points(1);
        points[0].frameNumber = 0;
        points[0].position = {90, 60};
        points[0].quality = Tracking::TrackPointQuality::Single;
        Centerline::CenterlineFrameContext context;
        context.wormId = 1;
        context.sortedPoints = &points;
        context.nPts = 20;
        context.captureDebug = true;
        Centerline::CenterlineSweepState state;
        Centerline::CenterlineFrameIo io;
        io.getDetectedBlobsForFrame = [&](int) { return QMap<int, Tracking::DetectedBlob>{{1, blob}}; };
        io.getMergeGroupsForFrame = [](int) { return QList<Tracking::MergeGroup>{}; };
        io.getTipBaseline = [](int) { return Centerline::TipFeatureBaseline{}; };
        io.setDetectedBlobForFrame = [](int, int, const auto&) {};
        io.recordTipFeatureSample = [](int, float, float) {};
        io.recordBodyLengthSample = [](int, float) {};
        io.setCenterlineDebugFrame = [](const auto&) {};
        auto result = Centerline::processFrame(context, {0, 1, true}, state, io);
        require(result.processed && result.debugRecord.snakeRan, "clean snake path did not run");
        require(result.debugRecord.tipCapRoles[0] == "head" && result.debugRecord.tipCapRoles[1] == "tail", "bootstrap DEBUG roles are stale");
        const auto& cl = result.blob.centerline;
        const auto head = cl.tipCandidates[cl.headTipIdx];
        const auto tail = cl.tipCandidates[cl.tailTipIdx];
        require(head.source == Tracking::TipCandidate::Source::AxisBoundary && tail.source == head.source, "stored estimator lost");
        require(same(cl.points.front(), head.point) && same(cl.points.back(), tail.point), "snake changed selected endpoints");
        require(same(state.predictor.lastHeadPos, head.point) && same(state.predictor.lastTailPos, tail.point), "predictor changed selected endpoints");
        require(same(result.debugRecord.initialCenterline.front(), cl.points.front()), "initial and final head differ");
        require(same(result.debugRecord.initialCenterline.back(), cl.points.back()), "initial and final tail differ");
        require(Centerline::relaxCenterlineToSmoothedMidpoint(result.blob, {90, 60}, 20, {}), "midpoint relaxation failed");
        require(same(result.blob.centerline.points.front(), head.point) && same(result.blob.centerline.points.back(), tail.point), "midpoint relaxation changed tips");
        if (argc > 1) {
            TrackingDataStorage storage;
            storage.setDetectedBlobForFrame(0, 1, result.blob);
            Debug::DebugDataStore store;
            store.setCenterlineFrame(result.debugRecord);
            QString error;
            require(Debug::DebugExporter::exportCenterlineFrame(&storage, &store, 1, 0, argv[1], &error), qPrintable(error));
        }
        for (size_t frame = 0; frame < truncatedHeadContours.size(); ++frame) {
            blob = Tracking::DetectedBlob{};
            blob.isValid = true;
            blob.contourPoints = truncatedHeadContours[frame];
            blob.centroid = {826, 968};
            state = Centerline::CenterlineSweepState{};
            auto actual = Centerline::processFrame(context, {0, 1, true}, state, io);
            require(actual.processed && actual.debugRecord.snakeRan, "recorded contour did not run clean snake path");
            const auto& caps = actual.debugRecord.tipCapDebug;
            require(caps.size() == 2, "recorded contour lost a tip");
            for (const auto& cap : caps) {
                require(cv::norm(cap.outwardDir) > 0.99, "recorded contour direction collapsed");
                require(cv::norm(cap.selectedPoint - cap.skelEndpoint) < 5, "recorded tip jumped away from its skeleton endpoint");
            }
            const auto& line = actual.blob.centerline;
            require(line.points.front().x < 812 && line.points.back().x > 840,
                    "recorded centerline is truncated");
            require(same(line.points.front(), line.tipCandidates[line.headTipIdx].point), "recorded head pin mismatch");
            std::cout << "Frame " << 1383 + frame << " head=" << line.points.front()
                      << " tail=" << line.points.back() << '\n';
        }
        for (size_t frame = 0; frame < visibleCapContours.size(); ++frame) {
            blob = Tracking::DetectedBlob{};
            blob.isValid = true;
            blob.contourPoints = visibleCapContours[frame];
            blob.centroid = {947, 853};
            state = Centerline::CenterlineSweepState{};
            auto actual = Centerline::processFrame(context, {0, 1, true}, state, io);
            const auto& line = actual.blob.centerline;
            require(actual.processed && actual.debugRecord.snakeRan, "visible cap fixture did not run");
            require(line.points.front().y < 842 && line.points.back().y > 870, "visible cap points are not terminal");
            for (const auto& tip : line.tipCandidates) {
                require(tip.source == Tracking::TipCandidate::Source::AxisBoundary, "visible cap switched estimators");
                require(std::abs(cv::pointPolygonTest(blob.contourPoints, tip.point, true)) < 1e-3,
                        "visible tip does not lie on contour");
            }
            // Geometry must not depend on which collinear contour vertices
            // CHAIN_APPROX_SIMPLE happens to retain.
            auto denseBlob = blob;
            denseBlob.contourPoints.clear();
            const auto& contour = blob.contourPoints;
            for (size_t j = 0; j < contour.size(); ++j) {
                const auto a = contour[j], b = contour[(j + 1) % contour.size()];
                const int steps = std::max(std::abs(b.x-a.x), std::abs(b.y-a.y));
                for (int k = 0; k < steps; ++k)
                    denseBlob.contourPoints.emplace_back(a.x + (b.x-a.x)*k/steps, a.y + (b.y-a.y)*k/steps);
            }
            auto dense = Centerline::detectEndpoints(denseBlob, {}, {}, false);
            auto sparse = Centerline::detectEndpoints(blob, {}, {}, false);
            require(dense.tips.size() == sparse.tips.size(), "contour sampling changed tip count");
            for (size_t j = 0; j < sparse.tips.size(); ++j)
                require(same(dense.tips[j].point, sparse.tips[j].point), "contour sampling changed selected position");
            std::reverse(denseBlob.contourPoints.begin(), denseBlob.contourPoints.end());
            dense = Centerline::detectEndpoints(denseBlob, {}, {}, false);
            for (size_t j = 0; j < sparse.tips.size(); ++j)
                require(same(dense.tips[j].point, sparse.tips[j].point), "contour traversal changed selected position");
            std::cout << "Frame " << 1534 + frame << " head=" << line.points.front()
                      << " tail=" << line.points.back() << '\n';
            if (argc > 1) {
                TrackingDataStorage storage;
                storage.setDetectedBlobForFrame(0, 1, actual.blob);
                Debug::DebugDataStore store;
                store.setCenterlineFrame(actual.debugRecord);
                QString error;
                require(Debug::DebugExporter::exportCenterlineFrame(&storage, &store, 1, 0,
                    QString::fromLocal8Bit(argv[1]) + QStringLiteral("/frame%1").arg(1534 + frame), &error), qPrintable(error));
            }
        }
        // Thinning must keep an oblique tail when only its last pixels change.
        // Frame 1564 alone, and frame 1563 minus the two terminal pixels that
        // vanish in 1564, both used to retreat the tail endpoint to (946,861).
        auto requireObliqueTail = [&](const cv::Mat& tailMask, const cv::Rect& bounds, const char* label) {
            const auto graph = Centerline::buildSkeletonGraph(tailMask);
            require(graph.endpointIndices.size() == 2, label);
            const auto search = Centerline::dijkstraSkeleton(graph.points, graph.adjacency, 0);
            for (double d : search.distances)
                require(std::isfinite(d), "oblique tail skeleton is disconnected");
            cv::Point tail = graph.points[graph.endpointIndices[0]];
            for (int idx : graph.endpointIndices)
                if (graph.points[idx].y > tail.y) tail = graph.points[idx];
            tail += bounds.tl();
            std::cout << label << " skeleton tail=" << tail << '\n';
            require(tail.x >= 950 && tail.y >= 863, "skeleton tail retreated along the diagonal");
        };
        for (size_t frame = 0; frame < obliqueTailContours.size(); ++frame) {
            Tracking::DetectedBlob tailBlob;
            tailBlob.isValid = true;
            tailBlob.contourPoints = obliqueTailContours[frame];
            cv::Mat tailMask;
            const cv::Rect bounds = Centerline::buildCenterlineMask(tailBlob, tailMask);
            requireObliqueTail(tailMask, bounds, frame == 0 ? "frame 1563" : "frame 1564");
            if (frame == 0) {
                require(tailMask.at<uchar>(866 - bounds.y, 951 - bounds.x) && tailMask.at<uchar>(865 - bounds.y, 952 - bounds.x),
                        "oblique tail fixture lost its terminal pixels");
                tailMask.at<uchar>(866 - bounds.y, 951 - bounds.x) = 0;
                tailMask.at<uchar>(865 - bounds.y, 952 - bounds.x) = 0;
                requireObliqueTail(tailMask, bounds, "frame 1563 minus terminal pixels");
            }
            blob = tailBlob;
            blob.centroid = {946, 846};
            state = Centerline::CenterlineSweepState{};
            auto actual = Centerline::processFrame(context, {0, 1, true}, state, io);
            require(actual.processed && actual.debugRecord.snakeRan, "oblique tail fixture did not run clean snake path");
            const auto& line = actual.blob.centerline;
            const auto tailTip = line.tipCandidates[line.tailTipIdx].point.y > line.tipCandidates[line.headTipIdx].point.y
                ? line.tipCandidates[line.tailTipIdx].point : line.tipCandidates[line.headTipIdx].point;
            std::cout << "Frame " << 1563 + frame << " selected tail=" << tailTip << '\n';
            require(tailTip.x > 949.5 && tailTip.y > 864.5, "selected tail left the oblique terminal cap");
        }
        // Self-contact sequences, swept backward from a clean frame as the
        // worker does. Every self-crossed centerline must fit the learned body
        // length (or be left unresolved), and each continuously visible end
        // must keep its role. Ground truth points are that end's skeleton
        // endpoint, traced frame to frame independently of the pipeline.
        struct RoleTrace { const char* name; std::vector<std::pair<int, cv::Point2f>> points; };
        auto runSequence = [&](int firstFrame, int lastFrame, const std::vector<RoleTrace>& traces) {
            std::map<int, Tracking::DetectedBlob> blobs;
            Tracking::Track track;
            for (const auto& seq : selfContactSequence) {
                if (seq.frame < firstFrame || seq.frame > lastFrame) continue;
                Tracking::DetectedBlob b;
                b.isValid = true;
                b.contourPoints = seq.contour;
                b.holeContourPoints = seq.holes;
                const cv::Moments m = cv::moments(seq.contour);
                b.centroid = {m.m10 / m.m00, m.m01 / m.m00};
                blobs[seq.frame] = b;
                Tracking::TrackPoint tp;
                tp.frameNumber = seq.frame;
                tp.position = {static_cast<float>(b.centroid.x()), static_cast<float>(b.centroid.y())};
                tp.quality = Tracking::TrackPointQuality::Single;
                track.push_back(tp);
            }
            Centerline::TipFeatureBaseline baseline;   // worm 5 baseline from the recorded run
            baseline.meanBodyLength = 39.7182f; baseline.lengthSamples = 1273;
            baseline.meanAbsCurvature = 0.322216f; baseline.curvatureSamples = 2546;
            baseline.meanWidth = 0.475839f; baseline.widthSamples = 2546;
            Centerline::CenterlineFrameContext seqContext = context;
            seqContext.wormId = 5;
            seqContext.sortedPoints = &track;
            seqContext.refLength = baseline.meanBodyLength;
            Centerline::CenterlineFrameIo seqIo = io;
            seqIo.getDetectedBlobsForFrame = [&](int f) {
                QMap<int, Tracking::DetectedBlob> out;
                if (blobs.count(f)) out.insert(5, blobs[f]);
                return out;
            };
            seqIo.setDetectedBlobForFrame = [&](int f, int, const auto& b) { blobs[f] = b; };
            seqIo.getTipBaseline = [&](int) { return baseline; };
            std::map<int, Debug::CenterlineFrameDebug> records;
            Centerline::CenterlineSweepState seqState;
            for (int idx = static_cast<int>(track.size()) - 1; idx >= 0; --idx) {
                const bool bootstrap = idx == static_cast<int>(track.size()) - 1;
                auto r = Centerline::processFrame(seqContext, {idx, -1, bootstrap}, seqState, seqIo);
                require(r.processed, "sequence frame was not processed");
                records[track[idx].frameNumber] = r.debugRecord;
            }
            auto roleNear = [&](int f, const cv::Point2f& p) -> char {
                const auto& cl = blobs[f].centerline;
                int best = -1;
                float bestDist = 4.f;
                for (int k = 0; k < static_cast<int>(cl.tipCandidates.size()); ++k) {
                    if (cl.tipCandidates[k].source == Tracking::TipCandidate::Source::HypothesizedHidden) continue;
                    const float d = cv::norm(cl.tipCandidates[k].point - p);
                    if (d <= bestDist) { bestDist = d; best = k; }
                }
                if (best >= 0 && best == cl.headTipIdx) return 'H';
                if (best >= 0 && best == cl.tailTipIdx) return 'T';
                return '-';
            };
            int unresolved = 0;
            for (int f = lastFrame; f >= firstFrame; --f) {
                const auto& rec = records[f];
                const auto& cl = blobs[f].centerline;
                const std::vector<cv::Point2f> pts(cl.points.begin(), cl.points.end());
                const float length = pts.size() >= 2 ? Centerline::resampledArcLength(pts, seqContext.nPts) : 0.f;
                if (rec.topology == Tracking::TopologyState::SelfCrossed) {
                    if (pts.size() < 2) ++unresolved;
                    else require(length >= Centerline::kRouteMinLengthFraction * baseline.meanBodyLength - 0.5f &&
                                 length <= Centerline::kRouteMaxLengthFraction * baseline.meanBodyLength + 0.5f,
                                 "self-crossed centerline outside the body-length window");
                }
                std::cout << "Frame " << f << ' ' << qPrintable(Debug::centerlineBranchToString(rec.branch))
                          << " len=" << length;
                if (pts.size() >= 2) std::cout << " head=" << pts.front() << " tail=" << pts.back();
                std::cout << '\n';
            }
            if (argc > 1) {
                for (const auto& [f, rec] : records) {
                    TrackingDataStorage storage;
                    storage.setDetectedBlobForFrame(f, 5, blobs[f]);
                    if (blobs.count(f + 1)) storage.setDetectedBlobForFrame(f + 1, 5, blobs[f + 1]);
                    Debug::DebugDataStore store;
                    store.setCenterlineFrame(rec);
                    QString error;
                    require(Debug::DebugExporter::exportCenterlineFrame(&storage, &store, 5, f,
                        QString::fromLocal8Bit(argv[1]) + QStringLiteral("/worm5_frame%1").arg(f), &error),
                        qPrintable(error));
                }
            }
            for (const auto& trace : traces) {
                const char expected = roleNear(trace.points.front().first, trace.points.front().second);
                require(expected != '-', "role trace anchor has no role");
                std::cout << trace.name << " anchor role " << expected << ':';
                for (const auto& [f, p] : trace.points) {
                    const char role = roleNear(f, p);
                    std::cout << ' ' << f << role;
                    if (role != expected) {
                        std::cout << '\n';
                        throw std::runtime_error(std::string(trace.name) + " changed role at frame " + std::to_string(f));
                    }
                }
                std::cout << '\n';
            }
            std::cout << "Frames " << firstFrame << '-' << lastFrame << " unresolved self-crossed frames: "
                      << unresolved << '\n';
            return unresolved;
        };
        // Set 2: the end at the top right of frame 590 stays visible through
        // the whole omega turn while the other end is hidden in the loop.
        RoleTrace visibleEnd{"set 543-590 visible end", {
            {590, {669, 992}}, {589, {669, 995}}, {588, {670, 998}}, {587, {671, 1001}},
            {585, {680, 1011}}, {584, {683, 1011}}, {583, {683, 1011}}, {582, {682, 1008}},
            {581, {681, 1008}}, {580, {679, 1009}}, {579, {680, 1013}}, {578, {680, 1013}},
            {577, {680, 1015}}, {576, {678, 1016}}, {575, {674, 1014}}, {574, {673, 1012}},
            {573, {674, 1011}}, {572, {674, 1011}}, {571, {673, 1008}}, {570, {675, 1007}},
            {569, {674, 1007}}, {568, {676, 1005}}, {567, {676, 1004}}, {566, {676, 1003}},
            {565, {676, 1003}}, {564, {677, 1002}}, {563, {677, 1002}}, {562, {677, 1001}},
            {561, {677, 1001}}, {560, {678, 1000}}, {559, {679, 1000}}, {558, {679, 1000}},
            {557, {677, 1001}}, {556, {677, 1001}}, {555, {677, 1001}}, {554, {677, 999}},
            {553, {678, 998}}, {552, {678, 998}}, {551, {678, 998}}, {550, {677, 999}},
            {549, {678, 998}}, {548, {677, 998}}, {547, {677, 998}}}};
        const int unresolvedSet2 = runSequence(543, 590, {visibleEnd});
        // Set 1: the right end of 791 stays visible until 787; a second visible
        // stretch runs from 783 to the clean frames at 767-766. Which physical
        // end reappears after the 786-784 contact is not established, so the
        // two stretches are checked separately.
        RoleTrace entering{"set 766-791 entering end", {
            {791, {616, 988}}, {790, {614, 986}}, {789, {610, 985}}, {788, {606, 984}}, {787, {603, 985}}}};
        RoleTrace leaving{"set 766-791 leaving end", {
            {783, {604, 992}}, {782, {603, 996}}, {781, {605, 995}}, {780, {606, 997}},
            {779, {607, 1001}}, {778, {605, 1005}}, {777, {602, 1004}}, {776, {602, 1004}},
            {775, {603, 1001}}, {774, {604, 997}}, {773, {606, 996}}, {772, {606, 996}},
            {771, {607, 996}}, {770, {608, 995}}, {769, {608, 995}}, {768, {607, 994}},
            {767, {607, 991}}, {766, {607, 991}}}};
        const int unresolvedSet1 = runSequence(766, 791, {entering, leaving});
        require(unresolvedSet1 + unresolvedSet2 <= 6, "too many unresolved self-crossed frames");
        std::cout << "Endpoint regression checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
