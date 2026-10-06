#include "core/centerlinegeometry.h"
#include "core/centerlineprocessor.h"
#include "core/centerlineroutes.h"
#include "core/centerlinetrack.h"
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
        struct SequenceSpec {
            const std::vector<SequenceFrame>* frames;
            int wormId;
            float bodyLength;     // recorded clean-frame baseline for that worm
            int minIslandFrames;
        };
        const SequenceSpec worm5Spec{&selfContactSequence, 5, 39.7182f, 1};
        const SequenceSpec worm3Spec{&worm3ContactSequence, 3, 45.0f, 4};
        auto runSequence = [&](const SequenceSpec& spec, int firstFrame, int lastFrame,
                               const std::vector<RoleTrace>& traces, bool expectSingleChain) {
            std::map<int, Tracking::DetectedBlob> blobs;
            Tracking::Track track;
            for (const auto& seq : *spec.frames) {
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
            Centerline::TipFeatureBaseline baseline;   // baseline from the recorded run
            baseline.meanBodyLength = spec.bodyLength; baseline.lengthSamples = 1273;
            baseline.meanAbsCurvature = 0.322216f; baseline.curvatureSamples = 2546;
            baseline.meanWidth = 0.475839f; baseline.widthSamples = 2546;
            Centerline::CenterlineFrameContext seqContext = context;
            seqContext.wormId = spec.wormId;
            seqContext.sortedPoints = &track;
            seqContext.refLength = baseline.meanBodyLength;
            Centerline::CenterlineFrameIo seqIo = io;
            seqIo.getDetectedBlobsForFrame = [&](int f) {
                QMap<int, Tracking::DetectedBlob> out;
                if (blobs.count(f)) out.insert(spec.wormId, blobs[f]);
                return out;
            };
            seqIo.setDetectedBlobForFrame = [&](int f, int, const auto& b) { blobs[f] = b; };
            seqIo.getTipBaseline = [&](int) { return baseline; };
            std::map<int, Debug::CenterlineFrameDebug> records;
            seqIo.setCenterlineDebugFrame = [&](const Debug::CenterlineFrameDebug& rec) { records[rec.frameNumber] = rec; };
            seqIo.getCenterlineDebugFrame = [&](int, int f, Debug::CenterlineFrameDebug& out) {
                if (!records.count(f)) return false;
                out = records[f];
                return true;
            };
            seqContext.captureDebug = true;
            Centerline::TrackPassConfig passConfig;
            passConfig.minIslandFrames = spec.minIslandFrames;
            const auto continuity = Centerline::processTrackContinuity(seqContext, seqIo, passConfig);
            for (const QString& line : continuity.log) std::cout << qPrintable(line) << '\n';
            // With no motion evidence in a short fixture, name the first chain
            // and let continuity carry it across weak bridges, as pass 3 does.
            if (continuity.chains.size() > 1) {
                std::vector<bool> decided(continuity.chains.size(), false), flipped(continuity.chains.size(), false);
                decided[0] = true;
                Centerline::propagateAcrossWeakLinks(seqIo, spec.wormId, continuity, decided, flipped);
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
            // Bridges use the neighbouring clean frames' length, not the run baseline.
            std::vector<float> cleanLengths;
            for (int f = firstFrame; f <= lastFrame; ++f) {
                const auto& cl = blobs[f].centerline;
                if (cl.topology == Tracking::TopologyState::Clean && cl.points.size() >= 2)
                    cleanLengths.push_back(Centerline::resampledArcLength(
                        std::vector<cv::Point2f>(cl.points.begin(), cl.points.end()), seqContext.nPts));
            }
            std::sort(cleanLengths.begin(), cleanLengths.end());
            const float localLength = cleanLengths.empty() ? baseline.meanBodyLength
                                                           : cleanLengths[cleanLengths.size() / 2];
            int unresolved = 0;
            for (int f = lastFrame; f >= firstFrame; --f) {
                const auto& rec = records[f];
                const auto& cl = blobs[f].centerline;
                const std::vector<cv::Point2f> pts(cl.points.begin(), cl.points.end());
                const float length = pts.size() >= 2 ? Centerline::resampledArcLength(pts, seqContext.nPts) : 0.f;
                if (rec.topology == Tracking::TopologyState::SelfCrossed) {
                    if (pts.size() < 2) ++unresolved;
                    else require(length >= Centerline::kRouteMinLengthFraction * localLength - 2.f &&
                                 length <= Centerline::kRouteMaxLengthFraction * localLength + 2.f,
                                 "self-crossed centerline outside the local body-length window");
                }
                std::cout << "Frame " << f << ' ' << qPrintable(Debug::centerlineBranchToString(rec.branch))
                          << " len=" << length;
                if (pts.size() >= 2) std::cout << " head=" << pts.front() << " tail=" << pts.back();
                std::cout << '\n';
            }
            if (argc > 1) {
                for (const auto& [f, rec] : records) {
                    TrackingDataStorage storage;
                    storage.setDetectedBlobForFrame(f, spec.wormId, blobs[f]);
                    if (blobs.count(f + 1)) storage.setDetectedBlobForFrame(f + 1, spec.wormId, blobs[f + 1]);
                    Debug::DebugDataStore store;
                    store.setCenterlineFrame(rec);
                    QString error;
                    require(Debug::DebugExporter::exportCenterlineFrame(&storage, &store, spec.wormId, f,
                        QString::fromLocal8Bit(argv[1]) + QStringLiteral("/worm%1_frame%2").arg(spec.wormId).arg(f), &error),
                        qPrintable(error));
                }
            }
            if (expectSingleChain) {
                require(continuity.chains.size() == 1, "sequence split into more than one continuity chain");
            } else if (continuity.chains.size() > 1) {
                // A broken chain must leave its bridge flagged for review.
                for (size_t c = 0; c + 1 < continuity.chains.size(); ++c)
                    require(continuity.review.count(continuity.chains[c].back()) > 0,
                            "continuity chain broke without flagging the bridge for review");
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
        const int unresolvedSet2 = runSequence(worm5Spec, 543, 590, {visibleEnd}, true);
        // Set 1: the right end of 791 stays visible until 787; a second visible
        // stretch runs from 783 to the clean frames at 767-766. Which physical
        // end reappears after the 786-784 contact is not established, so the
        // bridge may be flagged instead of trusted, and the two stretches are
        // checked separately.
        RoleTrace entering{"set 766-791 entering end", {
            {790, {614, 986}}, {789, {610, 985}}, {788, {606, 984}}, {787, {603, 985}}}};
        RoleTrace leaving{"set 766-791 leaving end", {
            {783, {604, 992}}, {782, {603, 996}}, {781, {605, 995}}, {780, {606, 997}},
            {779, {607, 1001}}, {778, {605, 1005}}, {777, {602, 1004}}, {776, {602, 1004}},
            {775, {603, 1001}}, {774, {604, 997}}, {773, {606, 996}}, {772, {606, 996}},
            {771, {607, 996}}, {770, {608, 995}}, {769, {608, 995}}, {768, {607, 994}},
            {767, {607, 991}}, {766, {607, 991}}}};
        const int unresolvedSet1 = runSequence(worm5Spec, 766, 791, {entering, leaving}, false);
        // Worm 3: one end stays visible through each of these contacts. Earlier
        // versions swapped it at 397 (split tip), 1120 (loop orientation) and
        // around 1054 (end crossing the neck).
        RoleTrace omega{"worm 3 360-415 visible end", {
            {411, {760, 1032}}, {410, {760, 1037}}, {409, {758, 1036}}, {408, {758, 1038}}, {407, {762, 1038}},
            {406, {765, 1042}}, {405, {765, 1042}}, {404, {764, 1043}}, {403, {764, 1042}}, {402, {764, 1044}},
            {400, {760, 1047}}, {399, {757, 1048}}, {398, {755, 1051}}, {397, {761, 1053}}, {396, {762, 1055}},
            {394, {763, 1054}}, {393, {763, 1054}}, {392, {763, 1053}}, {391, {763, 1051}}, {390, {762, 1050}},
            {389, {761, 1050}}, {388, {762, 1051}}, {387, {762, 1052}}, {386, {765, 1053}}, {385, {765, 1054}},
            {384, {766, 1054}}, {383, {766, 1055}}, {382, {769, 1053}}, {381, {770, 1052}}, {380, {772, 1050}},
            {379, {774, 1050}}, {378, {774, 1051}}, {377, {774, 1051}}, {376, {773, 1051}}, {375, {771, 1050}},
            {374, {771, 1050}}, {373, {770, 1049}}, {372, {769, 1049}}, {371, {767, 1049}}, {370, {766, 1049}},
            {369, {765, 1049}}, {368, {764, 1050}}, {367, {764, 1050}}, {366, {763, 1050}}, {365, {763, 1052}}}};
        const int unresolvedWorm3a = runSequence(worm3Spec, 360, 415, {omega}, false);
        RoleTrace crossing{"worm 3 1036-1066 visible end", {
            {1064, {495, 990}}, {1063, {493, 991}}, {1062, {493, 994}}, {1061, {494, 998}}, {1060, {495, 1000}},
            {1059, {499, 1001}}, {1058, {501, 999}}, {1057, {502, 996}}, {1055, {500, 996}}, {1054, {499, 996}},
            {1053, {499, 995}}, {1052, {495, 995}}, {1051, {494, 995}}, {1050, {493, 995}}, {1049, {492, 995}},
            {1048, {491, 995}}, {1047, {491, 995}}, {1046, {491, 995}}, {1045, {491, 995}}, {1044, {492, 995}},
            {1043, {491, 995}}, {1042, {489, 997}}, {1041, {489, 997}}, {1040, {489, 1000}}}};
        const int unresolvedWorm3b = runSequence(worm3Spec, 1036, 1066, {crossing}, true);
        RoleTrace orientationFlip{"worm 3 1100-1135 visible end", {
            {1131, {546, 1007}}, {1130, {546, 1009}}, {1129, {546, 1011}}, {1128, {546, 1011}}, {1127, {544, 1013}},
            {1126, {544, 1016}}, {1125, {541, 1018}}, {1124, {539, 1016}}, {1123, {539, 1016}}, {1122, {540, 1014}},
            {1121, {541, 1012}}, {1120, {542, 1006}}, {1119, {544, 1004}}, {1117, {544, 1003}}, {1116, {544, 1003}},
            {1115, {544, 1003}}, {1114, {544, 1003}}, {1113, {545, 1002}}, {1112, {545, 1002}}, {1111, {545, 1001}},
            {1110, {545, 1000}}, {1109, {545, 999}}, {1108, {545, 999}}, {1107, {545, 999}}, {1106, {545, 998}},
            {1105, {545, 996}}}};
        const int unresolvedWorm3c = runSequence(worm3Spec, 1100, 1135, {orientationFlip}, true);
        require(unresolvedSet1 + unresolvedSet2 + unresolvedWorm3a + unresolvedWorm3b + unresolvedWorm3c <= 6,
                "too many unresolved self-crossed frames");
        std::cout << "Endpoint regression checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
