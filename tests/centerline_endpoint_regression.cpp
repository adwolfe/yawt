#include "core/centerlinegeometry.h"
#include "core/centerlineprocessor.h"
#include "data/trackingdatastorage.h"
#include "debug/debugdatastore.h"
#include "debug/debugexporter.h"
#include <QCoreApplication>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
#include "truncated_head_contours.h"
#include "visible_cap_contours.h"
#include "oblique_tail_contours.h"

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
        std::cout << "Endpoint regression checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
