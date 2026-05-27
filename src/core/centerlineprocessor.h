#ifndef CENTERLINEPROCESSOR_H
#define CENTERLINEPROCESSOR_H

#include "centerlinetypes.h"
#include "../debug/debugrecords.h"

#include <QList>
#include <QMap>
#include <functional>
#include <vector>

namespace Centerline {

struct CenterlineState {
    std::vector<cv::Point2f> points;
    cv::Point2f blobCentroid;
    Tracking::DetectedBlob blob;
    bool valid = false;
    float turningAngle = 0.f;

    // Return the current head-side endpoint of the carried centerline.
    cv::Point2f nose() const { return points.front(); }
    // Return the current tail-side endpoint of the carried centerline.
    cv::Point2f tail() const { return points.back(); }
    // Return the midpoint sample of the carried centerline.
    cv::Point2f midpoint() const { return points[points.size() / 2]; }
};

struct CenterlineFrameContext {
    int wormId = -1;
    const std::vector<Tracking::WormTrackPoint>* sortedPoints = nullptr;
    int nPts = 4;
    float refLength = 0.f;
    CenterlineSnakeParams snakeParams;
    bool captureDebug = false;
};

struct CenterlineSweepState {
    HeadTailPredictor predictor;
    CenterlineState prevState;
};

struct CenterlineFrameRequest {
    int pointIndex = -1;
    int step = 1;
    bool isKeyframeBootstrap = false;
};

struct CenterlineFrameResult {
    bool processed = false;
    bool wroteBlob = false;
    Tracking::DetectedBlob blob;
    Debug::CenterlineFrameDebug debugRecord;
};

struct CenterlineFrameIo {
    std::function<QMap<int, Tracking::DetectedBlob>(int)> getDetectedBlobsForFrame;
    std::function<QList<QList<int>>(int)> getMergeGroupsForFrame;
    std::function<TipFeatureBaseline(int)> getTipBaseline;
    std::function<void(int, int, const Tracking::DetectedBlob&)> setDetectedBlobForFrame;
    std::function<void(int, float, float)> recordTipFeatureSample;
    std::function<void(int, float)> recordBodyLengthSample;
    std::function<void(const Debug::CenterlineFrameDebug&)> setCenterlineDebugFrame;
};

struct GraphSearchResult {
    std::vector<double> distances;
    std::vector<int> parents;
};

// Thin a binary worm mask into a Zhang-Suen one-pixel skeleton.
cv::Mat skeletonizeBinaryMask(const cv::Mat& binaryMask);

// Run weighted Dijkstra over an 8-connected skeleton pixel graph.
GraphSearchResult dijkstraSkeleton(const std::vector<cv::Point>& points,
                                   const std::vector<std::vector<int>>& adjacency,
                                   int startIndex);

// Reconstruct a world-coordinate centerline from Dijkstra parent links.
std::vector<cv::Point2f> reconstructCenterlinePath(const std::vector<cv::Point>& points,
                                                   const std::vector<int>& parents,
                                                   int startIndex,
                                                   int endIndex,
                                                   const cv::Point2f& offset);

// Build an 8-connected skeleton graph from a binary worm mask.
SkeletonGraph buildSkeletonGraph(const cv::Mat& mask);

// Build the padded binary blob mask used by centerline and endpoint analysis.
cv::Rect buildCenterlineMask(const Tracking::DetectedBlob& blob, cv::Mat& mask);

// Extract the longest usable ordered centerline from a binary worm mask.
std::vector<cv::Point2f> extractCenterlineFromMask(const cv::Mat& mask,
                                                   const cv::Point2f& offset);

// Measure the arc length of an ordered centerline polyline.
float arcLength(const std::vector<cv::Point2f>& points);

// Run the live centerline-analysis pipeline for one worm/frame and update sweep state.
CenterlineFrameResult processFrame(const CenterlineFrameContext& ctx,
                                   const CenterlineFrameRequest& req,
                                   CenterlineSweepState& state,
                                   CenterlineFrameIo& io);

} // namespace Centerline

#endif // CENTERLINEPROCESSOR_H
