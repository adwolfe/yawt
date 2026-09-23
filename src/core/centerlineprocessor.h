#ifndef CENTERLINEPROCESSOR_H
#define CENTERLINEPROCESSOR_H

#include "centerlinetypes.h"
#include "centerlinegeometry.h"
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
    cv::Point2f head() const { return points.front(); }
    // Return the current tail-side endpoint of the carried centerline.
    cv::Point2f tail() const { return points.back(); }
    // Return the midpoint sample of the carried centerline.
    cv::Point2f midpoint() const { return points[points.size() / 2]; }
};

struct CenterlineFrameContext {
    int wormId = -1;
    const Tracking::Track* sortedPoints = nullptr;
    int nPts = 4;
    float refLength = 0.f;
    CenterlineSnakeParams snakeParams;
    bool captureDebug = false;
};

struct CenterlineSweepState {
    HeadTailPredictor predictor;
    CenterlineState prevState;
    // Signed turning (head→tail) of the last trusted centerline. Held through a
    // self-contact so route selection can keep the loop's sense of rotation.
    bool hasOrientationReference = false;
    float orientationReference = 0.f;
};

struct CenterlineFrameRequest {
    int pointIndex = -1;
    int step = 1;
    bool isKeyframeBootstrap = false;
    bool skipIfMerged = false;  // When true, return early without computing centerline for merged frames.
};

struct CenterlineFrameResult {
    bool processed = false;
    bool wroteBlob = false;
    Tracking::DetectedBlob blob;
    Debug::CenterlineFrameDebug debugRecord;
};

struct CenterlineFrameIo {
    std::function<QMap<int, Tracking::DetectedBlob>(int)> getDetectedBlobsForFrame;
    std::function<QList<Tracking::MergeGroup>(int)> getMergeGroupsForFrame;
    std::function<TipFeatureBaseline(int)> getTipBaseline;
    std::function<void(int, int, const Tracking::DetectedBlob&)> setDetectedBlobForFrame;
    std::function<void(int, float, float)> recordTipFeatureSample;
    std::function<void(int, float)> recordBodyLengthSample;
    std::function<void(const Debug::CenterlineFrameDebug&)> setCenterlineDebugFrame;
};

// Measure the arc length of an ordered centerline polyline.
float arcLength(const std::vector<cv::Point2f>& points);

// Arc length after resampling to nPoints, the measure the body-length baseline uses.
float resampledArcLength(const std::vector<cv::Point2f>& points, int nPoints);

// Run the live centerline-analysis pipeline for one worm/frame and update sweep state.
CenterlineFrameResult processFrame(const CenterlineFrameContext& ctx,
                                   const CenterlineFrameRequest& req,
                                   CenterlineSweepState& state,
                                   CenterlineFrameIo& io);

// Re-run the snake with the assigned tips pinned and a soft target for the
// centerline midpoint. Updates blob.centerline.points in place. Returns false
// if the blob is not Clean, prerequisites are missing, or the target is outside
// the blob mask.
bool relaxCenterlineToSmoothedMidpoint(Tracking::DetectedBlob& blob,
                                      const cv::Point2f& midpointTarget,
                                      int nPoints,
                                      const CenterlineSnakeParams& params);

} // namespace Centerline

#endif // CENTERLINEPROCESSOR_H
