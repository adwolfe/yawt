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
    std::function<QList<QList<int>>(int)> getMergeGroupsForFrame;
    std::function<TipFeatureBaseline(int)> getTipBaseline;
    std::function<void(int, int, const Tracking::DetectedBlob&)> setDetectedBlobForFrame;
    std::function<void(int, float, float)> recordTipFeatureSample;
    std::function<void(int, float)> recordBodyLengthSample;
    std::function<void(const Debug::CenterlineFrameDebug&)> setCenterlineDebugFrame;
};

// Measure the arc length of an ordered centerline polyline.
float arcLength(const std::vector<cv::Point2f>& points);

// Run the live centerline-analysis pipeline for one worm/frame and update sweep state.
CenterlineFrameResult processFrame(const CenterlineFrameContext& ctx,
                                   const CenterlineFrameRequest& req,
                                   CenterlineSweepState& state,
                                   CenterlineFrameIo& io);

} // namespace Centerline

#endif // CENTERLINEPROCESSOR_H
