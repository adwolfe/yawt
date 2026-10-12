#pragma once

#include <QRectF>
#include <QSet>
#include <opencv2/core.hpp>
#include <vector>

// One physical blob shared by multiple conceptual worms on a frame.
struct SharedBlob {
    int uniqueId = -1;
    QRectF currentBoundingBox;
    cv::Point2f currentCentroid;
    double currentArea = 0.0;
    std::vector<cv::Point> contourPoints;
    std::vector<std::vector<cv::Point>> holeContourPoints;
    QSet<int> participatingWormTrackerIDs;
    int frameNumber = -1;
    int selectedByWormTrackerId = -1;
};
