#ifndef CENTERLINEGEOMETRY_H
#define CENTERLINEGEOMETRY_H

#include "centerlinetypes.h"

#include <vector>

namespace Centerline {

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

} // namespace Centerline

#endif // CENTERLINEGEOMETRY_H
