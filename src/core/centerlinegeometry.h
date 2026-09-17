#ifndef CENTERLINEGEOMETRY_H
#define CENTERLINEGEOMETRY_H

#include "centerlinetypes.h"

#include <QList>
#include <QPointF>
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

// Reconstruct a video-coordinate centerline from Dijkstra parent links.
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

// ── Blob-level centerline helpers ──────────────────────────────────────────
// Operate on Tracking::DetectedBlob; formerly declared in namespace Tracking.

/**
 * @brief Compute and store an ordered centerline for a detected blob from its contour.
 * The resulting polyline runs from one skeleton endpoint to the other and is stored in
 * DetectedBlob::centerlinePoints. Returns true when a usable centerline was found.
 */
bool populateCenterlineFromContour(Tracking::DetectedBlob& blob);

/**
 * @brief Compute a centerline after cutting a ring blob mask open.
 * @param blob Detected blob with ring topology. Updated with the best ordered centerline.
 * @param cutStart First endpoint of the cut line in video coordinates.
 * @param cutEnd Second endpoint of the cut line in video coordinates.
 * @param cutThickness Thickness of the erased cut line in pixels.
 * @return True when a usable centerline was found after applying the cut.
 */
bool populateCenterlineFromContourWithCut(Tracking::DetectedBlob& blob,
                                          const cv::Point2f& cutStart,
                                          const cv::Point2f& cutEnd,
                                          int cutThickness = 3);

/**
 * @brief Extract an ordered skeleton centerline from a detected worm blob.
 * @param blob Detected blob with contour points in video coordinates.
 * @return Ordered centerline points in video coordinates. Empty if no valid skeleton can be extracted.
 */
QList<QPointF> extractOrderedCenterlinePoints(const Tracking::DetectedBlob& blob);

/**
 * @brief Resample an ordered centerline to a fixed number of evenly spaced points.
 * @param points Ordered source centerline points.
 * @param pointCount Number of points to return.
 * @return Exactly pointCount points when input is non-empty; empty if input is empty or pointCount <= 0.
 */
QList<QPointF> resampleCenterlinePoints(const QList<QPointF>& points, int pointCount);

/**
 * @brief Resample an ordered centerline to a fixed number of evenly spaced points.
 * @param points Ordered source centerline points.
 * @param pointCount Number of points to return.
 * @return Exactly pointCount points when input is non-empty; empty if input is empty or pointCount <= 0.
 */
QList<QPointF> resampleCenterlinePoints(const std::vector<cv::Point2f>& points, int pointCount);

/**
 * @brief Extract and resample a detected blob centerline.
 * @param blob Detected blob with contour points in video coordinates.
 * @param pointCount Number of centerline points to return.
 * @return Fixed-count centerline points when extraction succeeds; otherwise empty.
 */
QList<QPointF> extractResampledCenterlinePoints(const Tracking::DetectedBlob& blob, int pointCount = 10);

} // namespace Centerline

#endif // CENTERLINEGEOMETRY_H
