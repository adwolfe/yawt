#include "centerlinegeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <opencv2/imgproc.hpp>

namespace Centerline {

constexpr int kCenterlinePaddingPixels = 2;

// Run one Zhang-Suen thinning sub-iteration over a normalized 0/1 mask.
static void zhangSuenThinningIteration(cv::Mat& image, int iteration)
{
    cv::Mat marker = cv::Mat::zeros(image.size(), CV_8UC1);

    for (int row = 1; row < image.rows - 1; ++row) {
        const uchar* previousRow = image.ptr<uchar>(row - 1);
        const uchar* currentRow = image.ptr<uchar>(row);
        const uchar* nextRow = image.ptr<uchar>(row + 1);
        uchar* markerRow = marker.ptr<uchar>(row);

        for (int col = 1; col < image.cols - 1; ++col) {
            if (currentRow[col] == 0) {
                continue;
            }

            const int p2 = previousRow[col];
            const int p3 = previousRow[col + 1];
            const int p4 = currentRow[col + 1];
            const int p5 = nextRow[col + 1];
            const int p6 = nextRow[col];
            const int p7 = nextRow[col - 1];
            const int p8 = currentRow[col - 1];
            const int p9 = previousRow[col - 1];

            const int neighborCount = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            if (neighborCount < 2 || neighborCount > 6) {
                continue;
            }

            const int transitions =
                (p2 == 0 && p3 == 1) +
                (p3 == 0 && p4 == 1) +
                (p4 == 0 && p5 == 1) +
                (p5 == 0 && p6 == 1) +
                (p6 == 0 && p7 == 1) +
                (p7 == 0 && p8 == 1) +
                (p8 == 0 && p9 == 1) +
                (p9 == 0 && p2 == 1);
            if (transitions != 1) {
                continue;
            }

            const bool shouldRemove =
                iteration == 0
                    ? (p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
                    : (p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0);
            if (shouldRemove) {
                markerRow[col] = 1;
            }
        }
    }

    image &= ~marker;
}

cv::Mat skeletonizeBinaryMask(const cv::Mat& binaryMask)
{
    cv::Mat img;
    if (binaryMask.type() != CV_8UC1) {
        binaryMask.convertTo(img, CV_8UC1);
    } else {
        img = binaryMask.clone();
    }

    cv::threshold(img, img, 0, 255, cv::THRESH_BINARY);

    // Zhang-Suen thinning preserves connected curved bodies much better than
    // iterative erosion/dilation skeletons for thick, bent worm masks.
    img /= 255;
    cv::Mat previous = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat diff;

    do {
        zhangSuenThinningIteration(img, 0);
        zhangSuenThinningIteration(img, 1);
        cv::absdiff(img, previous, diff);
        img.copyTo(previous);
    } while (cv::countNonZero(diff) > 0);

    img *= 255;
    return img;
}

GraphSearchResult dijkstraSkeleton(const std::vector<cv::Point>& points,
                                   const std::vector<std::vector<int>>& adjacency,
                                   int startIndex)
{
    GraphSearchResult result;
    result.distances.assign(points.size(), std::numeric_limits<double>::infinity());
    result.parents.assign(points.size(), -1);

    if (startIndex < 0 || startIndex >= static_cast<int>(points.size())) {
        return result;
    }

    using QueueEntry = std::pair<double, int>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
    result.distances[startIndex] = 0.0;
    queue.push({0.0, startIndex});

    while (!queue.empty()) {
        const auto [dist, current] = queue.top();
        queue.pop();
        if (dist > result.distances[current]) {
            continue;
        }

        const cv::Point& currentPoint = points[current];
        for (int neighbor : adjacency[current]) {
            const cv::Point& neighborPoint = points[neighbor];
            const int dx = neighborPoint.x - currentPoint.x;
            const int dy = neighborPoint.y - currentPoint.y;
            const double weight = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
            const double candidate = dist + weight;
            if (candidate + 1e-9 < result.distances[neighbor]) {
                result.distances[neighbor] = candidate;
                result.parents[neighbor] = current;
                queue.push({candidate, neighbor});
            }
        }
    }

    return result;
}

std::vector<cv::Point2f> reconstructCenterlinePath(const std::vector<cv::Point>& points,
                                                   const std::vector<int>& parents,
                                                   int startIndex,
                                                   int endIndex,
                                                   const cv::Point2f& offset)
{
    std::vector<cv::Point2f> path;
    if (startIndex < 0 || endIndex < 0 || startIndex >= static_cast<int>(points.size())
        || endIndex >= static_cast<int>(points.size())) {
        return path;
    }

    int current = endIndex;
    while (current != -1) {
        const cv::Point& point = points[current];
        path.emplace_back(offset.x + static_cast<float>(point.x),
                          offset.y + static_cast<float>(point.y));
        if (current == startIndex) {
            break;
        }
        current = parents[current];
    }

    if (path.empty() || current == -1) {
        path.clear();
        return path;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

Centerline::SkeletonGraph buildSkeletonGraph(const cv::Mat& mask)
{
    Centerline::SkeletonGraph graph;
    if (mask.empty()) {
        return graph;
    }

    cv::Mat skeleton = skeletonizeBinaryMask(mask);
    if (cv::countNonZero(skeleton) < 2) {
        return graph;
    }

    graph.skeleton = skeleton;
    graph.indexImage = cv::Mat(skeleton.size(), CV_32SC1, cv::Scalar(-1));
    graph.points.reserve(static_cast<size_t>(cv::countNonZero(skeleton)));

    for (int row = 0; row < skeleton.rows; ++row) {
        const uchar* rowPtr = skeleton.ptr<uchar>(row);
        int* indexRow = graph.indexImage.ptr<int>(row);
        for (int col = 0; col < skeleton.cols; ++col) {
            if (rowPtr[col] == 0) {
                continue;
            }
            indexRow[col] = static_cast<int>(graph.points.size());
            graph.points.emplace_back(col, row);
        }
    }

    if (graph.points.size() < 2) {
        graph = Centerline::SkeletonGraph{};
        return graph;
    }

    graph.adjacency.assign(graph.points.size(), {});
    graph.endpointIndices.reserve(graph.points.size());
    for (int idx = 0; idx < static_cast<int>(graph.points.size()); ++idx) {
        const cv::Point& point = graph.points[idx];
        int degree = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const int nx = point.x + dx;
                const int ny = point.y + dy;
                if (nx < 0 || ny < 0 || nx >= graph.indexImage.cols || ny >= graph.indexImage.rows) {
                    continue;
                }
                const int neighborIndex = graph.indexImage.at<int>(ny, nx);
                if (neighborIndex < 0) {
                    continue;
                }
                graph.adjacency[idx].push_back(neighborIndex);
                ++degree;
            }
        }
        if (degree == 1) {
            graph.endpointIndices.push_back(idx);
        }
    }

    return graph;
}

std::vector<cv::Point2f> extractCenterlineFromMask(const cv::Mat& mask,
                                                   const cv::Point2f& offset)
{
    Centerline::SkeletonGraph graph = buildSkeletonGraph(mask);
    if (graph.points.size() < 2) {
        return {};
    }

    int bestStart = -1;
    int bestEnd = -1;
    double bestDistance = -1.0;
    std::vector<int> bestParents;

    if (graph.endpointIndices.size() >= 2) {
        for (int endpoint : graph.endpointIndices) {
            GraphSearchResult search = dijkstraSkeleton(graph.points, graph.adjacency, endpoint);
            for (int candidate : graph.endpointIndices) {
                if (candidate == endpoint) {
                    continue;
                }
                const double distance = search.distances[candidate];
                if (!std::isfinite(distance) || distance <= bestDistance) {
                    continue;
                }
                bestDistance = distance;
                bestStart = endpoint;
                bestEnd = candidate;
                bestParents = search.parents;
            }
        }
    }

    if (bestStart == -1 || bestEnd == -1) {
        GraphSearchResult firstSweep = dijkstraSkeleton(graph.points, graph.adjacency, 0);
        int farthestA = 0;
        for (int idx = 1; idx < static_cast<int>(graph.points.size()); ++idx) {
            if (std::isfinite(firstSweep.distances[idx]) &&
                firstSweep.distances[idx] > firstSweep.distances[farthestA]) {
                farthestA = idx;
            }
        }

        GraphSearchResult secondSweep = dijkstraSkeleton(graph.points, graph.adjacency, farthestA);
        int farthestB = farthestA;
        for (int idx = 0; idx < static_cast<int>(graph.points.size()); ++idx) {
            if (std::isfinite(secondSweep.distances[idx]) &&
                secondSweep.distances[idx] > secondSweep.distances[farthestB]) {
                farthestB = idx;
            }
        }

        bestStart = farthestA;
        bestEnd = farthestB;
        bestParents = std::move(secondSweep.parents);
    }

    return reconstructCenterlinePath(graph.points, bestParents, bestStart, bestEnd, offset);
}


cv::Rect buildCenterlineMask(const Tracking::DetectedBlob& blob, cv::Mat& mask)
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

} // namespace Centerline
