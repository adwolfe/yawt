#ifndef JSONGEOMETRY_H
#define JSONGEOMETRY_H

#include <QJsonArray>
#include <QJsonObject>
#include <QPointF>
#include <QRectF>
#include <opencv2/core.hpp>
#include <vector>

// Shared geometry encodings used by tracking-state JSON:
// points {x,y}, rectangles {x,y,width,height}, and polylines [[x,y], ...].
// Readers retain the file format's zero defaults and skip coordinate arrays
// with fewer than two entries.
namespace JsonGeometry {

// ── Write ────────────────────────────────────────────────────────────────────

inline QJsonObject toJson(const QPointF& p)
{
    QJsonObject o;
    o["x"] = p.x();
    o["y"] = p.y();
    return o;
}

inline QJsonObject toJson(const cv::Point2f& p)
{
    QJsonObject o;
    o["x"] = static_cast<double>(p.x);
    o["y"] = static_cast<double>(p.y);
    return o;
}

inline QJsonObject toJson(const QRectF& r)
{
    QJsonObject o;
    o["x"]      = r.x();
    o["y"]      = r.y();
    o["width"]  = r.width();
    o["height"] = r.height();
    return o;
}

// Integer contour: [[x,y], ...]
inline QJsonArray contourToJson(const std::vector<cv::Point>& contour)
{
    QJsonArray arr;
    for (const cv::Point& pt : contour) {
        QJsonArray a;
        a.append(pt.x);
        a.append(pt.y);
        arr.append(a);
    }
    return arr;
}

// Nested integer contours (e.g. ring holes): [[[x,y], ...], ...]
inline QJsonArray holeContoursToJson(const std::vector<std::vector<cv::Point>>& holes)
{
    QJsonArray arr;
    for (const std::vector<cv::Point>& hole : holes) {
        arr.append(contourToJson(hole));
    }
    return arr;
}

// Float polyline (e.g. centerline points): [[x,y], ...]
inline QJsonArray pointsToJson(const std::vector<cv::Point2f>& points)
{
    QJsonArray arr;
    for (const cv::Point2f& pt : points) {
        QJsonArray a;
        a.append(static_cast<double>(pt.x));
        a.append(static_cast<double>(pt.y));
        arr.append(a);
    }
    return arr;
}

// ── Read ─────────────────────────────────────────────────────────────────────

inline QPointF pointFromJson(const QJsonObject& o)
{
    return QPointF(o.value("x").toDouble(), o.value("y").toDouble());
}

inline cv::Point2f cvPointFromJson(const QJsonObject& o)
{
    return cv::Point2f(static_cast<float>(o.value("x").toDouble()),
                       static_cast<float>(o.value("y").toDouble()));
}

inline QRectF rectFromJson(const QJsonObject& o)
{
    return QRectF(o.value("x").toDouble(), o.value("y").toDouble(),
                  o.value("width").toDouble(), o.value("height").toDouble());
}

inline std::vector<cv::Point> contourFromJson(const QJsonArray& arr)
{
    std::vector<cv::Point> contour;
    contour.reserve(static_cast<size_t>(arr.size()));
    for (const QJsonValue& v : arr) {
        const QJsonArray a = v.toArray();
        if (a.size() >= 2) {
            contour.push_back(cv::Point(a[0].toInt(), a[1].toInt()));
        }
    }
    return contour;
}

inline std::vector<std::vector<cv::Point>> holeContoursFromJson(const QJsonArray& arr)
{
    std::vector<std::vector<cv::Point>> holes;
    holes.reserve(static_cast<size_t>(arr.size()));
    for (const QJsonValue& hv : arr) {
        holes.push_back(contourFromJson(hv.toArray()));
    }
    return holes;
}

inline std::vector<cv::Point2f> pointsFromJson(const QJsonArray& arr)
{
    std::vector<cv::Point2f> points;
    points.reserve(static_cast<size_t>(arr.size()));
    for (const QJsonValue& v : arr) {
        const QJsonArray a = v.toArray();
        if (a.size() >= 2) {
            points.push_back(cv::Point2f(static_cast<float>(a[0].toDouble()),
                                         static_cast<float>(a[1].toDouble())));
        }
    }
    return points;
}

} // namespace JsonGeometry

#endif // JSONGEOMETRY_H
