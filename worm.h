#ifndef WORM_H
#define WORM_H

#include <QObject> // QObject for potential signals/slots if needed later
#include <QPointF>
#include <QRectF>
#include <QColor>
#include <QUuid> // For a unique ID

/**
 * @brief Holds data for a single worm.
 */
class Worm : public QObject {
    Q_OBJECT
public:
    explicit Worm(int id, const QPointF& initialCentroid, const QRectF& initialBoundingBox, QObject *parent = nullptr)
        : QObject(parent), m_id(id), m_uniqueId(QUuid::createUuid()),
        m_initialCentroid(initialCentroid), m_initialBoundingBox(initialBoundingBox) {
        // Assign a default color based on ID or use a predefined palette
        int colorIndex = id % 10; // Simple way to get varied colors
        const QColor colors[] = {Qt::red, Qt::green, Qt::blue, Qt::cyan, Qt::magenta, Qt::yellow, Qt::gray, Qt::darkRed, Qt::darkGreen, Qt::darkBlue};
        m_trackColor = colors[colorIndex];
    }

    int id() const { return m_id; }
    QUuid uniqueId() const { return m_uniqueId; }
    QPointF initialCentroid() const { return m_initialCentroid; }
    QRectF initialBoundingBox() const { return m_initialBoundingBox; }
    QColor trackColor() const { return m_trackColor; }
    void setTrackColor(const QColor& color) { m_trackColor = color; }

private:
    int m_id;                 // User-friendly ID (e.g., 1, 2, 3...)
    QUuid m_uniqueId;         // Globally unique identifier
    QPointF m_initialCentroid; // Centroid in original video coordinates at the keyframe
    QRectF m_initialBoundingBox; // Bounding box in original video coordinates at the keyframe
    QColor m_trackColor;      // Color for displaying this worm's track
};

#endif // WORM_H
