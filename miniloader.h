#ifndef MINILOADER_H
#define MINILOADER_H

#include <QWidget>
#include <QImage>
#include <QPointF>
#include <QSizeF>
#include <QRectF>
#include <QList>
#include "trackingcommon.h"

// Forward declarations
class TrackingDataStorage;

/**
 * Simple MiniLoader widget that displays a pre-cropped frame.
 * MainWindow handles all cropping logic and sends the result here for display.
 * Supports optional overlay drawing for selected worm blob outlines.
 */
class MiniLoader : public QWidget
{
    Q_OBJECT

public:
    explicit MiniLoader(QWidget* parent = nullptr);
    ~MiniLoader() override;

    /**
     * @brief Get the last known center point (in video coordinates)
     * Used by MainWindow as a fallback when no worm is selected
     * @return Last center point, or invalid QPointF() if none set
     */
    QPointF getLastCenterPoint() const;

    /**
     * @brief Sets the tracking data storage reference for getting worm blob data.
     * @param storage Pointer to the TrackingDataStorage instance
     */
    void setTrackingDataStorage(TrackingDataStorage* storage);

    /**
     * @brief Enable/disable drawing overlay outlines for selected worms.
     */
    void setShowOverlays(bool show);
    bool showOverlays() const;

    /**
     * @brief Gets the currently selected worm ID.
     * @return The worm ID, or -1 if no worm is selected
     */
    int getSelectedWorm() const;

    /**
     * @brief Update with a pre-cropped frame from MainWindow
     * @param frameNumber Current frame number
     * @param croppedFrame The cropped image to display
     * @param cropOffset Top-left corner of crop in video coordinates
     * @param cropSize Size of crop in video coordinates
     * @param centerPoint Center point used for this crop in video coordinates
     */
    void updateWithCroppedFrame(int frameNumber, const QImage& croppedFrame,
                                QPointF cropOffset, QSizeF cropSize,
                                QPointF centerPoint);

    /**
     * @brief Convert coordinates between mini widget and video coordinates
     */
    QPointF miniToVideo(const QPointF& miniPoint) const;
    QPointF videoToMini(const QPointF& videoPoint) const;

    /**
     * @brief Get crop metadata
     */
    QRectF getCurrentCropRectVideo() const;
    QPointF getCurrentCropOffset() const;
    QSizeF getCurrentCropSize() const;

public slots:
    /**
     * @brief Handles worm selection changes from the BlobTableModel.
     * @param selectedItems List of currently selected items
     */
    void onWormSelectionChanged(const QList<TableItems::ClickedItem>& selectedItems);

    /**
     * @brief Sets the selected worm directly by ID.
     * @param wormId The ID of the worm to focus on (-1 for none)
     */
    void setSelectedWorm(int wormId);

    /**
     * @brief Clears the selected worm.
     */
    void clearSelection();

signals:
    /**
     * @brief Emitted when the selected worm changes.
     * @param wormId The new selected worm ID (-1 if none)
     */
    void selectedWormChanged(int wormId);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    /**
     * @brief Draws overlay blob outlines for the selected worm.
     * @param painter The painter to draw with
     * @param targetRect The rectangle where the cropped image is displayed
     */
    void drawOverlays(QPainter& painter, const QRect& targetRect);

    /**
     * @brief Converts video coordinates to cropped image coordinates.
     * @param videoCoords Position in full video coordinates  
     * @return Position in cropped image coordinates
     */
    QPointF mapVideoToCropCoords(const QPointF& videoCoords) const;

private:
    // Display data
    QImage m_croppedFrame;          // The cropped image to display
    int m_currentFrameNumber;       // Current frame number
    
    // Crop metadata (from MainWindow)
    QPointF m_centerPoint;          // Center point in video coordinates
    QPointF m_cropOffset;           // Top-left of crop in video coordinates
    QSizeF m_cropSize;              // Size of crop in video coordinates

    // Overlay functionality
    TrackingDataStorage* m_trackingDataStorage;  // Reference to tracking data storage
    bool m_showOverlays;                         // Whether to draw overlays
    int m_selectedWormId;                        // Currently selected worm ID (-1 if none)
};

#endif // MINILOADER_H