#ifndef MINILOADER_H
#define MINILOADER_H

#include <QWidget>
#include <QImage>
#include <QPointF>
#include <QSizeF>
#include <QRectF>
#include <QList>
#include <QMap>
#include <QSet>
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

    // Visible worm IDs in the currently displayed crop (updated each draw)
    QList<int> getVisibleWormIds() const;
    void setVisibleWormIds(const QList<int>& ids);

    // Per-frame visibility map computed by the MiniLoader when multiple cropped frames are provided.
    // Key = absolute frame number, Value = set of visible worm IDs for that frame.
    QMap<int, QSet<int>> getVisibleWormsByFrame() const;
    void setVisibleWormsByFrame(const QMap<int, QSet<int>>& map);

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

    /**
     * @brief Update with multiple pre-cropped frames (centered around centerFrameNumber).
     *
     * The lists of frames/offsets/sizes must be the same length and correspond to consecutive frames
     * (e.g. center-radius .. center+radius). The MiniLoader will display the central frame but will
     * compute visibility across all supplied frames and emit per-frame visibility via the signal below.
     *
     * @param centerFrameNumber Absolute frame index corresponding to the central image in the lists.
     * @param croppedFrames List of cropped QImage frames ordered left-to-right (center-radius .. center+radius).
     * @param cropOffsets Top-left offsets in video coordinates for each cropped frame (same order).
     * @param cropSizes Sizes (video coordinates) for each cropped frame (same order).
     * @param centerPoint The center point used for the central crop (video coords).
     */
    void updateWithCroppedFrames(int centerFrameNumber,
                                 const QList<QImage>& croppedFrames,
                                 const QList<QPointF>& cropOffsets,
                                 const QList<QSizeF>& cropSizes,
                                 QPointF centerPoint);

signals:
    /**
     * @brief Emitted when the selected worm changes.
     * @param wormId The new selected worm ID (-1 if none)
     */
    void selectedWormChanged(int wormId);

    /**
     * @brief Emitted whenever the set of visible worm IDs in the miniLoader's current crop/frame is updated.
     * @param visibleIds List of worm IDs that are present (whole or partially) in the last drawn crop.
     */
    void visibleWormsUpdated(const QList<int>& visibleIds);

    /**
     * @brief Emitted when the MiniLoader computes per-frame visibility for a set of supplied cropped frames.
     * The map key is the absolute frame number and the value is the set of visible worm IDs in that frame.
     */
    void visibleWormsUpdatedPerFrame(const QMap<int, QSet<int>>& visibleByFrame);

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
    int m_selectedWormId;                        // Currently selected worm ID (-1 for none)

    // Simple storage of currently visible worm IDs (updated each paint/draw)
    QList<int> m_visibleWormIds;

    // Optional per-frame visibility map computed when multiple frames are supplied via updateWithCroppedFrames.
    // Key = absolute frame number; value = set of visible worm IDs for that frame.
    QMap<int, QSet<int>> m_visibleWormsByFrame;
};

#endif // MINILOADER_H