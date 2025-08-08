#ifndef MINIVIDEOLOADER_H
#define MINIVIDEOLOADER_H

#include <QWidget>
#include <QImage>
#include <QRectF>
#include <QPointF>
#include <QSize>
#include <QPaintEvent>
#include <QTimer>
#include "trackingcommon.h"

// Forward declarations
class TrackingDataStorage;

/**
 * @brief MiniVideoLoader provides a cropped, zoomed-in view around a selected worm.
 * 
 * This is a simple mirror/crop widget that displays a cropped portion of the main
 * VideoLoader's current frame, focused on a selected worm. It doesn't handle any
 * interactions - just displays a zoomed crop view for detailed inspection.
 */
class MiniVideoLoader : public QWidget
{
    Q_OBJECT

public:
    explicit MiniVideoLoader(QWidget *parent = nullptr);
    ~MiniVideoLoader();

    /**
     * @brief Sets the tracking data storage reference for polling worm positions.
     * @param storage Pointer to the TrackingDataStorage instance
     */
    void setTrackingDataStorage(TrackingDataStorage* storage);

    /**
     * @brief Sets the crop multiplier (how much area around the worm to show).
     * @param multiplier Factor to multiply the worm's ROI size (default 1.5)
     */
    void setCropMultiplier(double multiplier);

    /**
     * @brief Gets the current crop multiplier.
     */
    double getCropMultiplier() const;

    /**
     * @brief Gets the currently selected worm ID.
     * @return The worm ID, or -1 if no worm is selected
     */
    int getSelectedWorm() const;

    /**
     * @brief Checks if a valid crop is currently being displayed.
     */
    bool hasValidCrop() const;

public slots:
    /**
     * @brief Updates the display with a new frame from the main VideoLoader.
     * @param frameNumber The current frame number
     * @param frame The current frame as QImage
     */
    void updateFrame(int frameNumber, const QImage& frame);

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
     * @brief Clears the selected worm and shows a "no selection" message.
     */
    void clearSelection();

signals:
    /**
     * @brief Emitted when the selected worm changes.
     * @param wormId The new selected worm ID (-1 if none)
     */
    void selectedWormChanged(int wormId);

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    /**
     * @brief Calculates the crop rectangle around the selected worm.
     * @return The crop rectangle in image coordinates
     */
    QRectF calculateCropRect() const;

    /**
     * @brief Updates the cropped image based on current settings.
     */
    void updateCroppedImage();

    /**
     * @brief Polls TrackingDataStorage for current worm position.
     * @return True if worm position was found and updated, false otherwise
     */
    bool pollWormPosition();

    /**
     * @brief Draws the "no selection" message.
     * @param painter The painter to draw with
     */
    void drawNoSelectionMessage(QPainter& painter);

    /**
     * @brief Draws the cropped worm view.
     * @param painter The painter to draw with
     */
    void drawCroppedView(QPainter& painter);

private:
    // Current frame data
    QImage m_currentFrame;          // Full frame from main VideoLoader
    QImage m_croppedFrame;          // Cropped portion around selected worm
    int m_currentFrameNumber;       // Current frame number

    // Worm selection and position
    int m_selectedWormId;           // Currently selected worm ID (-1 if none)
    QPointF m_wormPosition;         // Current worm position in image coordinates
    QRectF m_wormRoi;              // Current worm ROI in image coordinates
    double m_cropMultiplier;        // Multiplier for crop size around worm

    // Data source
    TrackingDataStorage* m_trackingDataStorage;  // Reference to tracking data storage

    // Display settings
    QRectF m_cropRect;             // Current crop rectangle in image coordinates
    bool m_hasValidData;           // Whether we have valid worm data to display

    // Constants
    static constexpr double DEFAULT_CROP_MULTIPLIER = 1.5;
    static constexpr double MIN_CROP_SIZE = 50.0;  // Minimum crop size in pixels
};

#endif // MINIVIDEOLOADER_H