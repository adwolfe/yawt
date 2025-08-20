#ifndef MINILOADER_H
#define MINILOADER_H

#include <QWidget>
#include <QImage>
#include <QPointF>
#include <QSizeF>
#include <QRectF>

/**
 * Simple MiniLoader widget that displays a pre-cropped frame.
 * MainWindow handles all cropping logic and sends the result here for display.
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

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    // Display data
    QImage m_croppedFrame;          // The cropped image to display
    int m_currentFrameNumber;       // Current frame number
    
    // Crop metadata (from MainWindow)
    QPointF m_centerPoint;          // Center point in video coordinates
    QPointF m_cropOffset;           // Top-left of crop in video coordinates
    QSizeF m_cropSize;              // Size of crop in video coordinates
};

#endif // MINILOADER_H