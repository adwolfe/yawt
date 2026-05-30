#ifndef LIVEVIEW_H
#define LIVEVIEW_H

#include <QWidget>
#include <QImage>
#include <QPointF>
#include <QRectF>
#include <opencv2/core.hpp>

/**
 * LiveView — live-camera display widget with pan/zoom and ROI drawing.
 *
 * Pan/zoom:
 *   - Scroll wheel / trackpad: zoom centred on cursor
 *   - Left-button drag (pan mode): pan
 *   - Double-click: reset zoom & pan
 *
 * ROI mode (setRoiMode(true)):
 *   - Left-button drag draws a rectangle
 *   - Release emits roiDefined(QRectF) in video coordinates
 *   - Active ROI is shown as a dashed overlay at all times
 */
class LiveView : public QWidget
{
    Q_OBJECT
public:
    explicit LiveView(QWidget* parent = nullptr);

    double  zoomFactor() const { return m_zoomFactor; }
    QRectF  roi()        const { return m_roi; }
    bool    hasRoi()     const { return m_roi.isValid() && !m_roi.isEmpty(); }

public slots:
    void updateFrame(const cv::Mat& frame);
    void clear();
    void resetView();

    /** Enter/leave ROI-draw mode. */
    void setRoiMode(bool enabled);

    /** Programmatically set the displayed ROI (video coordinates). */
    void setRoi(const QRectF& videoRect);
    void clearRoi();

signals:
    /** Emitted when the user finishes drawing an ROI (video coordinates). */
    void roiDefined(QRectF videoRect);
    /** Emitted when the ROI is cleared by the user or programmatically. */
    void roiCleared();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void    setZoomFactorAtPoint(double factor, const QPointF& widgetPoint);
    QRectF  calculateTargetRect() const;
    void    clampPanOffset();
    QPointF widgetToVideo(const QPointF& wp) const;
    QPointF videoToWidget(const QPointF& vp) const;
    void    updateCursor();

    // --- Frame ---
    QImage  m_frame;
    QSize   m_frameSize;

    // --- Zoom / pan ---
    double  m_zoomFactor = 1.0;
    QPointF m_panOffset  = {0.0, 0.0};
    bool    m_isPanning  = false;
    QPointF m_lastMousePos;

    // --- ROI ---
    bool    m_roiMode     = false;
    bool    m_drawingRoi  = false;
    QPointF m_roiDragStart;   // widget coords during drag
    QPointF m_roiDragEnd;     // widget coords during drag
    QRectF  m_roi;            // committed ROI in video coords (empty = none)
};

#endif // LIVEVIEW_H
