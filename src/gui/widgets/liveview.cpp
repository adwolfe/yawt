#include "liveview.h"

#include <QPainter>
#include <QPainterPath>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QInputDevice>
#include <QtMath>
#include <opencv2/imgproc.hpp>
#include "../../utils/cvimageutils.h"

LiveView::LiveView(QWidget* parent)
    : QWidget(parent)
{
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::black);
    setPalette(pal);
    setMinimumSize(320, 240);
    setMouseTracking(true);
    updateCursor();
}

// ---------------------------------------------------------------------------
// Public slots
// ---------------------------------------------------------------------------

void LiveView::updateFrame(const cv::Mat& frame)
{
    if (frame.empty()) return;

    if (QSize(frame.cols, frame.rows) != m_frameSize) {
        m_frameSize  = QSize(frame.cols, frame.rows);
        m_zoomFactor = 1.0;
        m_panOffset  = {0.0, 0.0};
        // Scale existing ROI if resolution changed (e.g., after reconnect at different res)
        // — simplest safe choice is to clear it.
        if (!m_roi.isEmpty()) {
            m_roi = QRectF();
            emit roiCleared();
        }
    }

    m_frame = CvImageUtils::matToQImage(frame);
    update();
}

void LiveView::clear()
{
    m_frame      = QImage();
    m_frameSize  = QSize();
    m_zoomFactor = 1.0;
    m_panOffset  = {0.0, 0.0};
    m_roi        = QRectF();
    m_drawingRoi = false;
    update();
}

void LiveView::resetView()
{
    m_zoomFactor = 1.0;
    m_panOffset  = {0.0, 0.0};
    update();
}

void LiveView::setRoiMode(bool enabled)
{
    m_roiMode    = enabled;
    m_drawingRoi = false;
    if (enabled) { m_scaleMeasureMode = false; m_scaleHasStart = false; }
    updateCursor();
    update();
}

void LiveView::setScaleMeasureMode(bool enabled)
{
    m_scaleMeasureMode = enabled;
    m_scaleHasStart    = false;
    if (enabled) { m_roiMode = false; m_drawingRoi = false; }
    updateCursor();
    update();
}

void LiveView::setRoi(const QRectF& videoRect)
{
    m_roi = videoRect.normalized();
    update();
}

void LiveView::clearRoi()
{
    m_roi        = QRectF();
    m_drawingRoi = false;
    update();
    emit roiCleared();
}

// ---------------------------------------------------------------------------
// Paint
// ---------------------------------------------------------------------------

void LiveView::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);
    p.fillRect(rect(), Qt::black);

    if (m_frame.isNull()) {
        p.setPen(Qt::darkGray);
        p.drawText(rect(), Qt::AlignCenter, "No camera feed");
        return;
    }

    // Draw video frame
    p.drawImage(calculateTargetRect(), m_frame);

    // --- Committed ROI overlay ---
    if (!m_roi.isEmpty() && m_roi.isValid()) {
        QPointF tl = videoToWidget(m_roi.topLeft());
        QPointF br = videoToWidget(m_roi.bottomRight());
        QRectF  roiWidget = QRectF(tl, br).normalized();

        // Dim area outside ROI
        QPainterPath outer;
        outer.addRect(QRectF(rect()));
        QPainterPath inner;
        inner.addRect(roiWidget);
        p.fillPath(outer.subtracted(inner), QColor(0, 0, 0, 100));

        // Dashed border
        QPen roiPen(QColor(255, 220, 0), 2, Qt::DashLine);
        p.setPen(roiPen);
        p.setBrush(Qt::NoBrush);
        p.drawRect(roiWidget);

        // Corner handles
        const qreal hs = 6.0;
        p.setBrush(QColor(255, 220, 0));
        p.setPen(Qt::NoPen);
        for (QPointF corner : { roiWidget.topLeft(), roiWidget.topRight(),
                                 roiWidget.bottomLeft(), roiWidget.bottomRight() }) {
            p.drawRect(QRectF(corner.x() - hs/2, corner.y() - hs/2, hs, hs));
        }
    }

    // --- In-progress ROI drag ---
    if (m_drawingRoi) {
        QRectF drag = QRectF(m_roiDragStart, m_roiDragEnd).normalized();
        QPen dragPen(QColor(100, 200, 255), 1, Qt::DashLine);
        p.setPen(dragPen);
        p.setBrush(QColor(100, 200, 255, 30));
        p.drawRect(drag);
    }

    // --- ROI mode hint ---
    if (m_roiMode && !m_drawingRoi) {
        p.setPen(QColor(100, 200, 255, 180));
        p.drawText(rect().adjusted(6, 6, -6, -6),
                   Qt::AlignBottom | Qt::AlignLeft,
                   "Click and drag to draw ROI");
    }

    // --- Scale measure overlay ---
    if (m_scaleMeasureMode) {
        const QColor scaleColor(255, 220, 60);

        if (!m_scaleHasStart) {
            // Prompt
            p.setPen(QPen(scaleColor, 1));
            p.drawText(rect().adjusted(6, 6, -6, -6),
                       Qt::AlignBottom | Qt::AlignLeft,
                       "Click to set start of measurement line");
        } else {
            // Draw crosshair at start (widget coords)
            QPointF sw = videoToWidget(m_scaleStartVideo);
            QPointF ew = videoToWidget(m_scaleCurrentVideo);

            // Line
            p.setPen(QPen(scaleColor, 2, Qt::DashLine));
            p.drawLine(sw, ew);

            // Crosshair at start
            const qreal arm = 8.0;
            p.setPen(QPen(scaleColor, 2, Qt::SolidLine));
            p.drawLine(sw + QPointF(-arm, 0), sw + QPointF(arm, 0));
            p.drawLine(sw + QPointF(0, -arm), sw + QPointF(0, arm));

            // Crosshair at current end
            p.setPen(QPen(scaleColor, 1, Qt::SolidLine));
            p.drawLine(ew + QPointF(-arm * 0.7, 0), ew + QPointF(arm * 0.7, 0));
            p.drawLine(ew + QPointF(0, -arm * 0.7), ew + QPointF(0, arm * 0.7));

            // Live pixel-length annotation near midpoint
            QPointF mid = (sw + ew) / 2.0;
            QPointF delta = m_scaleCurrentVideo - m_scaleStartVideo;
            double px = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
            QString lengthStr = QString("%1 px").arg(px, 0, 'f', 1);
            p.setPen(scaleColor);
            p.drawText(mid + QPointF(6, -6), lengthStr);

            // Prompt
            p.drawText(rect().adjusted(6, 6, -6, -6),
                       Qt::AlignBottom | Qt::AlignLeft,
                       "Click to set end of measurement line");
        }
    }
}

// ---------------------------------------------------------------------------
// Mouse events
// ---------------------------------------------------------------------------

void LiveView::mousePressEvent(QMouseEvent* event)
{
    m_lastMousePos = event->position();

    if (event->button() == Qt::LeftButton && !m_frame.isNull()) {
        if (m_scaleMeasureMode) {
            QPointF vp = widgetToVideo(event->position());
            if (!m_scaleHasStart) {
                m_scaleStartVideo   = vp;
                m_scaleCurrentVideo = vp;
                m_scaleHasStart     = true;
            } else {
                // Second click — finalise
                QPointF delta = vp - m_scaleStartVideo;
                double px = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
                m_scaleHasStart    = false;
                m_scaleMeasureMode = false;
                updateCursor();
                update();
                emit scaleMeasured(px);
            }
            event->accept();
            return;
        }
        if (m_roiMode) {
            m_drawingRoi  = true;
            m_roiDragStart = event->position();
            m_roiDragEnd   = event->position();
            event->accept();
            return;
        }
        m_isPanning = true;
        updateCursor();
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void LiveView::mouseMoveEvent(QMouseEvent* event)
{
    QPointF pos   = event->position();
    QPointF delta = pos - m_lastMousePos;
    m_lastMousePos = pos;

    if (m_scaleMeasureMode && m_scaleHasStart) {
        m_scaleCurrentVideo = widgetToVideo(pos);
        update();
        event->accept();
        return;
    }

    if (m_drawingRoi && (event->buttons() & Qt::LeftButton)) {
        m_roiDragEnd = pos;
        update();
        event->accept();
        return;
    }

    if (m_isPanning && (event->buttons() & Qt::LeftButton)) {
        m_panOffset += delta;
        clampPanOffset();
        update();
        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void LiveView::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        if (m_drawingRoi) {
            m_drawingRoi = false;

            QRectF drag = QRectF(m_roiDragStart, event->position()).normalized();
            // Require a minimum drag size (avoid accidental single-pixel "ROIs")
            if (drag.width() > 4 && drag.height() > 4) {
                QPointF tl = widgetToVideo(drag.topLeft());
                QPointF br = widgetToVideo(drag.bottomRight());
                // Clamp to frame bounds
                tl.setX(qBound(0.0, tl.x(), (qreal)m_frameSize.width()));
                tl.setY(qBound(0.0, tl.y(), (qreal)m_frameSize.height()));
                br.setX(qBound(0.0, br.x(), (qreal)m_frameSize.width()));
                br.setY(qBound(0.0, br.y(), (qreal)m_frameSize.height()));

                m_roi = QRectF(tl, br).normalized();
                emit roiDefined(m_roi);
            }
            update();
            event->accept();
            return;
        }

        if (m_isPanning) {
            m_isPanning = false;
            updateCursor();
            event->accept();
            return;
        }
    }
    QWidget::mouseReleaseEvent(event);
}

void LiveView::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton && !m_roiMode) {
        resetView();
        event->accept();
        return;
    }
    QWidget::mouseDoubleClickEvent(event);
}

// ---------------------------------------------------------------------------
// Wheel — mirrors VideoLoader trackpad/mouse handling
// ---------------------------------------------------------------------------

void LiveView::wheelEvent(QWheelEvent* event)
{
    if (m_frame.isNull()) { event->ignore(); return; }

    int    steps = 0;
    double zs    = 0.15;
    const QPointF zoomAtPos = event->position();

    const bool isTouchpad = (event->device()->type() == QInputDevice::DeviceType::TouchPad);
    const bool hasPixel   = !event->pixelDelta().isNull() && event->pixelDelta().y() != 0;
    const bool hasAngle   = !event->angleDelta().isNull() && event->angleDelta().y() != 0;

    if (isTouchpad && hasPixel) {
        steps = event->pixelDelta().y();
        zs    = 0.02;
    } else if (hasAngle) {
        steps = event->angleDelta().y() / 120;
        zs    = 0.15;
    } else if (hasPixel) {
        int py = event->pixelDelta().y();
        steps  = py / 20;
        if (py != 0 && steps == 0) steps = (py > 0) ? 1 : -1;
    } else {
        event->ignore(); return;
    }

    if (steps == 0) { event->ignore(); return; }

    setZoomFactorAtPoint(m_zoomFactor * qPow(1.0 + zs, steps), zoomAtPos);
    event->accept();
}

void LiveView::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (!m_frame.isNull()) clampPanOffset();
}

// ---------------------------------------------------------------------------
// Private geometry helpers (direct port of VideoLoader math)
// ---------------------------------------------------------------------------

void LiveView::setZoomFactorAtPoint(double factor, const QPointF& widgetPoint)
{
    const double newZoom = qBound(0.05, factor, 30.0);
    if (qFuzzyCompare(m_zoomFactor, newZoom)) return;

    QRectF oldTarget = calculateTargetRect();
    QPointF videoPoint;
    if (oldTarget.isValid() && oldTarget.width() > 0 && oldTarget.height() > 0) {
        double nx = (widgetPoint.x() - oldTarget.left()) / oldTarget.width();
        double ny = (widgetPoint.y() - oldTarget.top())  / oldTarget.height();
        videoPoint = QPointF(nx * m_frameSize.width(), ny * m_frameSize.height());
    }

    m_zoomFactor = newZoom;

    if (!videoPoint.isNull() && m_frameSize.isValid()) {
        QRectF newTarget = calculateTargetRect();
        double nx = videoPoint.x() / m_frameSize.width();
        double ny = videoPoint.y() / m_frameSize.height();
        QPointF newWidgetPoint(newTarget.left() + nx * newTarget.width(),
                               newTarget.top()  + ny * newTarget.height());
        m_panOffset += (widgetPoint - newWidgetPoint);
    }

    clampPanOffset();
    update();
}

QRectF LiveView::calculateTargetRect() const
{
    if (m_frameSize.isEmpty() || m_zoomFactor <= 0) return QRectF();
    QSizeF ws = size();
    QSizeF scaled = QSizeF(m_frameSize);
    scaled.scale(ws, Qt::KeepAspectRatio);
    QSizeF zoomed = scaled * m_zoomFactor;
    QPointF tl((ws.width()  - zoomed.width())  / 2.0,
               (ws.height() - zoomed.height()) / 2.0);
    return QRectF(tl + m_panOffset, zoomed);
}

void LiveView::clampPanOffset()
{
    if (m_frameSize.isEmpty() || m_zoomFactor <= 0) return;
    QRectF  tr  = calculateTargetRect();
    QSizeF  ws  = size();
    QPointF unp = tr.topLeft() - m_panOffset;

    if (tr.width() <= ws.width())
        m_panOffset.setX((ws.width() - tr.width()) / 2.0 - unp.x());
    else {
        m_panOffset.setX(qBound(ws.width()  - tr.width()  - unp.x() - 50,
                                m_panOffset.x(),
                                -unp.x() + 50));
    }
    if (tr.height() <= ws.height())
        m_panOffset.setY((ws.height() - tr.height()) / 2.0 - unp.y());
    else {
        m_panOffset.setY(qBound(ws.height() - tr.height() - unp.y() - 50,
                                m_panOffset.y(),
                                -unp.y() + 50));
    }
}

QPointF LiveView::widgetToVideo(const QPointF& wp) const
{
    QRectF tr = calculateTargetRect();
    if (!tr.isValid() || tr.width() <= 0 || tr.height() <= 0) return {};
    double nx = (wp.x() - tr.left()) / tr.width();
    double ny = (wp.y() - tr.top())  / tr.height();
    return QPointF(nx * m_frameSize.width(), ny * m_frameSize.height());
}

QPointF LiveView::videoToWidget(const QPointF& vp) const
{
    QRectF tr = calculateTargetRect();
    if (!tr.isValid() || m_frameSize.isEmpty()) return {};
    double nx = vp.x() / m_frameSize.width();
    double ny = vp.y() / m_frameSize.height();
    return QPointF(tr.left() + nx * tr.width(),
                   tr.top()  + ny * tr.height());
}

void LiveView::updateCursor()
{
    if (m_scaleMeasureMode || m_roiMode)
        setCursor(Qt::CrossCursor);
    else if (m_isPanning)
        setCursor(Qt::ClosedHandCursor);
    else
        setCursor(Qt::OpenHandCursor);
}
