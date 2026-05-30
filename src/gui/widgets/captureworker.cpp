#include "captureworker.h"
#include <QDebug>
#include <QRegularExpression>

CaptureWorker::CaptureWorker(QObject* parent)
    : QObject(parent)
{}

CaptureWorker::~CaptureWorker()
{
    stopCapture();
}

void CaptureWorker::startCapture(ICameraSource* source, int width, int height, double fps)
{
    m_source.reset(source);  // take ownership immediately

    if (m_timer && m_timer->isActive())
        stopCapture();

    if (!m_source->open(width, height, fps)) {
        emit cameraError(QString("Could not open %1").arg(m_source->displayName()));
        m_source.reset();
        return;
    }

    emit cameraOpened(m_source->actualWidth(),
                      m_source->actualHeight(),
                      m_source->actualFps());

    // Query and emit parameter capabilities so the panel can configure its UI.
    emit paramsReady(m_source->queryParams());

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &CaptureWorker::grabFrame);
    const double timerFps = m_source->actualFps() > 0 ? m_source->actualFps() : fps;
    m_timer->start(static_cast<int>(1000.0 / timerFps * 0.9));
}

void CaptureWorker::stopCapture()
{
    if (m_timer) {
        m_timer->stop();
        m_timer->deleteLater();
        m_timer = nullptr;
    }
    if (m_recording) stopRecording();
    if (m_source) {
        m_source->close();
        m_source.reset();
    }
    m_desiredFps = 0.0;
}

void CaptureWorker::startRecording(const QString& filePath)
{
    if (!m_source || !m_source->isOpen()) {
        emit cameraError("Cannot record: camera not open");
        return;
    }
    if (m_recording) return;

    // VideoWriter dimensions match what grabFrame() will actually write:
    // the ROI region if set, otherwise the full frame.
    cv::Rect effectiveRoi;
    if (!m_roi.isEmpty()) {
        effectiveRoi = cv::Rect(m_roi.x(), m_roi.y(), m_roi.width(), m_roi.height())
                       & cv::Rect(0, 0, m_source->actualWidth(), m_source->actualHeight());
    }
    const int    outW   = effectiveRoi.area() > 0 ? effectiveRoi.width  : m_source->actualWidth();
    const int    outH   = effectiveRoi.area() > 0 ? effectiveRoi.height : m_source->actualHeight();
    // Use the user-requested rate if set (software limiting); otherwise hardware rate.
    const double fps = (m_desiredFps > 0) ? m_desiredFps : m_source->actualFps();
    const cv::Size sz(outW, outH);

    int fourcc = cv::VideoWriter::fourcc('a','v','c','1');
    m_writer.open(filePath.toStdString(), fourcc, fps, sz);

    if (!m_writer.isOpened()) {
        QString aviPath = filePath;
        aviPath.replace(
            QRegularExpression("\\.mp4$", QRegularExpression::CaseInsensitiveOption),
            ".avi");
        fourcc = cv::VideoWriter::fourcc('M','J','P','G');
        m_writer.open(aviPath.toStdString(), fourcc, fps, sz);
        if (!m_writer.isOpened()) {
            emit cameraError("Could not open VideoWriter — recording aborted");
            return;
        }
        emit recordingStarted(aviPath);
    } else {
        emit recordingStarted(filePath);
    }

    m_recording  = true;
    m_frameCount = 0;
}

void CaptureWorker::stopRecording()
{
    if (!m_recording) return;
    m_recording = false;
    m_writer.release();
    emit recordingStopped(m_frameCount);
    m_frameCount = 0;
}

void CaptureWorker::applyParam(int p, double value)
{
    if (!m_source || !m_source->isOpen()) return;

    m_source->setParam(static_cast<CameraParam>(p), value);

    // For FPS: also update the grab timer so software rate-limiting applies
    // even when the hardware ignores the request (common on macOS webcams).
    if (static_cast<CameraParam>(p) == CameraParam::FPS && value > 0) {
        m_desiredFps = value;
        if (m_timer && m_timer->isActive())
            m_timer->setInterval(static_cast<int>(1000.0 / m_desiredFps * 0.9));
    }
}

void CaptureWorker::applyParamAuto(int p, bool enabled)
{
    if (m_source && m_source->isOpen())
        m_source->setParamAuto(static_cast<CameraParam>(p), enabled);
}

void CaptureWorker::setRoi(QRect roi)
{
    m_roi = roi;
}

void CaptureWorker::clearRoi()
{
    m_roi = QRect();
}

void CaptureWorker::grabFrame()
{
    if (!m_source || !m_source->isOpen()) return;

    cv::Mat frame;
    if (!m_source->grabFrame(frame) || frame.empty()) return;

    // Always emit the full frame so LiveView can display the ROI overlay correctly.
    emit frameCaptured(frame);

    // Only crop when writing to disk — the ROI is a recording crop, not a preview crop.
    if (m_recording && m_writer.isOpened()) {
        if (!m_roi.isEmpty()) {
            cv::Rect crop(m_roi.x(), m_roi.y(), m_roi.width(), m_roi.height());
            crop &= cv::Rect(0, 0, frame.cols, frame.rows);
            if (crop.area() > 0)
                m_writer.write(frame(crop));
        } else {
            m_writer.write(frame);
        }
    }
}
