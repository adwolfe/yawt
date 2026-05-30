#ifndef CAPTUREWORKER_H
#define CAPTUREWORKER_H

#include "icamerasource.h"
#include <QObject>
#include <QTimer>
#include <QRect>
#include <QString>
#include <memory>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>

class CaptureWorker : public QObject
{
    Q_OBJECT
public:
    explicit CaptureWorker(QObject* parent = nullptr);
    ~CaptureWorker() override;

public slots:
    /** Takes ownership of @p source, opens it, starts grab timer. */
    void startCapture(ICameraSource* source, int width, int height, double fps);
    void stopCapture();

    void startRecording(const QString& filePath);
    void stopRecording();

    /** Set camera parameter on the worker thread. p is cast of CameraParam. */
    void applyParam(int p, double value);
    void applyParamAuto(int p, bool enabled);

    /** Crop rectangle in video frame coordinates (0,0 = top-left of frame).
     *  An empty rect disables cropping. */
    void setRoi(QRect roi);
    void clearRoi();

signals:
    void frameCaptured(cv::Mat frame);
    void cameraOpened(int width, int height, double fps);
    void paramsReady(CameraParamSet params);
    void cameraError(QString message);
    void recordingStarted(QString path);
    void recordingStopped(int frameCount);

private slots:
    void grabFrame();

private:
    QTimer*                        m_timer      = nullptr;
    std::unique_ptr<ICameraSource> m_source;
    cv::VideoWriter                m_writer;
    QRect                          m_roi;          // empty = full frame
    double                         m_desiredFps = 0.0; // 0 = use hardware rate
    bool   m_recording  = false;
    int    m_frameCount = 0;
};

#endif // CAPTUREWORKER_H
