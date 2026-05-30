#pragma once

#include "icamerasource.h"
#include <opencv2/videoio.hpp>

/**
 * OpenCVCameraSource — wraps cv::VideoCapture for generic USB/built-in cameras.
 *
 * Parameter support is driver-dependent. queryParams() probes each property
 * by attempting a round-trip set/get; if the camera ignores it the parameter
 * is reported as unsupported so the UI can disable that control.
 */
class OpenCVCameraSource : public ICameraSource
{
public:
    explicit OpenCVCameraSource(int deviceIndex, const QString& name = QString());

    bool open(int width, int height, double fps) override;
    void close() override;
    bool isOpen() const override;
    bool grabFrame(cv::Mat& frame) override;

    int    actualWidth()  const override { return m_width; }
    int    actualHeight() const override { return m_height; }
    double actualFps()    const override { return m_fps; }

    QString displayName() const override { return m_displayName; }
    QString backendName() const override { return QStringLiteral("OpenCV"); }

    CameraParamSet queryParams() const override;
    bool setParam(CameraParam p, double value) override;
    bool setParamAuto(CameraParam p, bool enabled) override;

private:
    /** Probe whether the camera honours a cv::CAP_PROP_* by set/get round-trip. */
    bool probeProperty(int propId, double testValue, double& outCurrent) const;

    int              m_deviceIndex;
    QString          m_displayName;
    mutable cv::VideoCapture m_cap;  // mutable: probeProperty does restore-after-test in const context
    int    m_width  = 0;
    int    m_height = 0;
    double m_fps    = 0.0;
};
