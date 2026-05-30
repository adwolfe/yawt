#pragma once

#include "icamerasource.h"

#ifdef YAWT_HAS_PYLON

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

/**
 * PylonCameraSource — Basler camera backend via the Pylon SDK.
 *
 * Wraps a single CInstantCamera.  Construction is cheap; the actual
 * device is not opened until open() is called.
 *
 * Thread-safety: all methods must be called from the same (worker) thread.
 */
class PylonCameraSource : public ICameraSource
{
public:
    explicit PylonCameraSource(const Pylon::CDeviceInfo& deviceInfo,
                               const QString& displayName);
    ~PylonCameraSource() override;

    bool open(int width, int height, double fps) override;
    void close() override;
    bool isOpen() const override;
    bool grabFrame(cv::Mat& frame) override;

    int    actualWidth()  const override { return m_width; }
    int    actualHeight() const override { return m_height; }
    double actualFps()    const override { return m_fps; }

    QString displayName() const override { return m_displayName; }
    QString backendName() const override { return QStringLiteral("Pylon"); }

    CameraParamSet queryParams() const override;
    bool setParam(CameraParam p, double value) override;
    bool setParamAuto(CameraParam p, bool enabled) override;

private:
    Pylon::CDeviceInfo                   m_deviceInfo;
    QString                              m_displayName;
    Pylon::CBaslerUniversalInstantCamera m_camera;
    Pylon::CImageFormatConverter         m_converter;
    bool   m_open   = false;
    int    m_width  = 0;
    int    m_height = 0;
    double m_fps    = 0.0;
};

#endif // YAWT_HAS_PYLON
