#include "opencvcamerasource.h"
#include <cmath>

OpenCVCameraSource::OpenCVCameraSource(int deviceIndex, const QString& name)
    : m_deviceIndex(deviceIndex)
    , m_displayName(name.isEmpty()
                    ? QString("Camera %1 (OpenCV)").arg(deviceIndex)
                    : name)
{}

bool OpenCVCameraSource::open(int width, int height, double fps)
{
    m_cap.open(m_deviceIndex);
    if (!m_cap.isOpened()) return false;

    if (width > 0 && height > 0) {
        m_cap.set(cv::CAP_PROP_FRAME_WIDTH,  width);
        m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    }
    if (fps > 0)
        m_cap.set(cv::CAP_PROP_FPS, fps);

    m_width  = static_cast<int>(m_cap.get(cv::CAP_PROP_FRAME_WIDTH));
    m_height = static_cast<int>(m_cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    m_fps    = m_cap.get(cv::CAP_PROP_FPS);
    if (m_fps <= 0) m_fps = fps > 0 ? fps : 30.0;

    return true;
}

void OpenCVCameraSource::close()
{
    if (m_cap.isOpened())
        m_cap.release();
}

bool OpenCVCameraSource::isOpen() const
{
    return m_cap.isOpened();
}

bool OpenCVCameraSource::grabFrame(cv::Mat& frame)
{
    if (!m_cap.isOpened()) return false;
    return m_cap.read(frame) && !frame.empty();
}

// ---------------------------------------------------------------------------
// Parameter probing
// ---------------------------------------------------------------------------

bool OpenCVCameraSource::probeProperty(int propId,
                                       double testValue,
                                       double& outCurrent) const
{
    if (!m_cap.isOpened()) return false;

    double original = m_cap.get(propId);

    // Guarantee the probe value is meaningfully different from the current value.
    // If testValue ≈ original we'd get a false positive (setting the same value
    // always "succeeds"), so nudge the probe to something clearly distinct.
    double probe = testValue;
    if (std::abs(probe - original) < std::max(0.5, std::abs(original) * 0.1))
        probe = (original > 1.0) ? original * 0.5 : original + 1.0;

    m_cap.set(propId, probe);
    double after = m_cap.get(propId);

    // Restore original value.
    m_cap.set(propId, original);
    outCurrent = m_cap.get(propId);

    // Supported only if the camera actually responded to the probe
    // (the readback changed away from the original value).
    return std::abs(after - original) > 0.5;
}

CameraParamSet OpenCVCameraSource::queryParams() const
{
    CameraParamSet ps;
    if (!m_cap.isOpened()) return ps;

    // --- FPS ---
    // Probe with a value clearly different from the current rate.
    // Note: on macOS (AVFoundation) the round-trip may succeed while the
    // actual hardware capture rate stays fixed; software limiting in the
    // worker compensates for this (see CaptureWorker::applyParam / timer).
    {
        double cur = 0;
        double testFps = (m_fps >= 20.0) ? 10.0 : 60.0;
        bool ok = probeProperty(cv::CAP_PROP_FPS, testFps, cur);
        ps.fps.supported  = ok;
        ps.fps.softwareOnly = !ok; // if hardware probe fails, worker uses software limiting
        ps.fps.minVal     = 1.0;
        ps.fps.maxVal     = 120.0;
        ps.fps.step       = 1.0;
        ps.fps.currentVal = cur > 0 ? cur : m_fps;
    }

    // --- Exposure ---
    // CAP_PROP_AUTO_EXPOSURE: 0.25 = manual, 0.75 = auto (driver-defined constants)
    {
        double autoVal = m_cap.get(cv::CAP_PROP_AUTO_EXPOSURE);
        double cur = 0;
        bool ok = probeProperty(cv::CAP_PROP_EXPOSURE, -5.0, cur);
        ps.exposure.supported   = ok;
        ps.exposure.hasAuto     = true;   // assume present; UI will reveal if toggle does nothing
        ps.exposure.autoEnabled = autoVal > 0.5;
        // OpenCV exposure is often in log2 seconds (e.g. -7 = 1/128 s) on V4L2 backends
        // and in ms on some others. We expose the raw range and label accordingly.
        ps.exposure.minVal      = -13.0;  // ~120 µs in log2-s
        ps.exposure.maxVal      =  0.0;   // 1 s
        ps.exposure.step        =  1.0;
        ps.exposure.currentVal  = cur;
    }

    // --- Gain ---
    {
        double cur = 0;
        bool ok = probeProperty(cv::CAP_PROP_GAIN, 50.0, cur);
        ps.gain.supported  = ok;
        ps.gain.minVal     = 0.0;
        ps.gain.maxVal     = 100.0;
        ps.gain.step       = 1.0;
        ps.gain.currentVal = cur;
    }

    // --- Gamma ---
    {
        double cur = 0;
        bool ok = probeProperty(cv::CAP_PROP_GAMMA, 100.0, cur);
        ps.gamma.supported  = ok;
        ps.gamma.minVal     = 1.0;
        ps.gamma.maxVal     = 500.0;
        ps.gamma.step       = 1.0;
        ps.gamma.currentVal = cur;
    }

    // --- Brightness ---
    {
        double cur = 0;
        bool ok = probeProperty(cv::CAP_PROP_BRIGHTNESS, 128.0, cur);
        ps.brightness.supported  = ok;
        ps.brightness.minVal     = 0.0;
        ps.brightness.maxVal     = 255.0;
        ps.brightness.step       = 1.0;
        ps.brightness.currentVal = cur;
    }

    return ps;
}

bool OpenCVCameraSource::setParam(CameraParam p, double value)
{
    if (!m_cap.isOpened()) return false;
    switch (p) {
    case CameraParam::ExposureUs:  return m_cap.set(cv::CAP_PROP_EXPOSURE,   value);
    case CameraParam::GainDb:      return m_cap.set(cv::CAP_PROP_GAIN,       value);
    case CameraParam::Gamma:       return m_cap.set(cv::CAP_PROP_GAMMA,      value);
    case CameraParam::FPS:         return m_cap.set(cv::CAP_PROP_FPS,        value);
    case CameraParam::Brightness:  return m_cap.set(cv::CAP_PROP_BRIGHTNESS, value);
    }
    return false;
}

bool OpenCVCameraSource::setParamAuto(CameraParam p, bool enabled)
{
    if (!m_cap.isOpened()) return false;
    if (p == CameraParam::ExposureUs) {
        // V4L2: 0.25 = manual, 0.75 = auto
        return m_cap.set(cv::CAP_PROP_AUTO_EXPOSURE, enabled ? 0.75 : 0.25);
    }
    return false;
}
