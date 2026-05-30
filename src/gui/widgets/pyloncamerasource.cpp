#include "pyloncamerasource.h"

#ifdef YAWT_HAS_PYLON

#include <QDebug>
#include <opencv2/imgproc.hpp>

// ---------------------------------------------------------------------------
// RAII singleton: initialise / terminate the Pylon runtime once per process.
// ---------------------------------------------------------------------------
namespace {
struct PylonRuntime {
    PylonRuntime()  { Pylon::PylonInitialize(); }
    ~PylonRuntime() { Pylon::PylonTerminate();  }
};
void ensurePylonRuntime() { static PylonRuntime r; (void)r; }
} // namespace

// ---------------------------------------------------------------------------
// PylonCameraSource
// ---------------------------------------------------------------------------

PylonCameraSource::PylonCameraSource(const Pylon::CDeviceInfo& deviceInfo,
                                     const QString& displayName)
    : m_deviceInfo(deviceInfo)
    , m_displayName(displayName)
{
    ensurePylonRuntime();
    m_converter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
}

PylonCameraSource::~PylonCameraSource()
{
    close();
}

bool PylonCameraSource::open(int width, int height, double fps)
{
    if (m_open) return true;
    try {
        Pylon::CTlFactory& tlf = Pylon::CTlFactory::GetInstance();
        m_camera.Attach(tlf.CreateDevice(m_deviceInfo));
        m_camera.Open();

        if (width > 0 && height > 0) {
            if (GenApi::IsWritable(m_camera.Width))  m_camera.Width.SetValue(width);
            if (GenApi::IsWritable(m_camera.Height)) m_camera.Height.SetValue(height);
        }
        if (fps > 0) {
            try {
                if (GenApi::IsWritable(m_camera.AcquisitionFrameRateEnable))
                    m_camera.AcquisitionFrameRateEnable.SetValue(true);
                if (GenApi::IsWritable(m_camera.AcquisitionFrameRate))
                    m_camera.AcquisitionFrameRate.SetValue(fps);
            } catch (...) {
                qWarning() << "[Pylon] Could not set frame rate.";
            }
        }

        m_width  = static_cast<int>(m_camera.Width.GetValue());
        m_height = static_cast<int>(m_camera.Height.GetValue());
        try { m_fps = m_camera.ResultingFrameRate.GetValue(); }
        catch (...) { m_fps = fps > 0 ? fps : 30.0; }

        m_camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly,
                               Pylon::GrabLoop_ProvidedByUser);
        m_open = true;
        return true;

    } catch (const Pylon::GenericException& e) {
        qWarning() << "[Pylon] open() failed:" << e.GetDescription();
        return false;
    }
}

void PylonCameraSource::close()
{
    if (!m_open) return;
    try {
        if (m_camera.IsGrabbing()) m_camera.StopGrabbing();
        if (m_camera.IsOpen())     m_camera.Close();
    } catch (const Pylon::GenericException& e) {
        qWarning() << "[Pylon] close() error:" << e.GetDescription();
    }
    m_open = false;
}

bool PylonCameraSource::isOpen() const
{
    return m_open && m_camera.IsOpen();
}

bool PylonCameraSource::grabFrame(cv::Mat& frame)
{
    if (!m_open || !m_camera.IsGrabbing()) return false;
    try {
        Pylon::CGrabResultPtr result;
        if (!m_camera.RetrieveResult(
                static_cast<unsigned int>(1000.0 / m_fps * 2.5),
                result, Pylon::TimeoutHandling_Return))
            return false;

        if (!result->GrabSucceeded()) {
            qWarning() << "[Pylon] Grab error:" << result->GetErrorDescription();
            return false;
        }

        Pylon::CPylonImage img;
        m_converter.Convert(img, result);
        cv::Mat tmp(static_cast<int>(result->GetHeight()),
                    static_cast<int>(result->GetWidth()),
                    CV_8UC3, img.GetBuffer());
        frame = tmp.clone();
        return true;

    } catch (const Pylon::GenericException& e) {
        qWarning() << "[Pylon] grabFrame() exception:" << e.GetDescription();
        return false;
    }
}

// ---------------------------------------------------------------------------
// Parameter interface
// ---------------------------------------------------------------------------

CameraParamSet PylonCameraSource::queryParams() const
{
    CameraParamSet ps;
    if (!m_open) return ps;

    try {
        // --- Exposure ---
        if (GenApi::IsAvailable(m_camera.ExposureTime)) {
            ps.exposure.supported   = true;
            ps.exposure.minVal      = m_camera.ExposureTime.GetMin();
            ps.exposure.maxVal      = m_camera.ExposureTime.GetMax();
            ps.exposure.step        = m_camera.ExposureTime.GetInc();
            ps.exposure.currentVal  = m_camera.ExposureTime.GetValue();
            if (GenApi::IsAvailable(m_camera.ExposureAuto)) {
                ps.exposure.hasAuto    = true;
                ps.exposure.autoEnabled =
                    m_camera.ExposureAuto.GetValue() != Basler_UniversalCameraParams::ExposureAutoEnums::ExposureAuto_Off;
            }
        }

        // --- Gain ---
        if (GenApi::IsAvailable(m_camera.Gain)) {
            ps.gain.supported   = true;
            ps.gain.minVal      = m_camera.Gain.GetMin();
            ps.gain.maxVal      = m_camera.Gain.GetMax();
            ps.gain.step        = m_camera.Gain.GetInc();
            ps.gain.currentVal  = m_camera.Gain.GetValue();
            if (GenApi::IsAvailable(m_camera.GainAuto)) {
                ps.gain.hasAuto    = true;
                ps.gain.autoEnabled =
                    m_camera.GainAuto.GetValue() != Basler_UniversalCameraParams::GainAutoEnums::GainAuto_Off;
            }
        }

        // --- Gamma ---
        if (GenApi::IsAvailable(m_camera.Gamma)) {
            ps.gamma.supported  = true;
            ps.gamma.minVal     = m_camera.Gamma.GetMin();
            ps.gamma.maxVal     = m_camera.Gamma.GetMax();
            ps.gamma.step       = m_camera.Gamma.GetInc();
            ps.gamma.currentVal = m_camera.Gamma.GetValue();
        }

        // --- FPS ---
        if (GenApi::IsAvailable(m_camera.AcquisitionFrameRate)) {
            ps.fps.supported   = true;
            ps.fps.minVal      = m_camera.AcquisitionFrameRate.GetMin();
            ps.fps.maxVal      = m_camera.AcquisitionFrameRate.GetMax();
            ps.fps.step        = m_camera.AcquisitionFrameRate.GetInc();
            ps.fps.currentVal  = m_camera.AcquisitionFrameRate.GetValue();
        }

        // Brightness not a standard Pylon GenICam node — leave unsupported.

    } catch (const Pylon::GenericException& e) {
        qWarning() << "[Pylon] queryParams() error:" << e.GetDescription();
    }

    return ps;
}

bool PylonCameraSource::setParam(CameraParam p, double value)
{
    if (!m_open) return false;
    try {
        switch (p) {
        case CameraParam::ExposureUs:
            if (GenApi::IsWritable(m_camera.ExposureTime)) {
                m_camera.ExposureTime.SetValue(value);
                return true;
            }
            break;
        case CameraParam::GainDb:
            if (GenApi::IsWritable(m_camera.Gain)) {
                m_camera.Gain.SetValue(value);
                return true;
            }
            break;
        case CameraParam::Gamma:
            if (GenApi::IsWritable(m_camera.Gamma)) {
                m_camera.Gamma.SetValue(value);
                return true;
            }
            break;
        case CameraParam::FPS:
            if (GenApi::IsWritable(m_camera.AcquisitionFrameRate)) {
                m_camera.AcquisitionFrameRate.SetValue(value);
                // Update cached fps
                try { m_fps = m_camera.ResultingFrameRate.GetValue(); }
                catch (...) { m_fps = value; }
                return true;
            }
            break;
        default:
            break;
        }
    } catch (const Pylon::GenericException& e) {
        qWarning() << "[Pylon] setParam() error:" << e.GetDescription();
    }
    return false;
}

bool PylonCameraSource::setParamAuto(CameraParam p, bool enabled)
{
    if (!m_open) return false;
    try {
        using namespace Basler_UniversalCameraParams;
        switch (p) {
        case CameraParam::ExposureUs:
            if (GenApi::IsWritable(m_camera.ExposureAuto)) {
                m_camera.ExposureAuto.SetValue(
                    enabled ? ExposureAuto_Continuous : ExposureAuto_Off);
                return true;
            }
            break;
        case CameraParam::GainDb:
            if (GenApi::IsWritable(m_camera.GainAuto)) {
                m_camera.GainAuto.SetValue(
                    enabled ? GainAuto_Continuous : GainAuto_Off);
                return true;
            }
            break;
        default:
            break;
        }
    } catch (const Pylon::GenericException& e) {
        qWarning() << "[Pylon] setParamAuto() error:" << e.GetDescription();
    }
    return false;
}

#endif // YAWT_HAS_PYLON
