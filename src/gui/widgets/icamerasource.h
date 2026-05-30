#pragma once

#include <QString>
#include <QSize>
#include <QMetaType>
#include <opencv2/core.hpp>

/**
 * CameraInfo — lightweight descriptor returned by CameraEnumerator.
 * Passed between the GUI thread (panel) and the worker thread (CaptureWorker).
 * Must be copyable; contains no handles.
 */
struct CameraInfo {
    QString backend;        ///< "opencv" | "pylon"
    QString id;             ///< Opaque backend-specific identifier (e.g. "0" or Pylon serial)
    QString displayName;    ///< Human-readable label shown in the combo box
};

// ---------------------------------------------------------------------------
// Camera parameter system
// ---------------------------------------------------------------------------

/** Parameters that backends may or may not support. */
enum class CameraParam {
    ExposureUs,     ///< Exposure time in microseconds
    GainDb,         ///< Gain in dB (Pylon) or driver units (OpenCV)
    Gamma,          ///< Gamma correction (0.1 – 4.0 typical)
    FPS,            ///< Acquisition frame rate
    Brightness,     ///< Brightness offset (OpenCV fallback)
};

/**
 * Per-parameter capability report.
 * Queried from the camera after open(); used to configure the UI.
 */
struct ParamInfo {
    bool   supported    = false;  ///< false → widget stays disabled
    bool   softwareOnly = false;  ///< true → hardware ignores it; worker applies in software
    bool   hasAuto      = false;  ///< true → show Auto checkbox
    bool   autoEnabled  = false;  ///< current auto state
    double minVal       = 0.0;
    double maxVal       = 0.0;
    double step         = 1.0;
    double currentVal   = 0.0;
};

/**
 * Full snapshot of all queryable parameters, emitted after camera open.
 * Panel reads this once to configure spinbox ranges and enabled states.
 */
struct CameraParamSet {
    ParamInfo exposure;
    ParamInfo gain;
    ParamInfo gamma;
    ParamInfo fps;
    ParamInfo brightness;
};
Q_DECLARE_METATYPE(CameraParamSet)

// ---------------------------------------------------------------------------
// Abstract camera source
// ---------------------------------------------------------------------------

/**
 * ICameraSource — abstract camera backend.
 *
 * Implementations live entirely on the worker thread after construction.
 * All methods are called from the worker thread only.
 */
class ICameraSource
{
public:
    virtual ~ICameraSource() = default;

    /** Open the camera at the requested parameters.
     *  Implementations may silently clamp or round values.
     *  @return true on success. */
    virtual bool open(int width, int height, double fps) = 0;

    virtual void close() = 0;
    virtual bool isOpen() const = 0;

    /** Grab the next available frame into @p frame (BGR8).
     *  Should return quickly; blocks at most one frame period.
     *  @return true if @p frame was populated. */
    virtual bool grabFrame(cv::Mat& frame) = 0;

    virtual int    actualWidth()  const = 0;
    virtual int    actualHeight() const = 0;
    virtual double actualFps()    const = 0;

    virtual QString displayName() const = 0;
    virtual QString backendName() const = 0;

    // --- Parameter interface (called on worker thread after open()) ---

    /** Return capability/range info for each parameter. */
    virtual CameraParamSet queryParams() const = 0;

    /** Set a scalar parameter value. @return false if unsupported or out-of-range. */
    virtual bool setParam(CameraParam p, double value) = 0;

    /** Toggle automatic control for a parameter (e.g. auto-exposure).
     *  @return false if auto mode is not supported for this parameter. */
    virtual bool setParamAuto(CameraParam p, bool enabled) = 0;
};
