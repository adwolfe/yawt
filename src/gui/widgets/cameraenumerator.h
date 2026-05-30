#pragma once

#include "icamerasource.h"
#include <QList>
#include <memory>

/**
 * CameraEnumerator — discovers all available cameras across all compiled backends.
 *
 * Probes OpenCV device indices 0-4 (skips any that won't open).
 * If built with YAWT_HAS_PYLON, also queries Pylon's transport-layer factory.
 *
 * Call enumerate() from the GUI thread (or a short-lived worker thread);
 * it may briefly open and close cameras to verify they are accessible.
 *
 * Call create() to get a ready-to-use ICameraSource for a discovered device.
 * The returned source must be opened before use (call open() from the worker thread).
 */
class CameraEnumerator
{
public:
    CameraEnumerator() = delete;

    /** Return all cameras currently visible to the system. */
    static QList<CameraInfo> enumerate();

    /** Create (but do not open) a camera source for the given descriptor. */
    static std::unique_ptr<ICameraSource> create(const CameraInfo& info);
};
