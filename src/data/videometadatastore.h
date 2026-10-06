#pragma once

#include <QString>
#include <QDateTime>

/**
 * VideoMetadataStore — reads and writes per-video metadata JSON files.
 *
 * File location: <dataDir>/<videoBaseName>_metadata.json
 * The dataDir is the "yawt" folder created by VideoLoader alongside a video file.
 *
 * File format (v1). Every key is optional. Each save* call merges into the
 * existing file and preserves the keys it does not own.
 * {
 *   "version": 1,
 *   "umPerPixel": 0.0221,          // Spatial scale in MICROMETERS PER PIXEL. This is the
 *                                  // canonical value read by tracking and analysis.
 *                                  // Written by saveUmPerPixel(); also derived and written
 *                                  // by saveScale() from the calibration below.
 *   "fps": 25.0,                   // Source video frame rate. saveFps() / loadFps().
 *   "scaleCalibration": {          // Raw measurement behind umPerPixel. saveScale() / loadScale().
 *     "pixelsPerUnit": 45.3,       //   pixelLength / physicalValue
 *     "unit": "mm",                //   mm | cm | inch | µm
 *     "physicalValue": 1.0,
 *     "pixelLength": 45.3,
 *     "timestamp": "2025-05-31T10:30:00"
 *   }
 * }
 *
 * Legacy key: "pixelSizeUm" held PIXELS PER MICROMETER in files written before
 * the unit flip. loadUmPerPixel() still accepts it (and inverts it) when
 * "umPerPixel" is absent. Nothing writes it any more; do not reintroduce it.
 */
class VideoMetadataStore
{
public:
    VideoMetadataStore() = delete;

    struct ScaleCalibration {
        double   pixelsPerUnit  = 0.0;  ///< computed: pixelLength / physicalValue
        QString  unit;                  ///< e.g. "mm"
        double   physicalValue  = 1.0;  ///< what the user entered
        double   pixelLength    = 0.0;  ///< raw measured pixel distance
        QDateTime timestamp;

        bool isValid() const { return pixelsPerUnit > 0 && !unit.isEmpty(); }
    };

    /** Canonical metadata path for a given data directory and video base name. */
    static QString metadataPath(const QString& dataDir,
                                const QString& videoBaseName);

    /** Save (or update) the "scaleCalibration" section of the metadata file.
     *  Other keys in an existing file are preserved. Also writes the top-level
     *  "umPerPixel" key derived from the calibration. Returns true on success. */
    static bool saveScale(const QString& dataDir,
                          const QString& videoBaseName,
                          const ScaleCalibration& cal);

    /** Queue a scale save on the serial metadata writer. Failures are logged. */
    static void saveScaleAsync(const QString& dataDir, const QString& videoBaseName,
                               const ScaleCalibration& cal);

    /** Load the "scaleCalibration" section. Returns false if the file doesn't
     *  exist or has no valid calibration — @p cal is left untouched in that case. */
    static bool loadScale(const QString& dataDir,
                          const QString& videoBaseName,
                          ScaleCalibration& cal);

    /** Save the spatial resolution as µm/pixel. Preserves all other metadata sections.
     *  This is the canonical display unit — the inverse of pixels/µm. */
    static bool saveUmPerPixel(const QString& dataDir,
                               const QString& videoBaseName,
                               double umPerPixel);

    /** Queue a spatial-resolution save on the serial metadata writer. */
    static void saveUmPerPixelAsync(const QString& dataDir, const QString& videoBaseName,
                                    double umPerPixel);

    /** Load the spatial resolution (µm/pixel). Returns false if not found;
     *  @p umPerPixel is left untouched in that case. */
    static bool loadUmPerPixel(const QString& dataDir,
                               const QString& videoBaseName,
                               double& umPerPixel);

    /** Read scale and fps in one file access. Missing values are returned as zero. */
    static void loadAnalysisMetadata(const QString& dataDir, const QString& videoBaseName,
                                     double& umPerPixel, double& fps);

    /** Save the video frame rate (frames per second). Preserves all other metadata sections. */
    static bool saveFps(const QString& dataDir,
                        const QString& videoBaseName,
                        double fps);

    /** Load the video frame rate. Returns false if not found;
     *  @p fps is left untouched in that case. */
    static bool loadFps(const QString& dataDir,
                        const QString& videoBaseName,
                        double& fps);
};
