#pragma once

#include <QString>
#include <QDateTime>

/**
 * VideoMetadataStore — reads and writes per-video metadata JSON files.
 *
 * File location: <dataDir>/<videoBaseName>_metadata.json
 * The dataDir is the "yawt" folder created by VideoLoader alongside a video file.
 *
 * File format (v1):
 * {
 *   "version": 1,
 *   "scale": {
 *     "pixelsPerUnit": 45.3,
 *     "unit": "mm",
 *     "physicalValue": 1.0,
 *     "pixelLength": 45.3,
 *     "timestamp": "2025-05-31T10:30:00"
 *   }
 * }
 *
 * Only the scale section is written/read today; the file is structured so
 * future metadata (capture settings, notes, etc.) can be added without
 * breaking existing readers.
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

    /** Save (or update) the scale section of the metadata file. Other sections
     *  in an existing file are preserved. Returns true on success. */
    static bool saveScale(const QString& dataDir,
                          const QString& videoBaseName,
                          const ScaleCalibration& cal);

    /** Load the scale section. Returns false if the file doesn't exist or has
     *  no valid scale data — @p cal is left untouched in that case. */
    static bool loadScale(const QString& dataDir,
                          const QString& videoBaseName,
                          ScaleCalibration& cal);
};
