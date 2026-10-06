#ifndef DEBUGEXPORTER_H
#define DEBUGEXPORTER_H

#include <QString>
#include "debugrecords.h"
#include "../data/trackingcommon.h"

class TrackingDataStorage;

namespace Debug {

class DebugDataStore;

class DebugExporter {
public:
    struct Snapshot {
        int wormId = -1;
        int frameNumber = -1;
        CenterlineFrameDebug record;
        Tracking::DetectedBlob blob;
        Tracking::DetectedBlob previousBlob;
        bool hasPreviousBlob = false;
    };
    static bool captureCenterlineFrame(const TrackingDataStorage* storage,
                                       const DebugDataStore* debugStore,
                                       int wormId, int frameNumber, Snapshot& snapshot,
                                       QString* error = nullptr);
    static bool exportCenterlineFrame(const Snapshot& snapshot, const QString& outputDir,
                                      QString* error = nullptr);
    static bool exportCenterlineFrame(const TrackingDataStorage* storage,
                                      const DebugDataStore* debugStore,
                                      int wormId,
                                      int frameNumber,
                                      const QString& outputDir,
                                      QString* outErrorMsg = nullptr);
};

} // namespace Debug

#endif // DEBUGEXPORTER_H
