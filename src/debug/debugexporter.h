#ifndef DEBUGEXPORTER_H
#define DEBUGEXPORTER_H

#include <QString>

class TrackingDataStorage;

namespace Debug {

class DebugDataStore;

class DebugExporter {
public:
    static bool exportCenterlineFrame(const TrackingDataStorage* storage,
                                      const DebugDataStore* debugStore,
                                      int wormId,
                                      int frameNumber,
                                      const QString& outputDir,
                                      QString* outErrorMsg = nullptr);
};

} // namespace Debug

#endif // DEBUGEXPORTER_H
