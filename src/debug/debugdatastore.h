#ifndef DEBUGDATASTORE_H
#define DEBUGDATASTORE_H

#include <QMap>

#include "debugrecords.h"

namespace Debug {

class DebugDataStore {
public:
    void clearAll();
    void clearCenterline();
    void clearCenterlineForWorm(int wormId);

    void setCenterlineFrame(const CenterlineFrameDebug& record);
    bool getCenterlineFrame(int wormId, int frameNumber,
                            CenterlineFrameDebug& out) const;
    QMap<int, CenterlineFrameDebug> getCenterlineFramesForWorm(int wormId) const;

private:
    // wormId -> frameNumber -> recorded live centerline diagnostics
    QMap<int, QMap<int, CenterlineFrameDebug>> m_centerlineByWorm;
};

} // namespace Debug

#endif // DEBUGDATASTORE_H
