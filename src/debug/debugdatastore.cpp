#include "debugdatastore.h"

namespace Debug {

void DebugDataStore::clearAll()
{
    clearCenterline();
}

void DebugDataStore::clearCenterline()
{
    m_centerlineByWorm.clear();
}

void DebugDataStore::clearCenterlineForWorm(int wormId)
{
    m_centerlineByWorm.remove(wormId);
}

void DebugDataStore::setCenterlineFrame(const CenterlineFrameDebug& record)
{
    if (record.wormId < 0 || record.frameNumber < 0) {
        return;
    }
    m_centerlineByWorm[record.wormId][record.frameNumber] = record;
}

bool DebugDataStore::getCenterlineFrame(int wormId, int frameNumber,
                                        CenterlineFrameDebug& out) const
{
    const auto wormIt = m_centerlineByWorm.constFind(wormId);
    if (wormIt == m_centerlineByWorm.constEnd()) {
        return false;
    }
    const auto frameIt = wormIt->constFind(frameNumber);
    if (frameIt == wormIt->constEnd()) {
        return false;
    }
    out = frameIt.value();
    return true;
}

QMap<int, CenterlineFrameDebug> DebugDataStore::getCenterlineFramesForWorm(int wormId) const
{
    return m_centerlineByWorm.value(wormId);
}

} // namespace Debug
