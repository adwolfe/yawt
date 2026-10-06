#include "framecache.h"
#include "../../utils/loggingcategories.h"
#include <QMutexLocker>
#include <QDebug>

// ============================================================================
// FrameCache Implementation
// ============================================================================

FrameCache::FrameCache(int maxCacheSize)
    : m_maxSize(maxCacheSize), m_hits(0), m_requests(0) {
    YAWT_DEBUG(lcGuiVideoLoader) << "FrameCache created with max size:" << m_maxSize;
}

FrameCache::~FrameCache() {
    clear();
    YAWT_INFO(lcGuiVideoLoader) << "FrameCache destroyed. Final hit rate:" << hitRate() << "%";
}

void FrameCache::insertFrame(int frameNumber, const cv::Mat& frame) {
    if (frame.empty()) return;

    QMutexLocker locker(&m_mutex);

    // Remove existing frame if present
    m_frames.remove(frameNumber);

    // Add new frame
    m_frames.insert(frameNumber, CachedFrame(frameNumber, frame));

    // Evict if over capacity
    while (m_frames.size() > m_maxSize) {
        evictLRU();
    }

    YAWT_DEBUG(lcGuiVideoLoader) << "FrameCache: Cached frame" << frameNumber << "- Cache size:" << m_frames.size();
}

bool FrameCache::getFrame(int frameNumber, cv::Mat& outFrame) {
    QMutexLocker locker(&m_mutex);
    m_requests.fetchAndAddOrdered(1);

    auto it = m_frames.find(frameNumber);
    if (it != m_frames.end() && it->isValid) {
        // Return a shallow copy to avoid an expensive deep clone on the main thread.
        outFrame = it->rawFrame;
        updateAccessTime(frameNumber);
        m_hits.fetchAndAddOrdered(1);
        return true;
    }

    return false;
}

bool FrameCache::hasFrame(int frameNumber) const {
    QMutexLocker locker(&m_mutex);
    auto it = m_frames.find(frameNumber);
    return (it != m_frames.end() && it->isValid);
}

void FrameCache::clear() {
    QMutexLocker locker(&m_mutex);
    m_frames.clear();
    YAWT_DEBUG(lcGuiVideoLoader) << "FrameCache: Cleared all frames";
}

void FrameCache::setMaxSize(int maxSize) {
    QMutexLocker locker(&m_mutex);
    m_maxSize = maxSize;
    while (m_frames.size() > m_maxSize) {
        evictLRU();
    }
}

int FrameCache::size() const {
    QMutexLocker locker(&m_mutex);
    return m_frames.size();
}

int FrameCache::maxSize() const {
    QMutexLocker locker(&m_mutex);
    return m_maxSize;
}

double FrameCache::hitRate() const {
    int requests = m_requests;
    int hits = m_hits;
    return requests > 0 ? (static_cast<double>(hits) / requests) * 100.0 : 0.0;
}

void FrameCache::evictLRU() {
    if (m_frames.isEmpty()) return;

    // Find frame with oldest access time
    auto oldest = m_frames.begin();
    for (auto it = m_frames.begin(); it != m_frames.end(); ++it) {
        if (it->lastAccessed < oldest->lastAccessed) {
            oldest = it;
        }
    }

    YAWT_DEBUG(lcGuiVideoLoader) << "FrameCache: Evicting frame" << oldest->frameNumber;
    m_frames.erase(oldest);
}

void FrameCache::updateAccessTime(int frameNumber) {
    auto it = m_frames.find(frameNumber);
    if (it != m_frames.end()) {
        it->lastAccessed = QDateTime::currentDateTime();
    }
}

