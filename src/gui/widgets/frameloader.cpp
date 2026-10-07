#include "frameloader.h"
#include "framecache.h"
#include "../../utils/yawtpaths.h"

#include <QDebug>
#include "../../utils/loggingcategories.h"
#include <QMutexLocker>
#include <QThread>
#include <QFileInfo>
#include "../../data/videometadatastore.h"
#include <algorithm>

// ============================================================================
// FrameLoader Implementation
// ============================================================================

FrameLoader::FrameLoader(QObject* parent)
    : QObject(parent), m_stopRequested(false), m_isProcessing(false), m_frameCache(nullptr) {
}

FrameLoader::~FrameLoader() {
    stop();
    if (m_videoCapture.isOpened()) {
        m_videoCapture.release();
    }
}

void FrameLoader::setVideoPath(const QString& path, quint64 generation) {
    QMutexLocker locker(&m_queueMutex);
    m_requestQueue.clear();
    m_inFlightFrame = -1;
    m_videoPath = path;
    m_generation = generation;
    m_openPending = true;
    // Serialize invalidation with worker insertion so an old frame cannot enter
    // the new video's cache after it has been cleared.
    if (m_frameCache) m_frameCache->clear();
    m_waitCondition.wakeOne();
}

void FrameLoader::openVideo(const QString& path, quint64 generation) {
    try {
        m_videoCapture.release();
        if (!m_videoCapture.open(path.toStdString())) {
            emit videoOpenFailed(generation, "Failed to open video file with OpenCV.");
            return;
        }
        const int count = static_cast<int>(m_videoCapture.get(cv::CAP_PROP_FRAME_COUNT));
        const QSize size(static_cast<int>(m_videoCapture.get(cv::CAP_PROP_FRAME_WIDTH)),
                         static_cast<int>(m_videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT)));
        double fps = m_videoCapture.get(cv::CAP_PROP_FPS);
        if (fps <= 0) fps = 25.0;
        if (count <= 0 || size.isEmpty()) {
            emit videoOpenFailed(generation, "Video has no frames or invalid dimensions.");
            return;
        }
        const QString dataDir = YawtPaths::ensureVideoDataDirectory(path);
        double umPerPixel = 0.0;
        if (!dataDir.isEmpty())
            VideoMetadataStore::loadUmPerPixel(dataDir, QFileInfo(path).completeBaseName(), umPerPixel);
        emit videoOpened(generation, count, fps, size, dataDir, umPerPixel);
    } catch (const cv::Exception& ex) {
        emit videoOpenFailed(generation, QString::fromUtf8(ex.what()));
    }
}

void FrameLoader::setFrameCache(std::shared_ptr<FrameCache> cache) {
    // Store a pointer to the shared FrameCache so the loader thread can
    // insert loaded frames directly into the cache without involving the UI thread.
    QMutexLocker locker(&m_queueMutex);
    m_frameCache = std::move(cache);
    YAWT_DEBUG(lcGuiVideoLoader) << "FrameLoader: frame cache set:" << (m_frameCache != nullptr);
}

void FrameLoader::requestFrames(const QList<int>& frameNumbers, int priority) {
    QMutexLocker locker(&m_queueMutex);

    bool anyAdded = false;
    for (int frameNumber : frameNumbers) {
        // Skip invalid frame numbers
        if (frameNumber < 0 || frameNumber == m_inFlightFrame) continue;

        // Skip if frame is already in cache
        if (m_frameCache && m_frameCache->hasFrame(frameNumber)) {
            continue;
        }

        // Skip if already queued
        bool alreadyQueued = false;
        for (const FrameLoadRequest& req : m_requestQueue) {
            if (req.frameNumber == frameNumber) { alreadyQueued = true; break; }
        }
        if (alreadyQueued) continue;

        m_requestQueue.enqueue(FrameLoadRequest(frameNumber, priority));
        anyAdded = true;
    }

    locker.unlock();
    if (anyAdded) {
        m_waitCondition.wakeOne();
    }
}

void FrameLoader::requestSingleFrame(int frameNumber, int priority) {
    QMutexLocker locker(&m_queueMutex);

    if (frameNumber < 0 || frameNumber == m_inFlightFrame) {
        return;
    }

    // If frame already cached, don't enqueue
    if (m_frameCache && m_frameCache->hasFrame(frameNumber)) {
        return;
    }

    for (auto it = m_requestQueue.begin(); it != m_requestQueue.end();) {
        if (it->frameNumber == frameNumber || (priority >= 100 && it->priority >= 100))
            it = m_requestQueue.erase(it);
        else ++it;
    }

    m_requestQueue.enqueue(FrameLoadRequest(frameNumber, priority));
    locker.unlock();
    m_waitCondition.wakeOne();
}

void FrameLoader::clearRequests() {
    QMutexLocker locker(&m_queueMutex);
    m_requestQueue.clear();
    YAWT_DEBUG(lcGuiVideoLoader) << "FrameLoader: Cleared all pending requests";
}

void FrameLoader::stop() {
    QMutexLocker locker(&m_queueMutex);
    m_stopRequested = true;
    locker.unlock();
    m_waitCondition.wakeAll();
}

void FrameLoader::processRequests() {
    m_isProcessing = true;
    for (;;) {
        QMutexLocker locker(&m_queueMutex);
        if (m_stopRequested) break;
        if (m_openPending) {
            const QString path = m_videoPath;
            const quint64 generation = m_generation;
            m_openPending = false;
            locker.unlock();
            openVideo(path, generation);
            continue;
        }
        if (m_requestQueue.isEmpty()) {
            m_waitCondition.wait(&m_queueMutex);
            continue;
        }
        auto best = std::max_element(m_requestQueue.begin(), m_requestQueue.end());
        const FrameLoadRequest request = *best;
        m_requestQueue.erase(best);
        const quint64 generation = m_generation;
        m_inFlightFrame = request.frameNumber;
        locker.unlock();
        loadFrame(request.frameNumber, generation);
        locker.relock();
        m_inFlightFrame = -1;
    }
    m_videoCapture.release();
    m_isProcessing = false;
}

void FrameLoader::loadFrame(int frameNumber, quint64 generation) {
    try {
        if (!m_videoCapture.isOpened() || frameNumber < 0) {
            emit frameLoadError(generation, frameNumber, "Video not opened or invalid frame number");
            return;
        }
        // Sequential reads avoid seeking back to a keyframe for every frame.
        if (static_cast<int>(m_videoCapture.get(cv::CAP_PROP_POS_FRAMES)) != frameNumber
            && !m_videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber))) {
            emit frameLoadError(generation, frameNumber, "Failed to seek to frame");
            return;
        }
        cv::Mat frame;
        if (!m_videoCapture.read(frame) || frame.empty()) {
            emit frameLoadError(generation, frameNumber, "Failed to read frame");
            return;
        }
        QMutexLocker locker(&m_queueMutex);
        if (generation != m_generation || m_stopRequested) return;
        if (m_frameCache) m_frameCache->insertFrame(frameNumber, frame);
        emit frameLoaded(generation, frameNumber, frame);
    } catch (const cv::Exception& ex) {
        emit frameLoadError(generation, frameNumber, QString::fromUtf8(ex.what()));
    }
}
