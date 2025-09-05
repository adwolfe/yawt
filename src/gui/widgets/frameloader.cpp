#include "frameloader.h"
#include "videoloader.h"

#include <QDebug>
#include <QMutexLocker>
#include <QThread>
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

void FrameLoader::setVideoPath(const QString& path) {
    QMutexLocker locker(&m_queueMutex);

    if (m_videoCapture.isOpened()) {
        m_videoCapture.release();
    }

    m_videoPath = path;
    if (!path.isEmpty()) {
        if (!m_videoCapture.open(path.toStdString())) {
            qWarning() << "FrameLoader: Failed to open video:" << path;
            return;
        }
        qDebug() << "FrameLoader: Video opened successfully:" << path;
    }
}

void FrameLoader::setFrameCache(FrameCache* cache) {
    // Store a pointer to the shared FrameCache so the loader thread can
    // insert loaded frames directly into the cache without involving the UI thread.
    QMutexLocker locker(&m_queueMutex);
    m_frameCache = cache;
    qDebug() << "FrameLoader: frame cache set:" << (m_frameCache != nullptr);
}

void FrameLoader::requestFrames(const QList<int>& frameNumbers, int priority) {
    QMutexLocker locker(&m_queueMutex);

    bool anyAdded = false;
    for (int frameNumber : frameNumbers) {
        // Skip invalid frame numbers
        if (frameNumber < 0) continue;

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

    if (frameNumber < 0) {
        return;
    }

    // If frame already cached, don't enqueue
    if (m_frameCache && m_frameCache->hasFrame(frameNumber)) {
        return;
    }

    // Avoid duplicate requests
    for (const FrameLoadRequest& req : m_requestQueue) {
        if (req.frameNumber == frameNumber) {
            return;
        }
    }

    m_requestQueue.enqueue(FrameLoadRequest(frameNumber, priority));
    locker.unlock();
    m_waitCondition.wakeOne();
}

void FrameLoader::clearRequests() {
    QMutexLocker locker(&m_queueMutex);
    m_requestQueue.clear();
    qDebug() << "FrameLoader: Cleared all pending requests";
}

void FrameLoader::stop() {
    QMutexLocker locker(&m_queueMutex);
    m_stopRequested = true;
    locker.unlock();
    m_waitCondition.wakeAll();
}

void FrameLoader::processRequests() {
    qDebug() << "FrameLoader: Started processing requests";
    m_isProcessing = true;

    while (!m_stopRequested) {
        QMutexLocker locker(&m_queueMutex);

        if (m_requestQueue.isEmpty()) {
            m_waitCondition.wait(&m_queueMutex, 1000); // Wait up to 1 second
            continue;
        }

        // Sort queue by priority
        QList<FrameLoadRequest> requests;
        while (!m_requestQueue.isEmpty()) {
            requests.append(m_requestQueue.dequeue());
        }
        std::sort(requests.begin(), requests.end(), std::greater<FrameLoadRequest>());

        // Process highest priority request
        FrameLoadRequest request = requests.first();
        requests.removeFirst();

        // Put remaining requests back
        for (const auto& req : requests) {
            m_requestQueue.enqueue(req);
        }

        locker.unlock();

        // Load the frame (outside of lock)
        loadFrame(request.frameNumber);

        // Small delay to prevent overwhelming the system
        QThread::msleep(5);
    }

    m_isProcessing = false;
    qDebug() << "FrameLoader: Stopped processing requests";
}

void FrameLoader::loadFrame(int frameNumber) {
    if (!m_videoCapture.isOpened() || frameNumber < 0) {
        emit frameLoadError(frameNumber, "Video not opened or invalid frame number");
        return;
    }

    try {
        if (!m_videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber))) {
            emit frameLoadError(frameNumber, "Failed to seek to frame");
            return;
        }

        cv::Mat frame;
        if (m_videoCapture.read(frame) && !frame.empty()) {
            // If a shared frame cache has been provided, insert the newly loaded
            // frame from this worker thread. FrameCache::insertFrame is thread-safe.
            if (m_frameCache) {
                m_frameCache->insertFrame(frameNumber, frame);
            }

            emit frameLoaded(frameNumber, frame);
        } else {
            emit frameLoadError(frameNumber, "Failed to read frame");
        }
    } catch (const cv::Exception& ex) {
        emit frameLoadError(frameNumber, QString("OpenCV exception: %1").arg(ex.what()));
    }
}