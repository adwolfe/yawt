#include "frameloader_internal.h"
#include <QDebug>
#include <QMutexLocker>
#include "thresholdingutils.h"

// Get the preload radius from videoloader.cpp static variables
extern int s_preloadRadius;

FrameLoaderWorker::FrameLoaderWorker(std::priority_queue<FrameLoadRequest, std::vector<FrameLoadRequest>, std::greater<FrameLoadRequest>>* sharedQueue,
                 QMutex* queueMutex, QWaitCondition* waitCondition, QMap<int, int>* pendingFrames)
    : m_sharedQueue(sharedQueue), m_queueMutex(queueMutex), m_waitCondition(waitCondition),
      m_pendingFrames(pendingFrames), m_stopRequested(false), m_isProcessing(false) {}

FrameLoaderWorker::~FrameLoaderWorker() {
    if (m_videoCapture.isOpened()) {
        m_videoCapture.release();
    }
}

void FrameLoaderWorker::loadFrame(const FrameLoadRequest& request) {
    int frameNumber = request.frameNumber;

    if (!m_videoCapture.isOpened() && !m_videoPath.isEmpty()) {
        m_videoCapture.open(m_videoPath.toStdString());
    }

    if (!m_videoCapture.isOpened()) {
        emit frameLoadError(frameNumber, "Failed to open video file");
        return;
    }

    try {
        if (!m_videoCapture.set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(frameNumber))) {
            emit frameLoadError(frameNumber, "Failed to seek to frame");
            return;
        }
        cv::Mat frame;
        if (m_videoCapture.read(frame) && !frame.empty()) {
            // Pre-process thresholded version using the snapshot of settings from the request
            cv::Mat thresholded;
            try {
                ThresholdingUtils::applyThresholding(frame, thresholded, request.thresholdSettings);
            } catch (const cv::Exception& ex) {
                qWarning() << "FrameLoaderWorker: thresholding exception for frame" << frameNumber << ":" << ex.what();
                thresholded = cv::Mat();
            }
            emit frameLoaded(frameNumber, frame, thresholded);
        } else {
            emit frameLoadError(frameNumber, QString("Failed to read frame %1").arg(frameNumber));
        }
    } catch (const cv::Exception& ex) {
        emit frameLoadError(frameNumber, QString("OpenCV exception: %1").arg(ex.what()));
    }
}

void FrameLoaderWorker::processRequests() {
    m_isProcessing = true;
    while (!m_stopRequested) {
        QMutexLocker locker(m_queueMutex);
        if (m_sharedQueue->empty()) {
            m_waitCondition->wait(m_queueMutex, 1000);
            continue;
        }
        FrameLoadRequest request = m_sharedQueue->top();
        m_sharedQueue->pop();

        // Skip if frame was already processed by another worker
        if (!m_pendingFrames->contains(request.frameNumber)) {
            continue;
        }

        m_pendingFrames->remove(request.frameNumber);
        locker.unlock();
        loadFrame(request);
    }
    m_isProcessing = false;
}

void FrameLoaderWorker::setVideoPath(const QString& path) {
    m_videoPath = path;
    if (m_videoCapture.isOpened()) {
        m_videoCapture.release();
    }
    if (!path.isEmpty()) {
        m_videoCapture.open(path.toStdString());
    }
}

void FrameLoaderWorker::stop() {
    QMutexLocker locker(m_queueMutex);
    m_stopRequested = true;
    m_waitCondition->wakeAll();
}

FrameLoaderManager::FrameLoaderManager(int numWorkers, QObject* parent)
    : QObject(parent), m_numWorkers(numWorkers) {
    m_queueMutex = new QMutex();
    m_waitCondition = new QWaitCondition();
    m_pendingFrames = new QMap<int, int>();
    m_requestQueue = new std::priority_queue<FrameLoadRequest, std::vector<FrameLoadRequest>, std::greater<FrameLoadRequest>>();
}

FrameLoaderManager::~FrameLoaderManager() {
    stopAllLoaders();
    delete m_queueMutex;
    delete m_waitCondition;
    delete m_pendingFrames;
    delete m_requestQueue;
}

void FrameLoaderManager::setVideoPath(const QString& path) {
    m_videoPath = path;
    for (auto* worker : m_workers) {
        worker->setVideoPath(path);
    }
}

void FrameLoaderManager::requestFrames(const QList<FrameLoadRequest>& requests) {
    if (requests.isEmpty()) return;

    QMutexLocker locker(m_queueMutex);

    // Attempt to capture threshold settings from a high-priority "center" request, if one is provided.
    // Many callers will create a single high-priority request for the center frame and lower-priority
    // requests for neighbors. If the center request carries an explicit ThresholdSettings snapshot,
    // propagate that snapshot to other requests that still contain default settings. This allows the
    // background workers to precompute thresholded frames consistently for the neighborhood without
    // requiring callers to populate settings for every request.
    Thresholding::ThresholdSettings defaultSettings; // default-initialized sentinel
    bool haveCenterSettings = false;
    Thresholding::ThresholdSettings centerSettings;

    for (const auto& r : requests) {
        if (r.priority >= 100) { // treat high-priority requests as potential "center" frames
            const Thresholding::ThresholdSettings& ts = r.thresholdSettings;
            // Compare against defaults to see if this request actually contains non-default settings.
            if (ts.algorithm != defaultSettings.algorithm ||
                ts.globalThresholdValue != defaultSettings.globalThresholdValue ||
                ts.adaptiveBlockSize != defaultSettings.adaptiveBlockSize ||
                ts.adaptiveCValue != defaultSettings.adaptiveCValue ||
                ts.assumeLightBackground != defaultSettings.assumeLightBackground ||
                ts.enableBlur != defaultSettings.enableBlur ||
                ts.blurKernelSize != defaultSettings.blurKernelSize ||
                ts.blurSigmaX != defaultSettings.blurSigmaX) {
                centerSettings = ts;
                haveCenterSettings = true;
                break;
            }
        }
    }

    for (auto request : requests) {
        if (!m_pendingFrames->contains(request.frameNumber)) {
            // If we discovered a center/settings snapshot, and this request still has default settings,
            // adopt the center's settings so background workers can produce consistent thresholded frames.
            if (haveCenterSettings) {
                const Thresholding::ThresholdSettings& ts = request.thresholdSettings;
                if (ts.algorithm == defaultSettings.algorithm &&
                    ts.globalThresholdValue == defaultSettings.globalThresholdValue &&
                    ts.adaptiveBlockSize == defaultSettings.adaptiveBlockSize &&
                    ts.adaptiveCValue == defaultSettings.adaptiveCValue &&
                    ts.assumeLightBackground == defaultSettings.assumeLightBackground &&
                    ts.enableBlur == defaultSettings.enableBlur &&
                    ts.blurKernelSize == defaultSettings.blurKernelSize &&
                    ts.blurSigmaX == defaultSettings.blurSigmaX) {
                    request.thresholdSettings = centerSettings;
                }
            }

            m_requestQueue->push(request);
            m_pendingFrames->insert(request.frameNumber, request.priority);
        }
    }
    m_waitCondition->wakeAll();
}

void FrameLoaderManager::requestSingleFrame(int frameNumber, int priority) {
    QList<FrameLoadRequest> requests;
    requests.append(FrameLoadRequest(frameNumber, priority));
    requestFrames(requests);
}

void FrameLoaderManager::clearRequests() {
    QMutexLocker locker(m_queueMutex);
    while (!m_requestQueue->empty()) {
        m_requestQueue->pop();
    }
    m_pendingFrames->clear();
    m_waitCondition->wakeAll();
}

void FrameLoaderManager::resetPrioritiesForCurrentFrame(int centerFrame, int radius) {
    QMutexLocker locker(m_queueMutex);

    // Clear existing queue and rebuild with updated priorities using provided radius
    std::priority_queue<FrameLoadRequest, std::vector<FrameLoadRequest>, std::greater<FrameLoadRequest>> newQueue;

    while (!m_requestQueue->empty()) {
        FrameLoadRequest request = m_requestQueue->top();
        m_requestQueue->pop();

        // Update priority based on distance from center frame using caller-provided radius
        int distance = qAbs(request.frameNumber - centerFrame);
        int newPriority = (distance <= radius) ? 100 : 1;
        request.priority = newPriority;

        newQueue.push(request);
    }

    *m_requestQueue = newQueue;
    m_waitCondition->wakeAll();
}

void FrameLoaderManager::startAllLoaders() {
    for (int i = 0; i < m_numWorkers; ++i) {
        FrameLoaderWorker* worker = new FrameLoaderWorker(m_requestQueue, m_queueMutex, m_waitCondition, m_pendingFrames);
        QThread* thread = new QThread(this);

        worker->moveToThread(thread);
        worker->setVideoPath(m_videoPath);

        connect(thread, &QThread::started, worker, &FrameLoaderWorker::processRequests);
        // Explicitly bind the new signature (original + thresholded cv::Mat) to avoid ambiguity
        connect(worker,
                static_cast<void (FrameLoaderWorker::*)(int, cv::Mat, cv::Mat)>(&FrameLoaderWorker::frameLoaded),
                this,
                static_cast<void (FrameLoaderManager::*)(int, cv::Mat, cv::Mat)>(&FrameLoaderManager::frameLoaded));
        connect(worker, &FrameLoaderWorker::frameLoadError, this, &FrameLoaderManager::frameLoadError);
        connect(thread, &QThread::finished, worker, &QObject::deleteLater);

        m_workers.append(worker);
        m_threads.append(thread);
        thread->start();
    }
}

void FrameLoaderManager::stopAllLoaders() {
    for (auto* worker : m_workers) {
        worker->stop();
    }

    for (auto* thread : m_threads) {
        if (thread->isRunning()) {
            thread->quit();
            if (!thread->wait(3000)) {
                qWarning() << "FrameLoaderManager: Worker thread failed to stop gracefully, terminating";
                thread->terminate();
                thread->wait(1000);
            }
        }
    }

    m_workers.clear();
    m_threads.clear();
}
