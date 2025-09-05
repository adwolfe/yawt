#ifndef FRAMELOADER_H
#define FRAMELOADER_H

#include <QObject>
#include <QString>
#include <QList>
#include <QQueue>
#include <QMutex>
#include <QWaitCondition>
#include <QDateTime>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>


/**
 * @file frameloader.h
 * @brief Declarations for FrameLoadRequest and FrameLoader.
 *
 * This header extracts the background frame loader worker used by VideoLoader.
 * It declares a simple prioritized request struct and a QObject-based worker
 * that can be moved to a QThread to process frame load requests using OpenCV.
 */
 
// Forward-declare FrameCache so the loader can accept a cache pointer without
// requiring inclusion of the full cache header here.
class FrameCache;

/// Frame loading request structure used by the background loader.
struct FrameLoadRequest {
    int frameNumber;
    int priority; // Higher number = higher priority
    QDateTime requestTime;

    FrameLoadRequest(int fn = -1, int prio = 1)
        : frameNumber(fn), priority(prio), requestTime(QDateTime::currentDateTime()) {}

    // Sorting: higher priority first; for equal priority newer requests first.
    bool operator<(const FrameLoadRequest& other) const {
        if (priority != other.priority) return priority < other.priority;
        return requestTime > other.requestTime; // Newer requests first for same priority
    }

    bool operator>(const FrameLoadRequest& other) const {
        if (priority != other.priority) return priority > other.priority;
        return requestTime < other.requestTime; // Older requests last for same priority
    }
};

/**
 * @brief Background frame loader worker.
 *
 * Designed to be instantiated and moved to a QThread. Accepts frame load
 * requests and emits signals when frames are available or when errors occur.
 */
class FrameLoader : public QObject {
    Q_OBJECT

public:
    explicit FrameLoader(QObject* parent = nullptr);
    ~FrameLoader();

    void setVideoPath(const QString& path);
    void requestFrames(const QList<int>& frameNumbers, int priority = 1);
    void requestSingleFrame(int frameNumber, int priority = 1);
    void clearRequests();

    // Provide the loader with a pointer to the shared FrameCache so it can
    // insert loaded frames directly into the cache from the worker thread.
    void setFrameCache(FrameCache* cache);

    void stop();

signals:
    /// Emitted when a frame has been successfully loaded.
    void frameLoaded(int frameNumber, cv::Mat frame);

    /// Emitted when loading a frame failed for any reason.
    void frameLoadError(int frameNumber, QString error);

public slots:
    /// Main request processing loop. Intended to be connected to QThread::started().
    void processRequests();

private:
    void loadFrame(int frameNumber);

    QString m_videoPath;
    cv::VideoCapture m_videoCapture;
    FrameCache* m_frameCache;
    QQueue<FrameLoadRequest> m_requestQueue;
    mutable QMutex m_queueMutex;
    QWaitCondition m_waitCondition;
    bool m_stopRequested;
    bool m_isProcessing;
};

#endif // FRAMELOADER_H
