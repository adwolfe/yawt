#ifndef FRAMELOADER_INTERNAL_H
#define FRAMELOADER_INTERNAL_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QWaitCondition>
#include <QMap>
#include <QList>
#include <QString>
#include <QDateTime>
#include <queue>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

// Forward declaration of FrameLoadRequest from videoloader.cpp
struct FrameLoadRequest {
    int frameNumber;
    int priority;
    QDateTime requestTime;

    FrameLoadRequest(int frame = -1, int prio = 1)
        : frameNumber(frame), priority(prio), requestTime(QDateTime::currentDateTime()) {}

    bool operator<(const FrameLoadRequest& other) const {
        return priority < other.priority;
    }

    bool operator>(const FrameLoadRequest& other) const {
        return priority > other.priority;
    }
};

class FrameLoaderWorker : public QObject {
    Q_OBJECT
public:
    FrameLoaderWorker(std::priority_queue<FrameLoadRequest, std::vector<FrameLoadRequest>, std::greater<FrameLoadRequest>>* sharedQueue,
                     QMutex* queueMutex, QWaitCondition* waitCondition, QMap<int, int>* pendingFrames);

    ~FrameLoaderWorker();

    void setVideoPath(const QString& path);

    void stop();

signals:
    void frameLoaded(int frameNumber, cv::Mat frame);
    void frameLoadError(int frameNumber, QString error);

public slots:
    void processRequests();

private:
    void loadFrame(int frameNumber);

    std::priority_queue<FrameLoadRequest, std::vector<FrameLoadRequest>, std::greater<FrameLoadRequest>>* m_sharedQueue;
    QMutex* m_queueMutex;
    QWaitCondition* m_waitCondition;
    QMap<int, int>* m_pendingFrames;
    QString m_videoPath;
    cv::VideoCapture m_videoCapture;
    bool m_stopRequested;
    bool m_isProcessing;
};

class FrameLoaderManager : public QObject {
    Q_OBJECT
public:
    explicit FrameLoaderManager(int numWorkers = 3, QObject* parent = nullptr);

    ~FrameLoaderManager();

    void setVideoPath(const QString& path);

    void requestFrames(const QList<FrameLoadRequest>& requests);

    void requestSingleFrame(int frameNumber, int priority = 50);

    void clearRequests();

    void resetPrioritiesForCurrentFrame(int centerFrame, int radius);

    void startAllLoaders();

    void stopAllLoaders();

signals:
    void frameLoaded(int frameNumber, cv::Mat frame);
    void frameLoadError(int frameNumber, QString error);

private:
    int m_numWorkers;
    QString m_videoPath;
    QList<FrameLoaderWorker*> m_workers;
    QList<QThread*> m_threads;
    std::priority_queue<FrameLoadRequest, std::vector<FrameLoadRequest>, std::greater<FrameLoadRequest>>* m_requestQueue;
    QMutex* m_queueMutex;
    QWaitCondition* m_waitCondition;
    QMap<int, int>* m_pendingFrames;
};

#endif // FRAMELOADER_INTERNAL_H