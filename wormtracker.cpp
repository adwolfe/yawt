#include "WormTracker.h"
#include <QDebug>

WormTracker::WormTracker(Worm* wormData, const QString& videoPath, int keyFrame,
                         const QVariantMap& trackingParams,
                         int totalFrames, double fps,
                         QObject *parent)
    : QObject(parent),
    m_wormData(wormData), // WormTracker takes ownership
    m_videoPath(videoPath),
    m_keyFrame(keyFrame),
    m_trackingParameters(trackingParams),
    m_totalFrames(totalFrames),
    m_fps(fps),
    m_forwardTrackerThread(nullptr),
    m_backwardTrackerThread(nullptr),
    m_forwardCompleted(false),
    m_backwardCompleted(false)
{
    if (m_wormData) {
        m_wormData->setParent(this); // Ensure Worm object is deleted with WormTracker
    }
}

WormTracker::~WormTracker() {
    stopTracking(); // Ensures threads are requested to stop and waited for
    // m_wormData is deleted due to QObject parent-child
    qDebug() << "WormTracker for worm ID" << (m_wormData ? m_wormData->id() : -1) << "destroyed.";
}

bool WormTracker::isTrackingCompleted() const {
    return m_forwardCompleted && m_backwardCompleted;
}

void WormTracker::startTracking() {
    if (!m_wormData) {
        emit trackingErrorForWorm(-1, "Worm data is null.");
        return;
    }
    if (m_videoPath.isEmpty()) {
        emit trackingErrorForWorm(m_wormData->id(), "Video path is empty.");
        return;
    }

    qDebug() << "WormTracker ID" << m_wormData->id() << ": Starting tracking. KeyFrame:" << m_keyFrame;

    m_forwardCompleted = false;
    m_backwardCompleted = false;
    m_trackPointsForward.clear();
    m_trackPointsBackward.clear();
    m_trackPointsBackwardReversed.clear();

    // Add the keyframe point to both tracks as the starting point
    // The centroid from m_wormData is at the keyframe
    m_trackPointsForward.append(m_wormData->initialCentroid());
    m_trackPointsBackward.append(m_wormData->initialCentroid());


    // --- Forward Tracker ---
    if (m_keyFrame < m_totalFrames -1) { // Only start if there are frames to track forward
        m_forwardTrackerThread = new TrackingThread(m_videoPath, m_wormData->id(), m_wormData->initialBoundingBox(),
                                                    m_keyFrame, TrackingDirection::Forward, m_trackingParameters,
                                                    m_totalFrames, m_fps);
        connect(m_forwardTrackerThread, &TrackingThread::frameProcessed, this, &WormTracker::onFrameProcessed);
        connect(m_forwardTrackerThread, &TrackingThread::trackingCompleted, this, &WormTracker::onTrackingThreadCompleted);
        connect(m_forwardTrackerThread, &TrackingThread::trackingError, this, &WormTracker::onTrackingThreadError);
        connect(m_forwardTrackerThread, &TrackingThread::newWormCropImage, this, &WormTracker::onNewWormCropImageFromThread);
        connect(m_forwardTrackerThread, &QThread::finished, m_forwardTrackerThread, &QObject::deleteLater); // Auto-delete thread object
        m_forwardTrackerThread->start();
    } else {
        qDebug() << "WormTracker ID" << m_wormData->id() << ": No frames to track forward from keyframe" << m_keyFrame;
        m_forwardCompleted = true; // No forward tracking needed
    }


    // --- Backward Tracker ---
    if (m_keyFrame > 0) { // Only start if there are frames to track backward
        m_backwardTrackerThread = new TrackingThread(m_videoPath, m_wormData->id(), m_wormData->initialBoundingBox(),
                                                     m_keyFrame, TrackingDirection::Backward, m_trackingParameters,
                                                     m_totalFrames, m_fps);
        connect(m_backwardTrackerThread, &TrackingThread::frameProcessed, this, &WormTracker::onFrameProcessed);
        connect(m_backwardTrackerThread, &TrackingThread::trackingCompleted, this, &WormTracker::onTrackingThreadCompleted);
        connect(m_backwardTrackerThread, &TrackingThread::trackingError, this, &WormTracker::onTrackingThreadError);
        connect(m_backwardTrackerThread, &TrackingThread::newWormCropImage, this, &WormTracker::onNewWormCropImageFromThread);
        connect(m_backwardTrackerThread, &QThread::finished, m_backwardTrackerThread, &QObject::deleteLater); // Auto-delete thread object
        m_backwardTrackerThread->start();
    } else {
        qDebug() << "WormTracker ID" << m_wormData->id() << ": No frames to track backward from keyframe" << m_keyFrame;
        m_backwardCompleted = true; // No backward tracking needed
    }

    checkAndEmitOverallCompletion(); // Check if already completed (e.g. single frame video at keyframe)
}

void WormTracker::stopTracking() {
    qDebug() << "WormTracker ID" << (m_wormData ? m_wormData->id() : -1) << ": Stopping tracking threads.";
    if (m_forwardTrackerThread && m_forwardTrackerThread->isRunning()) {
        m_forwardTrackerThread->stopTracking();
        // m_forwardTrackerThread->quit(); // Use custom stopTracking flag
        // m_forwardTrackerThread->wait(3000); // Wait for graceful exit
    }
    if (m_backwardTrackerThread && m_backwardTrackerThread->isRunning()) {
        m_backwardTrackerThread->stopTracking();
        // m_backwardTrackerThread->quit();
        // m_backwardTrackerThread->wait(3000);
    }
    // Threads will be deleted later due to deleteLater connection
    m_forwardTrackerThread = nullptr;
    m_backwardTrackerThread = nullptr;
}

void WormTracker::onFrameProcessed(int wormId, TrackingDirection direction, int frameNumber, const QPointF& newCentroid, const QRectF& newRoi) {
    if (!m_wormData || wormId != m_wormData->id()) return;

    //qDebug() << "Worm" << wormId << (direction == TrackingDirection::Forward ? "Fwd" : "Bwd")
    //         << "Frame:" << frameNumber << "Centroid:" << newCentroid;

    if (direction == TrackingDirection::Forward) {
        // Skip adding if it's the keyframe itself, as it's already added
        if (frameNumber != m_keyFrame) {
            m_trackPointsForward.append(newCentroid);
        }
    } else { // Backward
        if (frameNumber != m_keyFrame) {
            m_trackPointsBackward.append(newCentroid);
        }
    }
    emit trackUpdated(wormId, direction, frameNumber, newCentroid);
}

void WormTracker::onTrackingThreadCompleted(int wormId, TrackingDirection direction) {
    if (!m_wormData || wormId != m_wormData->id()) return;

    qDebug() << "WormTracker ID" << wormId << ":" << (direction == TrackingDirection::Forward ? "Forward" : "Backward")
             << "tracking thread completed.";

    if (direction == TrackingDirection::Forward) {
        m_forwardCompleted = true;
        if (m_forwardTrackerThread) m_forwardTrackerThread = nullptr; // It will deleteLater
    } else { // Backward
        m_backwardCompleted = true;
        // Reverse the backward track to be in chronological order
        m_trackPointsBackwardReversed.clear();
        std::reverse_copy(m_trackPointsBackward.begin(), m_trackPointsBackward.end(),
                          std::back_inserter(m_trackPointsBackwardReversed));
        if (m_backwardTrackerThread) m_backwardTrackerThread = nullptr; // It will deleteLater
    }
    checkAndEmitOverallCompletion();
}

void WormTracker::onTrackingThreadError(int wormId, TrackingDirection direction, const QString& errorMessage) {
    if (!m_wormData || wormId != m_wormData->id()) return;

    qWarning() << "WormTracker ID" << wormId << ":" << (direction == TrackingDirection::Forward ? "Forward" : "Backward")
               << "tracking thread error:" << errorMessage;

    // Treat error as completion to avoid hanging
    if (direction == TrackingDirection::Forward) {
        m_forwardCompleted = true;
        if (m_forwardTrackerThread) m_forwardTrackerThread = nullptr;
    } else {
        m_backwardCompleted = true;
        if (m_backwardTrackerThread) m_backwardTrackerThread = nullptr;
    }
    emit trackingErrorForWorm(wormId, errorMessage); // Notify main app
    checkAndEmitOverallCompletion();
}

void WormTracker::onNewWormCropImageFromThread(int wormId, TrackingDirection direction, int frameNumber, const QImage& cropImage) {
    if (!m_wormData || wormId != m_wormData->id()) return;
    // Re-emit for the VideoLoader or main UI to catch
    emit newDisplayableCrop(wormId, direction, frameNumber, cropImage);
}


void WormTracker::checkAndEmitOverallCompletion() {
    if (m_forwardCompleted && m_backwardCompleted) {
        qDebug() << "WormTracker ID" << (m_wormData ? m_wormData->id() : -99) << ": Both tracking directions completed.";
        emit trackingForWormCompleted(m_wormData ? m_wormData->id() : -99);
    }
}
