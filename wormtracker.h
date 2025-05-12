#ifndef WORMTRACKER_H
#define WORMTRACKER_H

#include <QObject>
#include <QList> // To store track points
#include <QPointF>
#include <QRectF>
#include <QVariantMap>

#include "worm.h" // Definition of Worm class
#include "trackingthread.h" // Definition of TrackingThread

class VideoLoader; // Forward declaration

/**
 * @brief Manages the tracking process for a single worm, including its
 * forward and backward tracking threads.
 */
class WormTracker : public QObject {
    Q_OBJECT
public:
    explicit WormTracker(Worm* wormData, const QString& videoPath, int keyFrame,
                         const QVariantMap& trackingParams, // Pass threshold params etc.
                         int totalFrames, double fps,      // Pass video properties
                         QObject *parent = nullptr);
    ~WormTracker();

    Worm* getWormData() const { return m_wormData; }
    const QList<QPointF>& getTrackForward() const { return m_trackPointsForward; }
    const QList<QPointF>& getTrackBackward() const { return m_trackPointsBackwardReversed; }
    bool isTrackingCompleted() const;


public slots:
    void startTracking();
    void stopTracking(); // Stops both threads

private slots:
    void onFrameProcessed(int wormId, TrackingDirection direction, int frameNumber, const QPointF& newCentroid, const QRectF& newRoi);
    void onTrackingThreadCompleted(int wormId, TrackingDirection direction);
    void onTrackingThreadError(int wormId, TrackingDirection direction, const QString& errorMessage);
    void onNewWormCropImageFromThread(int wormId, TrackingDirection direction, int frameNumber, const QImage& cropImage);


signals:
    // Signals to notify the main application or UI
    void trackUpdated(int wormId, TrackingDirection direction, int frameNumber, const QPointF& centroid); // When a new point is added to either track
    void trackingForWormCompleted(int wormId); // Both directions for this worm are done
    void trackingErrorForWorm(int wormId, const QString& message);
    void newDisplayableCrop(int wormId, TrackingDirection direction, int frameNum, const QImage& cropImg);


private:
    Worm* m_wormData; // WormTracker takes ownership of wormData
    QString m_videoPath;
    int m_keyFrame;
    QVariantMap m_trackingParameters;
    int m_totalFrames;
    double m_fps;

    TrackingThread* m_forwardTrackerThread;
    TrackingThread* m_backwardTrackerThread;

    QList<QPointF> m_trackPointsForward;    // Stores points from keyFrame onwards
    QList<QPointF> m_trackPointsBackward;   // Stores points from keyFrame backwards (in processing order)
    QList<QPointF> m_trackPointsBackwardReversed; // Backward points, reversed for chronological display

    bool m_forwardCompleted;
    bool m_backwardCompleted;

    void checkAndEmitOverallCompletion();
};

#endif // WORMTRACKER_H
