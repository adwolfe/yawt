#ifndef APPCONTROLLER_H
#define APPCONTROLLER_H

#include <QObject>
#include <QString>
#include <QRectF>
#include <vector>

#include "../data/trackingcommon.h" // for Tracking::DetectedBlob, Tracking::AllWormTracks, Thresholding::ThresholdSettings, InitialWormInfo

// Forward declarations of types owned/used by AppController.
// Keep GUI-only headers out of this file.
class TrackingManager;
class TrackingDataStorage;
class BlobTableModel;
class AnnotationTableModel;

/**
 * AppController
 *
 * Non-GUI application controller responsible for owning and coordinating
 * core application components that are not part of the view layer.
 *
 * Responsibilities (summary):
 *  - Create and own TrackingDataStorage, TrackingManager, and application models
 *    such as BlobTableModel and AnnotationTableModel.
 *  - Expose models to the GUI (MainWindow) so views can bind to them.
 *  - Provide high-level operations to manipulate models and to start/cancel tracking.
 *  - Translate TrackingManager signals into controller-level signals suitable for UI.
 *
 * Design notes:
 *  - MainWindow keeps ownership of UI widgets (dialogs, VideoLoader, MiniLoader, etc).
 *  - AppController is a QObject and should be parented by MainWindow (or another long-lived QObject).
 *  - AppController connects to TrackingManager and forwards progress/finished/failed events via signals.
 */
class AppController : public QObject
{
    Q_OBJECT

public:
    explicit AppController(QObject* parent = nullptr);
    // Alternate constructor: use an existing TrackingDataStorage instance (optional)
    explicit AppController(TrackingDataStorage* storage, QObject* parent = nullptr);
    ~AppController() override;

    // Accessors for UI to bind models
    BlobTableModel* blobTableModel() const;
    AnnotationTableModel* annotationTableModel() const;
    TrackingDataStorage* trackingDataStorage() const;

    // Model-manipulation commands (can be invoked from UI)
    Q_INVOKABLE void addBlobFromVideo(const Tracking::DetectedBlob& blob, int frame);
    Q_INVOKABLE void addRoi(const QRectF& roi, int frame);
    Q_INVOKABLE void removeAllBlobs();
    Q_INVOKABLE void deleteBlobById(int id);
    Q_INVOKABLE void setRoiSizeMultiplier(double factor);

    // Tracking control API (high-level)
    // Note: Thresholding::ThresholdSettings and Tracking::InitialWormInfo are defined in trackingcommon.h
    Q_INVOKABLE void requestStartTracking(const QString& videoPath,
                                          int keyFrame,
                                          const Thresholding::ThresholdSettings& settings,
                                          const std::vector<Tracking::InitialWormInfo>& initialWorms,
                                          int totalFrames);
    Q_INVOKABLE void cancelTracking();

signals:
    // Emitted when tracks are available/updated (after they are stored in storage)
    void tracksUpdated(const Tracking::AllWormTracks& tracks);

    // Progress and status signals for UI/dialogs
    void trackingStarted();
    void trackingProgress(int overallPercent, const QString& statusMessage);
    void trackingFinished();
    void trackingFailed(const QString& reason);
    void trackingCancelled();

    // Generic status message (optional)
    void trackingStatusMessage(const QString& message);

private slots:
    // Internal slots to receive TrackingManager events and re-emit as controller signals
    void onTrackingManagerOverallProgress(int percent);
    void onTrackingManagerStatusUpdate(const QString& status);
    void onTrackingManagerAllTracksUpdated(const Tracking::AllWormTracks& tracks);
    void onTrackingManagerFinishedSuccessfully(const QString& outputPath);
    void onTrackingManagerFailed(const QString& reason);
    void onTrackingManagerCancelled();

private:
    // Helper to initialize owned components
    void initWithNewStorage();
    void connectTrackingManagerSignals();

    // Owned components (lifetime managed by QObject parent/child)
    TrackingDataStorage* m_storage = nullptr;
    TrackingManager* m_manager = nullptr;
    BlobTableModel* m_blobModel = nullptr;
    AnnotationTableModel* m_annotationModel = nullptr;
};

#endif // APPCONTROLLER_H