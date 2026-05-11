/**
 * @file appcontroller.h
 * @brief Non-UI application controller that owns storage, models, and TrackingManager; exposes high-level APIs and forwards signals for UI consumption.
 *
 * Responsibilities:
 *  - Create and own TrackingDataStorage, TrackingManager, BlobTableModel, and AnnotationTableModel.
 *  - Expose models to the GUI so views can bind to them.
 *  - Provide high-level operations to manipulate models and to start/cancel tracking (e.g., beginTrackingFromModel, requestStartTracking, showTrackingDialog, cancelTracking).
 *  - Translate TrackingManager signals into controller-level signals suitable for UI.
 *
 * Threading/Lifetime:
 *  - QObject living on the GUI thread; parents storage/models/manager by default for deterministic lifetime.
 *  - Connects to TrackingManager via Qt signals/slots (queued where cross-thread). Ensure meta-types are registered (handled in TrackingManager).
 */
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
class TrackingProgressDialog;
class QWidget;

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
/**
 * @class AppController
 * @brief Non-UI controller that owns storage, models, and TrackingManager; exposes high-level APIs and forwards progress/status signals to the UI.
 *
 * Creates and owns TrackingDataStorage, TrackingManager, BlobTableModel, and AnnotationTableModel.
 * Provides high-level operations (beginTrackingFromModel, requestStartTracking, showTrackingDialog, cancelTracking)
 * and forwards TrackingManager signals as controller-level signals for UI consumption.
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
    Q_INVOKABLE void setPixelSizePixelsPerUm(double value);

    // Tracking control API (high-level)
    // Note: Thresholding::ThresholdSettings and Tracking::InitialWormInfo are defined in trackingcommon.h
    /**
     * @brief Start a full tracking run using explicitly provided initial worms.
     * @param videoPath Absolute path to the source video.
     * @param keyFrame Frame index where all worms are visible.
     * @param settings Thresholding parameters captured from the UI.
     * @param initialWorms Vector of initial worm descriptors (ID, ROI, color).
     * @param totalFrames Total number of frames in the video (hint for progress and bounds).
     */
    Q_INVOKABLE void requestStartTracking(const QString& videoPath,
                                          int keyFrame,
                                          const Thresholding::ThresholdSettings& settings,
                                          const std::vector<Tracking::InitialWormInfo>& initialWorms,
                                          int totalFrames,
                                          const QString& dataDirectory);
    Q_INVOKABLE void cancelTracking();

    // Higher-level orchestration: start tracking using the current BlobTableModel contents.
    // If onlyTrackMissing is true, any items that already have tracks in storage will be skipped.
    // This lets the UI hand a lightweight request to the controller without building initialWorms itself.
    Q_INVOKABLE void beginTrackingFromModel(const QString& videoPath,
                                            int keyFrame,
                                            const Thresholding::ThresholdSettings& settings,
                                            bool onlyTrackMissing,
                                            int totalFrames,
                                            const QString& dataDirectory);

    // Helper queries for UI
    // - Returns the number of items currently marked as worms in the blob model
    Q_INVOKABLE int countWormItems() const;
    // - Returns the number of distinct items that already have tracks stored
    Q_INVOKABLE int countItemsWithTracks() const;
    // - Convenience: whether there are any worm items available to track
    Q_INVOKABLE bool hasWormItems() const;
    // UI helper: create and show the tracking progress dialog parented to 'parent' (optional).
    // The controller will create the dialog, connect to its signals and forward dialog requests to the manager.
    //
    // This variant accepts the common set of video/tracking parameters so the controller can populate
    // the dialog with accurate context (video file name, keyframe, threshold settings, whether only-missing
    // should be tracked, and total frames). If the caller supplies these, the controller will store them
    // and use them when starting tracking from the dialog flow.
    /**
     * @brief Create, wire, and execute the controller-owned tracking dialog.
     * @param videoPath Absolute path to the source video.
     * @param keyFrame Frame index used as the keyframe.
     * @param settings Thresholding parameters snapshot for this session.
     * @param onlyTrackMissing When true, initial worms are built by skipping items already tracked in storage.
     * @param totalFrames Total number of frames in the video (for progress UI and bounds).
     * @param parent Optional parent widget for the dialog.
     *
     * The dialog emits begin/cancel requests; this controller responds by starting or cancelling tracking via TrackingManager.
     */
    Q_INVOKABLE void showTrackingDialog(const QString& videoPath,
                                        int keyFrame,
                                        const Thresholding::ThresholdSettings& settings,
                                        bool onlyTrackMissing,
                                        int totalFrames,
                                        const QString& dataDirectory,
                                        QWidget* parent = nullptr);

    /**
     * @brief Update the active-contour parameters used for ring/coiled centerline refinement.
     *
     * These params are stored persistently on the controller and used both:
     *   (a) when a new tracking run is started via the dialog, and
     *   (b) when an on-demand rerun is triggered from the Debug tab.
     *
     * Safe to call at any time (not just before tracking).
     */
    void setCenterlineSnakeParams(const Tracking::CenterlineSnakeParams& params);

    /**
     * @brief Re-run the post-tracking centerline computation with the given params.
     *
     * Stores the params (equivalent to calling setCenterlineSnakeParams first) and
     * immediately launches a background CenterlineWorker pass over whatever blobs are
     * currently in storage. Progress and completion are emitted via centerlineProgress /
     * centerlineFinished — connect those signals in the caller to update the UI.
     *
     * No-op when no tracking data is in storage or when a centerline pass is already running.
     */
    Q_INVOKABLE void rerunCenterline(const Tracking::CenterlineSnakeParams& params);

signals:
    /**
     * @brief Emitted after tracks are persisted in TrackingDataStorage.
     * UI can refresh overlays, tables, and mini-loaders from the single source of truth.
     */
    void tracksUpdated(const Tracking::AllWormTracks& tracks);

    /**
     * @name Tracking lifecycle and progress
     * @{
     */
    /** Emitted when a tracking run is initiated by the controller. */
    void trackingStarted();
    /**
     * @brief Overall percent progress and optional status message for UI/dialogs.
     * @param overallPercent 0–100 aggregate progress across video processing and trackers.
     * @param statusMessage Optional human-readable status (e.g., chunk info, saving progress).
     */
    void trackingProgress(int overallPercent, const QString& statusMessage);
    /** Emitted when tracking finishes successfully (after finalization). */
    void trackingFinished();
    /** Emitted when tracking fails irrecoverably. Reason is human-readable. */
    void trackingFailed(const QString& reason);
    /** Emitted when a user- or system-requested cancellation completes. */
    void trackingCancelled();
    /** @} */

    /** Percent progress (0–100) for the post-tracking centerline computation phase. */
    void centerlineProgress(int percentage);
    /** Emitted when post-tracking centerline computation completes. */
    void centerlineFinished();

    /** Generic status message stream for detailed updates (can be verbose). */
    void trackingStatusMessage(const QString& message);

private slots:
    // Internal slots to receive TrackingManager events and re-emit as controller signals
    void onTrackingManagerOverallProgress(int percent);
    void onTrackingManagerStatusUpdate(const QString& status);
    void onTrackingManagerAllTracksUpdated(const Tracking::AllWormTracks& tracks);
    void onTrackingManagerFinishedSuccessfully(const QString& outputPath);
    void onTrackingManagerFailed(const QString& reason);
    void onTrackingManagerCancelled();
    void onTrackingManagerCenterlineProgress(int percentage);
    void onTrackingManagerCenterlineFinished();

    // Slots to receive dialog requests (when controller owns the dialog)
    void onDialogBeginRequested();
    void onDialogCancelRequested();

private:
    /**
     * @brief Initialize controller-owned storage, manager, and models.
     * Parents all created objects to this controller for deterministic lifetime.
     */
    void initWithNewStorage();
    /**
     * @brief Connect TrackingManager signals to controller handlers and forwarding signals.
     * Safe to call multiple times; guarded against a null manager.
     */
    void connectTrackingManagerSignals();

    /**
     * @brief Build initial worm list from the blob model.
     * @param onlyTrackMissing When true, skip items that already have tracks in storage.
     * @return Vector of InitialWormInfo constructed from current BlobTableModel contents.
     *
     * Thread affinity: GUI thread (reads model/storage).
     */
    std::vector<Tracking::InitialWormInfo> buildInitialWormsFromModel(bool onlyTrackMissing) const;

    // Owned components (QObject-parented to this controller unless storage is injected)
    // - If constructed by AppController, these are children of this and auto-destroyed.
    // - If an external TrackingDataStorage is injected via ctor, m_storage is non-owned.
    TrackingDataStorage* m_storage = nullptr;         // owned unless injected
    TrackingManager* m_manager = nullptr;             // owned
    BlobTableModel* m_blobModel = nullptr;            // owned
    AnnotationTableModel* m_annotationModel = nullptr;// owned

    // Dialog parameters cached for controller-owned dialog orchestration (UI thread only)
    QString m_dialogVideoPath;
    int m_dialogKeyFrame = -1;
    Thresholding::ThresholdSettings m_dialogSettings;
    bool m_dialogOnlyTrackMissing = true;
    int m_dialogTotalFrames = 0;
    QString m_dialogDataDirectory;

    // Active-contour params stored persistently. Updated by the Debug tab via
    // setCenterlineSnakeParams(); used for both new tracking runs and on-demand reruns.
    Tracking::CenterlineSnakeParams m_snakeParams;

    // Controller-owned tracking progress dialog (created on demand, parented to provided 'parent' widget)
    TrackingProgressDialog* m_trackingDialog = nullptr;
};

#endif // APPCONTROLLER_H
