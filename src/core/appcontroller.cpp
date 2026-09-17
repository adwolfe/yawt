/**
 * @file appcontroller.cpp
 * @brief Implements AppController: non-UI orchestration for models, storage, and tracking.
 *
 * Responsibilities (implementation notes):
 *  - Construct and own TrackingDataStorage, TrackingManager, and UI-facing models.
 *  - Wire TrackingManager signals to controller slots and forward UI-friendly signals.
 *  - Provide high-level APIs used by the GUI to start/cancel tracking and modify models.
 *  - Optionally own and orchestrate the TrackingProgressDialog lifecycle.
 *
 * Typical flows:
 *  1) Direct start:
 *     - beginTrackingFromModel(...) builds InitialWormInfo from AnnotationTableModel
 *       (optionally filtering items already tracked) and invokes TrackingManager.
 *  2) Dialog-driven:
 *     - showTrackingDialog(...) creates a controller-owned dialog, connects its
 *       signals, and onDialogBeginRequested() triggers TrackingManager with the
 *       cached parameters.
 *
 * Threading:
 *  - AppController lives on the GUI thread.
 *  - TrackingManager may run workers on background threads; all cross-thread
 *    communication uses Qt signals/slots. Meta-types are registered in TrackingManager.
 *
 * Lifetime:
 *  - When AppController creates components, it parents them to itself for
 *    deterministic cleanup. If external storage is injected, ownership remains external.
 */
#include "appcontroller.h"

#include "trackingmanager.h"
#include "../data/trackingdatastorage.h"
#include "../debug/debugdatastore.h"
#include "../models/annotationtablemodel.h"
#include "../models/lostsegmenttablemodel.h"
#include "../gui/trackingprogressdialog.h"
#include <QWidget>
#include <QMessageBox>

#include "../utils/loggingcategories.h"
#include <QDebug>

AppController::AppController(QObject* parent)
    : QObject(parent)
{
    initWithNewStorage();
    connectTrackingManagerSignals();
}

AppController::AppController(TrackingDataStorage* storage, QObject* parent)
    : QObject(parent)
{
    if (storage) {
        // Adopt provided storage but do not reparent it (we assume caller manages lifetime).
        // However for simplicity, we will keep pointer and not delete it explicitly.
        m_storage = storage;
    } else {
        initWithNewStorage();
    }
    // Ensure manager/models are created if they weren't created by initWithNewStorage
    if (!m_storage) initWithNewStorage();
    if (!m_annotationModel) m_annotationModel = new AnnotationTableModel(m_storage, this);
    if (!m_lostSegmentModel) m_lostSegmentModel = new LostSegmentTableModel(m_storage, this);
    if (!m_manager) {
        if (!m_debugStore) {
            m_debugStore = new Debug::DebugDataStore();
        }
        m_manager = new TrackingManager(m_storage, m_debugStore, this);
    }
    connectTrackingManagerSignals();
}

AppController::~AppController()
{
    // QObject parent-child will delete owned children (m_manager, m_annotationModel, m_lostSegmentModel, m_storage if created with 'this' parent).
    // If m_storage was provided by caller, we do not delete it here (it may not be parented to us).
    delete m_debugStore;
    YAWT_DEBUG(lcCoreAppController) << "AppController destroyed";
}

void AppController::initWithNewStorage()
{
    // Create storage and models; parent them to this controller so Qt manages lifetime.
    if (!m_storage) {
        m_storage = new TrackingDataStorage(this);
    }
    if (!m_debugStore) {
        m_debugStore = new Debug::DebugDataStore();
    }

    // Create TrackingManager with storage; TrackingManager expects a storage pointer
    if (!m_manager) {
        m_manager = new TrackingManager(m_storage, m_debugStore, this);
    }

    // Create application models that adapt the storage to views
    if (!m_annotationModel) {
        m_annotationModel = new AnnotationTableModel(m_storage, this);
    }
    if (!m_lostSegmentModel) {
        m_lostSegmentModel = new LostSegmentTableModel(m_storage, this);
    }
}

void AppController::connectTrackingManagerSignals()
{
    if (!m_manager) return;

    // Forward overall progress and status updates
    connect(m_manager, &TrackingManager::overallTrackingProgress,
            this, &AppController::onTrackingManagerOverallProgress);
    connect(m_manager, &TrackingManager::trackingStatusUpdate,
            this, &AppController::onTrackingManagerStatusUpdate);

    // Track completion/failure/cancel
    connect(m_manager, &TrackingManager::trackingFinishedSuccessfully,
            this, &AppController::onTrackingManagerFinishedSuccessfully);
    connect(m_manager, &TrackingManager::trackingFailed,
            this, &AppController::onTrackingManagerFailed);
    connect(m_manager, &TrackingManager::trackingCancelled,
            this, &AppController::onTrackingManagerCancelled);

    connect(m_manager, &TrackingManager::centerlineProgress,
            this, &AppController::onTrackingManagerCenterlineProgress);
    connect(m_manager, &TrackingManager::centerlineFinished,
            this, &AppController::onTrackingManagerCenterlineFinished);

    // All tracks updated: handle storage and re-emit
    connect(m_manager, &TrackingManager::allTracksUpdated,
            this, &AppController::onTrackingManagerAllTracksUpdated);
}

AnnotationTableModel* AppController::annotationTableModel() const
{
    return m_annotationModel;
}

LostSegmentTableModel* AppController::lostSegmentTableModel() const
{
    return m_lostSegmentModel;
}

TrackingDataStorage* AppController::trackingDataStorage() const
{
    return m_storage;
}

Debug::DebugDataStore* AppController::debugDataStore() const
{
    return m_debugStore;
}

void AppController::addItemFromBlob(const Tracking::DetectedBlob& blob, int frame)
{
    if (!m_annotationModel) {
        YAWT_WARN(lcCoreAppController) << "addItemFromBlob: annotation model not available";
        return;
    }

    // Use the same semantics as MainWindow: add as Worm item
    bool added = m_annotationModel->addItem(blob.centroid, blob.boundingBox, frame, TableItems::ItemType::Worm);
    if (!added) {
        qWarning() << "AppController::addItemFromBlob: addItem returned false";
    } else {
        // Optionally notify consumers that storage changed (UI uses model signals, so this is not strictly necessary)
        // emit trackingStatusMessage(QString("Added worm blob at frame %1").arg(frame));
    }
}

void AppController::addRoi(const QRectF& roi, int frame)
{
    if (!m_annotationModel) {
        YAWT_WARN(lcCoreAppController) << "addRoi: annotation model not available";
        return;
    }

    bool added = m_annotationModel->addItem(QPointF(roi.x() + roi.width() / 2.0, roi.y() + roi.height() / 2.0),
                                      roi, frame, TableItems::ItemType::Region);
    if (!added) {
        qWarning() << "AppController::addRoi: addItem(ROI) returned false";
    }
}

void AppController::removeAllItems()
{
    if (!m_annotationModel) {
        YAWT_WARN(lcCoreAppController) << "removeAllItems: annotation model not available";
        return;
    }

    // Remove all rows from the model. Use model API to ensure proper notifications.
    int rowCount = m_annotationModel->rowCount();
    if (rowCount > 0) {
        m_annotationModel->removeRows(0, rowCount);
    }
}

void AppController::deleteItemById(int id)
{
    if (!m_annotationModel) {
        YAWT_WARN(lcCoreAppController) << "deleteItemById: annotation model not available";
        return;
    }

    // Find the row index matching the given id
    const QList<TableItems::AnnotationItem>& items = m_annotationModel->getAllItems();
    for (int i = 0; i < items.size(); ++i) {
        if (items[i].id == id) {
            if (!m_annotationModel->removeRows(i, 1)) {
                YAWT_WARN(lcCoreAppController) << "deleteItemById: failed to remove row" << i;
            }
            return;
        }
    }
    qWarning() << "AppController::deleteItemById: item id" << id << "not found";
}

void AppController::setRoiSizeMultiplier(double factor)
{
    if (!m_annotationModel) {
        YAWT_WARN(lcCoreAppController) << "setRoiSizeMultiplier: annotation model not available";
        return;
    }
    m_annotationModel->updateRoiSizeMultiplier(factor);
}

void AppController::setUmPerPixel(double umPerPixel)
{
    if (!m_manager) {
        YAWT_WARN(lcCoreAppController) << "setUmPerPixel: TrackingManager not available";
        return;
    }

    m_manager->setUmPerPixel(umPerPixel);
}

void AppController::requestStartTracking(const QString& videoPath,
                                        int keyFrame,
                                        const Thresholding::ThresholdSettings& settings,
                                        const std::vector<Tracking::InitialWormInfo>& initialWorms,
                                        int totalFrames,
                                        const QString& dataDirectory)
{
    if (!m_manager) {
        YAWT_WARN(lcCoreAppController) << "requestStartTracking: TrackingManager not available";
        emit trackingFailed("Internal error: TrackingManager missing");
        return;
    }

    // Emit that tracking is starting
    emit trackingStarted();

    m_manager->startFullTrackingProcess(videoPath, dataDirectory, keyFrame, initialWorms, settings, totalFrames);
}

void AppController::cancelTracking()
{
    if (!m_manager) {
        YAWT_WARN(lcCoreAppController) << "cancelTracking: TrackingManager not available";
        return;
    }
    m_manager->cancelTracking();
}

// ----- Private slots: handlers for TrackingManager signals -----

void AppController::onTrackingManagerOverallProgress(int percent)
{
    // Forward progress as both percent and (optionally) empty message.
    emit trackingProgress(percent, QString());
}

void AppController::onTrackingManagerStatusUpdate(const QString& status)
{
    // Forward the status message; UI/dialog can choose to render it
    emit trackingStatusMessage(status);
}

void AppController::onTrackingManagerAllTracksUpdated(const Tracking::AllWormTracks& tracks)
{
    if (!m_storage) {
        YAWT_WARN(lcCoreAppController) << "onTrackingManagerAllTracksUpdated: storage missing";
    } else {
        // Store tracks into central storage (merge/overwrite per item)
        for (auto it = tracks.begin(); it != tracks.end(); ++it) {
            int wormId = it->first;
            const std::vector<Tracking::WormTrackPoint>& track = it->second;
            m_storage->setTrackForWorm(wormId, track);
            YAWT_DEBUG(lcCoreAppController) << "stored track for worm" << wormId << "with" << (int)track.size() << "points";
        }
    }

    // Emit tracksUpdated with combined set from storage as a convenience
    if (m_storage) {
        const auto& allTracks = m_storage->getAllTracks();
        emit tracksUpdated(allTracks);
    } else {
        emit tracksUpdated(tracks);
    }
}

void AppController::onTrackingManagerFinishedSuccessfully(const QString& /*outputPath*/)
{
    // Forward finished signal
    emit trackingFinished();
}

void AppController::onTrackingManagerFailed(const QString& reason)
{
    emit trackingFailed(reason);
}

void AppController::onTrackingManagerCancelled()
{
    emit trackingCancelled();
}

void AppController::onTrackingManagerCenterlineProgress(int percentage)
{
    emit centerlineProgress(percentage);
}

void AppController::onTrackingManagerCenterlineFinished()
{
    emit centerlineFinished();
}

// Build vector<InitialWormInfo> from AnnotationTableModel, optionally filtering out items that already have tracks.
std::vector<Tracking::InitialWormInfo> AppController::buildInitialWormsFromModel(bool onlyTrackMissing) const
{
    std::vector<Tracking::InitialWormInfo> result;
    if (!m_annotationModel) {
        qWarning() << "AppController::buildInitialWormsFromModel: annotation model not available";
        return result;
    }

    // If requested, collect the set of item ids that already have tracks.
    QSet<int> itemsWithTracks;
    if (onlyTrackMissing && m_storage) {
        itemsWithTracks = m_storage->getWormsWithTracks();
    }

    const QList<TableItems::AnnotationItem>& items = m_annotationModel->getAllItems();
    result.reserve(items.size());
    for (const TableItems::AnnotationItem& it : items) {
        if (it.type != TableItems::ItemType::Worm) continue;
        if (onlyTrackMissing && itemsWithTracks.contains(it.id)) continue;

        Tracking::InitialWormInfo info;
        info.id = it.id;
        info.initialSearchWindow = it.initialBoundingBox;
        info.color = it.color;
        result.push_back(info);
    }

    return result;
}

bool AppController::validateAndGetSharedKeyframe(bool onlyTrackMissing, int& outKeyFrame, QString& outError) const {
    outKeyFrame = -1;
    outError.clear();

    if (!m_annotationModel) {
        outError = "Internal error: annotation model missing.";
        return false;
    }

    QSet<int> itemsWithTracks;
    if (onlyTrackMissing && m_storage)
        itemsWithTracks = m_storage->getWormsWithTracks();

    const QList<TableItems::AnnotationItem>& items = m_annotationModel->getAllItems();
    int sharedKeyframe = -2; // sentinel: no worm seen yet
    QStringList conflictDesc;

    for (const TableItems::AnnotationItem& it : items) {
        if (it.type != TableItems::ItemType::Worm) continue;
        if (onlyTrackMissing && itemsWithTracks.contains(it.id)) continue;

        if (sharedKeyframe == -2) {
            sharedKeyframe = it.frameOfSelection;
        } else if (it.frameOfSelection != sharedKeyframe) {
            conflictDesc << QString("worm %1 (frame %2)").arg(it.id).arg(it.frameOfSelection);
        }
    }

    if (sharedKeyframe == -2) {
        // No worms to track — not a keyframe error; caught later as "nothing to track".
        outKeyFrame = 0;
        return true;
    }

    if (!conflictDesc.isEmpty()) {
        outError = QString("All worms to be tracked must be selected on the same frame, "
                           "but they are not. First worm is on frame %1; conflicting: %2.")
                       .arg(sharedKeyframe)
                       .arg(conflictDesc.join(", "));
        return false;
    }

    outKeyFrame = sharedKeyframe;
    return true;
}

void AppController::beginTrackingFromModel(const QString& videoPath,
                                          const Thresholding::ThresholdSettings& settings,
                                          bool onlyTrackMissing,
                                          int totalFrames,
                                          const QString& dataDirectory)
{
    if (!m_manager) {
        YAWT_WARN(lcCoreAppController) << "beginTrackingFromModel: TrackingManager not available";
        emit trackingFailed("Internal error: TrackingManager missing");
        return;
    }
    if (!m_annotationModel) {
        YAWT_WARN(lcCoreAppController) << "beginTrackingFromModel: AnnotationTableModel not available";
        emit trackingFailed("Internal error: Annotation model missing");
        return;
    }

    int derivedKeyFrame;
    QString keyFrameError;
    if (!validateAndGetSharedKeyframe(onlyTrackMissing, derivedKeyFrame, keyFrameError)) {
        emit trackingFailed(keyFrameError);
        return;
    }

    std::vector<Tracking::InitialWormInfo> initialWorms = buildInitialWormsFromModel(onlyTrackMissing);
    if (initialWorms.empty()) {
        emit trackingFailed("No worm items available to track.");
        return;
    }

    emit trackingStarted();

    m_manager->startFullTrackingProcess(videoPath, dataDirectory, derivedKeyFrame, initialWorms, settings, totalFrames);
}

int AppController::countWormItems() const
{
    if (!m_annotationModel) return 0;
    int cnt = 0;
    const QList<TableItems::AnnotationItem>& items = m_annotationModel->getAllItems();
    for (const TableItems::AnnotationItem& it : items) {
        if (it.type == TableItems::ItemType::Worm) ++cnt;
    }
    return cnt;
}

int AppController::countWormsWithTracks() const
{
    if (!m_storage) return 0;
    return m_storage->getWormsWithTracks().size();
}

bool AppController::hasWormItems() const
{
    return countWormItems() > 0;
}

// ---- Dialog orchestration: controller-owned TrackingProgressDialog ----

void AppController::showTrackingDialog(const QString& videoPath,
                                       const Thresholding::ThresholdSettings& settings,
                                       bool onlyTrackMissing,
                                       int totalFrames,
                                       const QString& dataDirectory,
                                       QWidget* parent)
{
    // Validate that worms-to-track share a keyframe before opening the dialog.
    int derivedKeyFrame;
    QString keyFrameError;
    if (!validateAndGetSharedKeyframe(onlyTrackMissing, derivedKeyFrame, keyFrameError)) {
        QMessageBox::critical(parent, "Cannot Begin Tracking", keyFrameError);
        return;
    }

    // Store provided parameters so the dialog begin handler can use them.
    // Use the per-worm derived keyframe rather than the video's current position.
    m_dialogVideoPath = videoPath;
    m_dialogKeyFrame = derivedKeyFrame;   // the worms' shared frameOfSelection
    m_dialogSettings = settings;
    m_dialogOnlyTrackMissing = onlyTrackMissing;
    m_dialogTotalFrames = totalFrames;
    m_dialogDataDirectory = dataDirectory;

    if (!m_trackingDialog) {
        // Create dialog parented to provided widget or to nullptr (will use application's top-level)
        m_trackingDialog = new TrackingProgressDialog(parent);

        // Connect dialog requests to controller handlers
        connect(m_trackingDialog, &TrackingProgressDialog::beginTrackingRequested,
                this, &AppController::onDialogBeginRequested);
        connect(m_trackingDialog, &TrackingProgressDialog::cancelTrackingRequested,
                this, &AppController::onDialogCancelRequested);

        // Connect controller signals to update dialog UI
        connect(this, &AppController::trackingStatusMessage, m_trackingDialog, &TrackingProgressDialog::updateStatusMessage);
        connect(this, &AppController::trackingProgress, m_trackingDialog, &TrackingProgressDialog::updateOverallProgress);
        connect(this, &AppController::trackingFinished, m_trackingDialog, &TrackingProgressDialog::onTrackingSuccessfullyFinished);
        connect(this, &AppController::trackingFailed, m_trackingDialog, &TrackingProgressDialog::onTrackingFailed);
        connect(this, &AppController::trackingCancelled, m_trackingDialog, &TrackingProgressDialog::onTrackingCancelledByManager);
        connect(this, &AppController::centerlineProgress, m_trackingDialog, &TrackingProgressDialog::onCenterlineProgress);
        connect(this, &AppController::centerlineFinished, m_trackingDialog, &TrackingProgressDialog::onCenterlineFinished);
    }

    // Provide the dialog with accurate context using the parameters passed in and model/storage counts.
    int wormCount = countWormItems();
    int wormsWithTracks = countWormsWithTracks();

    m_trackingDialog->setTrackingParameters(m_dialogVideoPath, m_dialogKeyFrame, m_dialogSettings,
                                            wormCount, m_dialogTotalFrames, wormsWithTracks);

    // Execute the dialog modally. The dialog will emit begin/cancel signals which the controller handles.
    m_trackingDialog->exec();

    // Dialog closed - ensure teardown and avoid dangling pointer.
    m_trackingDialog->deleteLater();
    m_trackingDialog = nullptr;
}

void AppController::onDialogBeginRequested()
{
    // Use stored dialog parameters (set via showTrackingDialog) to start tracking.
    if (!m_manager) {
        YAWT_WARN(lcCoreAppController) << "onDialogBeginRequested: TrackingManager not available";
        emit trackingFailed("Internal error: TrackingManager missing");
        if (m_trackingDialog) m_trackingDialog->onTrackingFailed("Internal error: TrackingManager missing");
        return;
    }
    if (!m_annotationModel) {
        YAWT_WARN(lcCoreAppController) << "onDialogBeginRequested: AnnotationTableModel not available";
        emit trackingFailed("Internal error: Annotation model missing");
        if (m_trackingDialog) m_trackingDialog->onTrackingFailed("Internal error: Annotation model missing");
        return;
    }

    // Re-read onlyTrackMissing from the actual dialog checkbox state.
    if (m_trackingDialog)
        m_dialogOnlyTrackMissing = m_trackingDialog->onlyTrackMissingChecked();

    // Re-validate keyframes with the actual checkbox state (user may have toggled it).
    int derivedKeyFrame;
    QString keyFrameError;
    if (!validateAndGetSharedKeyframe(m_dialogOnlyTrackMissing, derivedKeyFrame, keyFrameError)) {
        YAWT_WARN(lcCoreAppController) << "onDialogBeginRequested: keyframe validation failed:" << keyFrameError;
        if (m_trackingDialog) m_trackingDialog->onTrackingFailed(keyFrameError);
        return;
    }
    m_dialogKeyFrame = derivedKeyFrame;

    // Build initial worm list from model, respecting the dialog's only-missing preference.
    std::vector<Tracking::InitialWormInfo> initialWorms = buildInitialWormsFromModel(m_dialogOnlyTrackMissing);
    if (initialWorms.empty()) {
        YAWT_INFO(lcCoreAppController) << "onDialogBeginRequested: no worm items to track (after filtering).";
        emit trackingFailed("No worm items available to track.");
        if (m_trackingDialog) m_trackingDialog->onTrackingFailed("No worms to track.");
        return;
    }

    // Forward the stored snake params (edited via the Debug tab) to the manager.
    // m_snakeParams holds whatever was last set via setCenterlineSnakeParams();
    // defaults from the struct are used on a fresh controller before any user edit.
    m_manager->setCenterlineSnakeParams(m_snakeParams);

    // Read centerline options from the dialog.
    if (m_trackingDialog) {
        m_manager->setCenterlineEnabled(m_trackingDialog->computeCenterlineChecked());
        m_manager->setSkipMergedFrames(m_trackingDialog->skipMergedFramesChecked());
        m_manager->setSmoothCenterline(m_trackingDialog->smoothCenterlineChecked());
    }

    // Start tracking via the manager using stored dialog parameters.
    emit trackingStarted();
    m_manager->startFullTrackingProcess(m_dialogVideoPath, m_dialogDataDirectory, m_dialogKeyFrame,
                                        initialWorms, m_dialogSettings, m_dialogTotalFrames);

    // Leave the dialog open — progress/status signals from the manager will be forwarded to it.
}

void AppController::setCenterlineSnakeParams(const Centerline::CenterlineSnakeParams& params)
{
    m_snakeParams = params;
    if (m_manager) {
        m_manager->setCenterlineSnakeParams(params);
    }
}

void AppController::setLoadedRunContext(const QString& videoPath,
                                        const QString& runDirectory,
                                        int keyFrameNum)
{
    if (!m_manager) {
        return;
    }
    m_manager->setLoadedRunContext(videoPath, runDirectory, keyFrameNum);
}

void AppController::rerunCenterline(const Centerline::CenterlineSnakeParams& params)
{
    if (!m_manager) {
        YAWT_WARN(lcCoreAppController) << "rerunCenterline: no TrackingManager available";
        return;
    }
    m_snakeParams = params;
    m_manager->setCenterlineSnakeParams(params);
    m_manager->startCenterlineComputation();
}

void AppController::onDialogCancelRequested()
{
    // Forward cancellation to the manager
    cancelTracking();

    // Update dialog UI if present
    if (m_trackingDialog) {
        m_trackingDialog->onTrackingCancelledByManager();
        // Close the dialog (will cause exec() to return)
        m_trackingDialog->reject();
    }
}
