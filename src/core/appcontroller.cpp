#include "appcontroller.h"

#include "trackingmanager.h"
#include "../data/trackingdatastorage.h"
#include "../models/blobtablemodel.h"
#include "../models/annotationtablemodel.h"

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
    if (!m_blobModel) m_blobModel = new BlobTableModel(m_storage, this);
    if (!m_annotationModel) m_annotationModel = new AnnotationTableModel(m_storage, this);
    if (!m_manager) {
        m_manager = new TrackingManager(m_storage, this);
    }
    connectTrackingManagerSignals();
}

AppController::~AppController()
{
    // QObject parent-child will delete owned children (m_manager, m_blobModel, m_annotationModel, m_storage if created with 'this' parent).
    // If m_storage was provided by caller, we do not delete it here (it may not be parented to us).
    qDebug() << "AppController: destroyed";
}

void AppController::initWithNewStorage()
{
    // Create storage and models; parent them to this controller so Qt manages lifetime.
    if (!m_storage) {
        m_storage = new TrackingDataStorage(this);
    }

    // Create TrackingManager with storage; TrackingManager expects a storage pointer
    if (!m_manager) {
        m_manager = new TrackingManager(m_storage, this);
    }

    // Create application models that adapt the storage to views
    if (!m_blobModel) {
        m_blobModel = new BlobTableModel(m_storage, this);
    }
    if (!m_annotationModel) {
        m_annotationModel = new AnnotationTableModel(m_storage, this);
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

    // All tracks updated: handle storage and re-emit
    connect(m_manager, &TrackingManager::allTracksUpdated,
            this, &AppController::onTrackingManagerAllTracksUpdated);
}

BlobTableModel* AppController::blobTableModel() const
{
    return m_blobModel;
}

AnnotationTableModel* AppController::annotationTableModel() const
{
    return m_annotationModel;
}

TrackingDataStorage* AppController::trackingDataStorage() const
{
    return m_storage;
}

void AppController::addBlobFromVideo(const Tracking::DetectedBlob& blob, int frame)
{
    if (!m_blobModel) {
        qWarning() << "AppController::addBlobFromVideo: blob model not available";
        return;
    }

    // Use the same semantics as MainWindow: add as Worm item
    bool added = m_blobModel->addItem(blob.centroid, blob.boundingBox, frame, TableItems::ItemType::Worm);
    if (!added) {
        qWarning() << "AppController::addBlobFromVideo: addItem returned false";
    } else {
        // Optionally notify consumers that storage changed (UI uses model signals, so this is not strictly necessary)
        // emit trackingStatusMessage(QString("Added worm blob at frame %1").arg(frame));
    }
}

void AppController::addRoi(const QRectF& roi, int frame)
{
    if (!m_blobModel) {
        qWarning() << "AppController::addRoi: blob model not available";
        return;
    }

    bool added = m_blobModel->addItem(QPointF(roi.x() + roi.width() / 2.0, roi.y() + roi.height() / 2.0),
                                      roi, frame, TableItems::ItemType::ROI);
    if (!added) {
        qWarning() << "AppController::addRoi: addItem(ROI) returned false";
    }
}

void AppController::removeAllBlobs()
{
    if (!m_blobModel) {
        qWarning() << "AppController::removeAllBlobs: blob model not available";
        return;
    }

    // Remove all rows from the model. Use model API to ensure proper notifications.
    int rowCount = m_blobModel->rowCount();
    if (rowCount > 0) {
        m_blobModel->removeRows(0, rowCount);
    }
}

void AppController::deleteBlobById(int id)
{
    if (!m_blobModel) {
        qWarning() << "AppController::deleteBlobById: blob model not available";
        return;
    }

    // Find the row index matching the given id
    const QList<TableItems::ClickedItem>& items = m_blobModel->getAllItems();
    for (int i = 0; i < items.size(); ++i) {
        if (items[i].id == id) {
            if (!m_blobModel->removeRows(i, 1)) {
                qWarning() << "AppController::deleteBlobById: failed to remove row" << i;
            }
            return;
        }
    }
    qWarning() << "AppController::deleteBlobById: item id" << id << "not found";
}

void AppController::setRoiSizeMultiplier(double factor)
{
    if (!m_blobModel) {
        qWarning() << "AppController::setRoiSizeMultiplier: blob model not available";
        return;
    }
    m_blobModel->updateRoiSizeMultiplier(factor);
}

void AppController::requestStartTracking(const QString& videoPath,
                                        int keyFrame,
                                        const Thresholding::ThresholdSettings& settings,
                                        const std::vector<Tracking::InitialWormInfo>& initialWorms,
                                        int totalFrames)
{
    if (!m_manager) {
        qWarning() << "AppController::requestStartTracking: TrackingManager not available";
        emit trackingFailed("Internal error: TrackingManager missing");
        return;
    }

    // Emit that tracking is starting
    emit trackingStarted();

    // For now, dataDirectory left empty; UI (MainWindow) may decide to extend this API to pass it explicitly.
    QString dataDirectory;
    m_manager->startFullTrackingProcess(videoPath, dataDirectory, keyFrame, initialWorms, settings, totalFrames);
}

void AppController::cancelTracking()
{
    if (!m_manager) {
        qWarning() << "AppController::cancelTracking: TrackingManager not available";
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
        qWarning() << "AppController::onTrackingManagerAllTracksUpdated: storage missing";
    } else {
        // Store tracks into central storage (merge/overwrite per item)
        for (auto it = tracks.begin(); it != tracks.end(); ++it) {
            int wormId = it->first;
            const std::vector<Tracking::WormTrackPoint>& track = it->second;
            m_storage->setTrackForItem(wormId, track);
            qDebug() << "AppController: stored track for worm" << wormId << "with" << (int)track.size() << "points";
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