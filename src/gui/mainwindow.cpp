// Disconnect cleanup-tab MiniLoaders (mlon2, mlon1, miniLoaderOverlay, mlop1, mlop2)
// from Tracking-tab UI by removing connect() calls and preventing updates
// from frame/selection/model signals; keep primary `miniLoader` connected.

#include "mainwindow.h"
#include <QTimer>
#include <QShortcut>
#include "../utils/debugutils.h"
#include "ui_mainwindow.h"
#include "widgets/miniloader.h"
#include "../models/annotationtablemodel.h"
#include "../models/blobtablemodel.h"
#include "delegates/colordelegate.h"
#include "delegates/itemtypedelegate.h"
#include "trackingprogressdialog.h"
#include "../core/appcontroller.h"
#include "../data/trackingdatastorage.h"
#include "version.h"

#include "widgets/cachestatuswidget.h"

#include <QStandardPaths>
#include <QFileDialog>
#include <QIcon>
#include <QMessageBox>
#include <QDebug>
#include <QButtonGroup> // For m_interactionModeButtonGroup
#include <QSet>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_blobTableModel(nullptr)
    , m_colorDelegate(nullptr)
    , m_itemTypeDelegate(nullptr)
    , m_trackingManager(nullptr)
    , m_interactionModeButtonGroup(new QButtonGroup(this))
    , m_trackingDataStorage(nullptr)
    , m_hasCompletedTracking(false)
    , roiFactorSpinBoxD(1.5) // Initialize ROI factor to default value 1.5
{
    ui->setupUi(this);

    // Create application controller which owns storage, models, and TrackingManager
    m_appController = new AppController(this);

    // Retrieve storage from controller and keep a local pointer for legacy code paths in MainWindow
    m_trackingDataStorage = m_appController->trackingDataStorage();

    // Set up MiniLoader instances with the storage obtained from AppController
    // We keep the widgets around, but we will explicitly avoid wiring the cleanup-tab mini loaders
    // (mlon2, mlon1, miniLoaderOverlay, mlop1, mlop2) to tracking-tab signals.
    if (ui->miniLoader) {
        ui->miniLoader->setTrackingDataStorage(m_trackingDataStorage);
        ui->miniLoader->setShowOverlays(false);  // No overlays on main miniLoader
    }

    if (ui->miniLoaderOverlay) {
        // Keep storage and overlay mode configured, but we will NOT connect it to tracking tab signals.
        ui->miniLoaderOverlay->setTrackingDataStorage(m_trackingDataStorage);
        ui->miniLoaderOverlay->setShowOverlays(true);  // Overlays enabled on overlay instance
    }

    // Set up all other MiniLoader instances with TrackingDataStorage (they live on the Cleanup tab)
    // They will not be connected to the tracking UI's signals.
    if (ui->mlon2) {
        ui->mlon2->setTrackingDataStorage(m_trackingDataStorage);
        ui->mlon2->setShowOverlays(true);
    }
    if (ui->mlon1) {
        ui->mlon1->setTrackingDataStorage(m_trackingDataStorage);
        ui->mlon1->setShowOverlays(true);
    }
    if (ui->mlop1) {
        ui->mlop1->setTrackingDataStorage(m_trackingDataStorage);
        ui->mlop1->setShowOverlays(true);
    }
    if (ui->mlop2) {
        ui->mlop2->setTrackingDataStorage(m_trackingDataStorage);
        ui->mlop2->setShowOverlays(true);
    }

    // Model and Delegates - obtain models from AppController
    m_blobTableModel = m_appController->blobTableModel();
    ui->wormTableView->setModel(m_blobTableModel);

    // Annotation Table Model from controller (no dedicated annotation view in this UI)
    m_annotationTableModel = m_appController->annotationTableModel();
    qDebug() << "MainWindow: AnnotationTableModel obtained from AppController (no view in this UI)";

    // Merge/Split events model and view
    m_mergeSplitModel = new QStandardItemModel(this);
    m_mergeSplitModel->setColumnCount(3);
    m_mergeSplitModel->setHeaderData(0, Qt::Horizontal, "Frame");
    m_mergeSplitModel->setHeaderData(1, Qt::Horizontal, "Event");
    m_mergeSplitModel->setHeaderData(2, Qt::Horizontal, "Details");
    ui->mergeSplitTableView->setModel(m_mergeSplitModel);
    ui->mergeSplitTableView->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    ui->mergeSplitTableView->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    ui->mergeSplitTableView->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);

    // Jump to frame when a merge/split row is clicked
    connect(ui->mergeSplitTableView, &QTableView::clicked, this, [this](const QModelIndex &index){
        if (!index.isValid()) return;
        // Frame number is in column 0
        QModelIndex frameIdx = m_mergeSplitModel->index(index.row(), 0);
        QVariant v = m_mergeSplitModel->data(frameIdx, Qt::DisplayRole);
        bool ok = false;
        int frame = v.toInt(&ok);
        if (ok) seekFrame(frame);
    });

    m_itemTypeDelegate = new ItemTypeDelegate(this);
    ui->wormTableView->setItemDelegateForColumn(BlobTableModel::Column::Type, m_itemTypeDelegate);

    m_colorDelegate = new ColorDelegate(this);
    ui->wormTableView->setItemDelegateForColumn(BlobTableModel::Column::Color, m_colorDelegate);

    ui->wormTableView->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    // Don't stretch last section - we'll handle column widths in resizeTableColumns()
    ui->wormTableView->horizontalHeader()->setStretchLastSection(false);
    // Set header to always be visible, even for empty tables
    ui->wormTableView->horizontalHeader()->setVisible(true);
    // Ensure horizontal scrollbar appears when needed
    ui->wormTableView->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    // Configure selection behavior to select entire rows and allow only single selection
    ui->wormTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->wormTableView->setSelectionMode(QAbstractItemView::SingleSelection);

    // Configure Show/Hide column with checkboxes
    ui->wormTableView->setItemDelegateForColumn(BlobTableModel::Column::Show, nullptr); // Use default delegate for checkboxes
    // Allow checking checkboxes in the header
    ui->wormTableView->horizontalHeader()->setSectionsClickable(true);
    ui->wormTableView->horizontalHeader()->setSectionResizeMode(BlobTableModel::Column::Show, QHeaderView::ResizeToContents);

    resizeTableColumns();

    // TrackingManager is now owned by AppController; MainWindow keeps the pointer field null to avoid accidental direct usage.
    m_trackingManager = nullptr;

    // Pass data storage to VideoLoader
    ui->videoLoader->setTrackingDataStorage(m_trackingDataStorage);

    setupInteractionModeButtonGroup(); // Setup for exclusive interaction mode buttons
    setupPlaybackSpeedComboBox(); // Initialize playback speed options
    setupConnections();
    initializeUIStates();

    // Set initial modes in VideoLoader (after connections are set up)
    // Interaction mode buttons will be synced by syncInteractionModeButtons via the signal
    ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::PanZoom);

    // View mode buttons will be synced by syncViewModeOptionButtons via the signal
    // Set initial active view modes in VideoLoader if desired, e.g., show Blobs by default
    ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Blobs, true);

    qDebug() << "Setting to global" << ui->globalThreshAutoCheck->checkState();
    ui->statusbar->showMessage(QString("Welcome to YAWT version ")+QString(PROJECT_VERSION)+" !", 10000);

    // --- Cache status widget: visualize frames currently present in the VideoLoader cache ---
    // Create widget and add to status bar as a permanent (right-most) widget.
    CacheStatusWidget* cacheStatusWidget = new CacheStatusWidget(this);
    statusBar()->addPermanentWidget(cacheStatusWidget, 0);

    // When VideoLoader emits that a frame was cached, mark it in the widget.
    connect(ui->videoLoader, &VideoLoader::frameCached, cacheStatusWidget, &CacheStatusWidget::onFrameCached);
    // Also listen for explicit eviction notifications so the widget can remove frames immediately.
    connect(ui->videoLoader, &VideoLoader::frameEvicted, cacheStatusWidget, &CacheStatusWidget::onFrameEvicted);

    // When a new video is loaded, update the widget with total frame count.
    connect(ui->videoLoader, &VideoLoader::videoLoaded, this, [cacheStatusWidget](const QString&, int totalFrames, double, QSize) {
        cacheStatusWidget->setTotalFrames(totalFrames);
    });

    // Highlight the current frame as VideoLoader updates.
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, [cacheStatusWidget](int frameNumber, const QImage&) {
        cacheStatusWidget->setCurrentFrame(frameNumber);
    });

    // Clicks on the cache strip will seek the main video to that frame.
    connect(cacheStatusWidget, &CacheStatusWidget::frameClicked, this, [this](int frame) {
        seekFrame(frame);
    });

    // Periodic poll to detect evictions / reconcile cache differences.
    QTimer* cachePollTimer = new QTimer(this);
    cachePollTimer->setInterval(500); // 500 ms
    connect(cachePollTimer, &QTimer::timeout, this, [this, cacheStatusWidget]() {
        if (!ui || !ui->videoLoader) return;
        if (!ui->videoLoader->isVideoLoaded()) {
            cacheStatusWidget->clear();
            return;
        }
        int total = ui->videoLoader->getTotalFrames();
        if (total <= 0) {
            cacheStatusWidget->clear();
            return;
        }
        int center = ui->videoLoader->getCurrentFrameNumber();
        if (center < 0) center = 0;
        int preload = ui->videoLoader->getPreloadRadius();
        // scan radius: cover the preload radius plus a margin, with a reasonable minimum window
        int scanRadius = qMax(preload + 50, 200);
        int start = qMax(0, center - scanRadius);
        int end = qMin(total - 1, center + scanRadius);

        QSet<int> present;
        // Query VideoLoader for frames available in cache (getQImageForFrame returns null if not cached)
        for (int f = start; f <= end; ++f) {
            QImage qi = ui->videoLoader->getQImageForFrame(f);
            if (!qi.isNull()) present.insert(f);
        }

        cacheStatusWidget->setCachedFrames(present);
    });
    cachePollTimer->start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setPlayButtonsState(bool playing, bool blockSignals) {
    // Helper: set playback state and keep button consistent.
    if (!ui) return;
    if (blockSignals) {
        ui->playPauseButton->blockSignals(true);
    }

    ui->playPauseButton->setChecked(playing);

    QIcon icon = playing ? QIcon::fromTheme("media-playback-pause", QIcon(":/icons/pause.png"))
                        : QIcon::fromTheme("media-playback-start", QIcon(":/icons/play.png"));
    ui->playPauseButton->setIcon(icon);

    if (playing) ui->videoLoader->play();
    else ui->videoLoader->pause();

    if (blockSignals) {
        ui->playPauseButton->blockSignals(false);
    }
}

bool MainWindow::arePlayButtonsChecked() const {
    if (!ui) return false;
    return ui->playPauseButton->isChecked();
}

void MainWindow::setupInteractionModeButtonGroup() {
    // Interaction Mode Buttons - These should be checkable QToolButtons
    m_interactionModeButtonGroup->addButton(ui->panModeButton);
    m_interactionModeButtonGroup->addButton(ui->roiModeButton);
    m_interactionModeButtonGroup->addButton(ui->cropModeButton);
    m_interactionModeButtonGroup->addButton(ui->selectionModeButton);  // Rename in UI to "Edit Blobs"
    m_interactionModeButtonGroup->addButton(ui->trackModeButton);      // Rename in UI to "Edit Tracks"
    m_interactionModeButtonGroup->setExclusive(true);
    // No QButtonGroup for view modes as they are independent toggles now
}


// Override resize event to handle table column resizing
void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    // Call resizeTableColumns directly instead of using a signal
    resizeTableColumns();
}

void MainWindow::showEvent(QShowEvent* event) {
    QMainWindow::showEvent(event);
    // Resize table columns when the window is first shown
    // First immediate call
    resizeTableColumns();
    // Then schedule another resize after layout is complete
    QTimer::singleShot(0, this, &MainWindow::resizeTableColumns);
    // And one more after all Qt internal events are processed
    QTimer::singleShot(300, this, &MainWindow::resizeTableColumns);
}

void MainWindow::setupConnections() {
    qDebug() << "MainWindow::setupConnections - invoked (should appear only once)";
    // Connect ROI factor spinbox to BlobTableModel
    connect(ui->roiFactorSpinBoxD, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_blobTableModel, &BlobTableModel::updateRoiSizeMultiplier);

    // File/Directory
    connect(ui->selectDirButton, &QToolButton::clicked, this, &MainWindow::chooseWorkingDirectory);

    // VideoLoader basic signals
    connect(ui->videoLoader, &VideoLoader::videoLoaded, this, &MainWindow::initiateFrameDisplay);
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, &MainWindow::updateFrameDisplay, Qt::UniqueConnection);
    // Keep updateMiniLoaderCrop hooked to frameChanged, but that function is updated to only update the primary miniLoader.
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, &MainWindow::updateMiniLoaderCrop, Qt::UniqueConnection);
    connect(ui->videoLoader, &VideoLoader::interactionModeChanged, this, &MainWindow::syncInteractionModeButtons);
    connect(ui->videoLoader, &VideoLoader::activeViewModesChanged, this, &MainWindow::syncViewModeOptionButtons); // Updated signal

    // Connect VideoLoader frameCached signal only to the primary miniLoader (tracking-tab)
    // Cleanup-tab mini loaders intentionally NOT connected here.
    if (ui->miniLoader) connect(ui->videoLoader, &VideoLoader::frameCached, ui->miniLoader, &MiniLoader::onFrameCached, Qt::UniqueConnection);

    // When an ROI is drawn in VideoLoader, add it as an ROI item in the BlobTableModel
    connect(ui->videoLoader, &VideoLoader::roiDefined, this, &MainWindow::handleRoiDefined);

    // Playback controls (single button) - use helper to keep it in sync
    connect(ui->playPauseButton, &QToolButton::toggled, this, [this](bool checked) { setPlayButtonsState(checked); });
    connect(ui->firstFrameButton, &QToolButton::clicked, this, &MainWindow::goToFirstFrame);
    connect(ui->firstFrameButton_2, &QToolButton::clicked, this, &MainWindow::goToFirstFrame);
    connect(ui->lastFrameButton, &QToolButton::clicked, this, &MainWindow::goToLastFrame);
    connect(ui->lastFrameButton_2, &QToolButton::clicked, this, &MainWindow::goToLastFrame);
    connect(ui->framePosition, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::seekFrame);
    // Mirror the frame position spinbox for the secondary UI (framePosition_2)
    connect(ui->framePosition_2, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::seekFrame);
    connect(ui->frameSlider, &QAbstractSlider::valueChanged, this, &MainWindow::frameSliderMoved);

    // Interaction Mode Buttons -> VideoLoader (via slots that call VideoLoader)
    connect(ui->panModeButton, &QToolButton::clicked, this, &MainWindow::panModeButtonClicked);
    connect(ui->roiModeButton, &QToolButton::clicked, this, &MainWindow::roiModeButtonClicked);
    connect(ui->cropModeButton, &QToolButton::clicked, this, &MainWindow::cropModeButtonClicked);
    connect(ui->selectionModeButton, &QToolButton::clicked, this, &MainWindow::editBlobsModeButtonClicked);
    connect(ui->trackModeButton, &QToolButton::clicked, this, &MainWindow::editTracksModeButtonClicked);

    // View Mode Option Buttons (Checkable QToolButtons or QCheckBoxes) -> VideoLoader
    connect(ui->viewThreshButton, &QToolButton::toggled, this, &MainWindow::onViewThresholdToggled);
    connect(ui->viewBlobsButton, &QToolButton::toggled, this, &MainWindow::onViewBlobsToggled);
    connect(ui->viewTracksButton, &QToolButton::toggled, this, &MainWindow::onViewTracksToggled);

    // Thresholding UI -> VideoLoader & MainWindow
    connect(ui->globalRadio, &QRadioButton::clicked, this, &MainWindow::updateThresholdAlgorithmSettings);
    connect(ui->adaptiveRadio, &QRadioButton::clicked, this, &MainWindow::updateThresholdAlgorithmSettings);
    connect(ui->globalThreshAutoCheck, &QCheckBox::toggled, this, &MainWindow::setGlobalThresholdType);
    connect(ui->globalThreshSlider, &QAbstractSlider::valueChanged, this, &MainWindow::setGlobalThresholdValue);
    connect(ui->globalThreshValueSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::setGlobalThresholdValue);
    connect(ui->adaptiveTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::setAdaptiveThresholdType);
    connect(ui->blockSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::setAdaptiveBlockSize);
    connect(ui->tuningDoubleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::setAdaptiveCValue);
    connect(ui->blurCheck, &QCheckBox::toggled, this, &MainWindow::setBlurEnabled);
    connect(ui->blurKernelSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::setBlurKernel);
    connect(ui->bgCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::setBackgroundAssumption);

    // VideoLoader -> MainWindow (for adding blobs)
    connect(ui->videoLoader, &VideoLoader::blobClickedForAddition, this, &MainWindow::handleBlobClickedForAddition);

    // BlobTableModel -> VideoLoader
    connect(m_blobTableModel, &BlobTableModel::itemsChanged, ui->videoLoader, &VideoLoader::updateItemsToDisplay);
    connect(m_blobTableModel, &BlobTableModel::itemVisibilityChanged,
            [this](int id, bool visible) {
                // When an item's visibility changes, update the VideoLoader with current items
                ui->videoLoader->updateItemsToDisplay(m_blobTableModel->getAllItems());
            });

    // NOTE: We purposely DO NOT connect the cleanup-tab mini loaders (mlon2, mlon1, miniLoaderOverlay, mlop1, mlop2)
    // to model changes or selection changes here. Those widgets are isolated on the Cleanup tab for separate debugging.

    connect(ui->clearAllButton, &QPushButton::clicked, this, &MainWindow::handleRemoveBlobsClicked);
    connect(ui->deleteButton, &QPushButton::clicked, this, &MainWindow::handleDeleteSelectedBlobClicked);

    // Auto-resize table columns when model data changes
    connect(m_blobTableModel, &BlobTableModel::dataChanged, this, &MainWindow::resizeTableColumns);
    connect(m_blobTableModel, &BlobTableModel::rowsInserted, this, &MainWindow::resizeTableColumns);
    connect(m_blobTableModel, &BlobTableModel::rowsRemoved, this, &MainWindow::resizeTableColumns);

    // Table View Selection -> VideoLoader
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &MainWindow::updateVisibleTracksInVideoLoader);

    // When selection changes, update merge/split events table
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [this](const QItemSelection &selected, const QItemSelection&) {
        if (selected.isEmpty()) {
            m_mergeSplitModel->removeRows(0, m_mergeSplitModel->rowCount());
            return;
        }
        int row = selected.indexes().first().row();
        if (row < 0 || row >= m_blobTableModel->rowCount()) return;
        const TableItems::ClickedItem& sel = m_blobTableModel->getItem(row);
        // Populate merge/split table for this worm
        m_mergeSplitModel->removeRows(0, m_mergeSplitModel->rowCount());
        // Determine frame range from video loader
        int maxFrame = ui->videoLoader->getTotalFrames();
        QList<int> prevGroup; // empty means not merged on previous frame
        for (int f = 0; f < maxFrame; ++f) {
            QList<QList<int>> groups = m_trackingDataStorage->getMergeGroupsForFrame(f);
            QList<int> currGroup;
            for (const QList<int>& g : groups) {
                if (g.contains(sel.id)) { currGroup = g; break; }
            }

            QSet<int> prevSet;
            for (int id : prevGroup) prevSet.insert(id);
            QSet<int> currSet;
            for (int id : currGroup) currSet.insert(id);
            if (prevSet == currSet) {
                prevGroup = currGroup;
                continue; // no change
            }

            QString eventType;
            QString details;

            if (prevSet.isEmpty() && !currSet.isEmpty()) {
                eventType = "Merge";
                QSet<int> others = currSet;
                others.remove(sel.id);
                if (others.isEmpty()) details = "Merged (no other ids)";
                else {
                    QStringList ids;
                    for (int id : others) ids << QString::number(id);
                    details = QString("Merged with worm(s) %1").arg(ids.join(", "));
                }
            } else if (!prevSet.isEmpty() && currSet.isEmpty()) {
                eventType = "Split";
                QSet<int> others = prevSet;
                others.remove(sel.id);
                if (others.isEmpty()) details = "Split (no other ids)";
                else {
                    QStringList ids;
                    for (int id : others) ids << QString::number(id);
                    details = QString("Split from worm(s) %1").arg(ids.join(", "));
                }
            } else {
                QSet<int> added = currSet - prevSet;
                QSet<int> removed = prevSet - currSet;
                if (!added.isEmpty() && removed.isEmpty()) {
                    eventType = "Merge";
                    QSet<int> others = added;
                    others.remove(sel.id);
                    QStringList ids;
                    for (int id : others) ids << QString::number(id);
                    details = QString("Merged with worm(s) %1").arg(ids.join(", "));
                } else if (added.isEmpty() && !removed.isEmpty()) {
                    eventType = "Split";
                    QSet<int> others = removed;
                    others.remove(sel.id);
                    QStringList ids;
                    for (int id : others) ids << QString::number(id);
                    details = QString("Split from worm(s) %1").arg(ids.join(", "));
                } else {
                    eventType = "Merge/Split";
                    QStringList parts;
                    if (!added.isEmpty()) {
                        QStringList ids; for (int id : added) ids << QString::number(id);
                        parts << QString("Added: %1").arg(ids.join(", "));
                    }
                    if (!removed.isEmpty()) {
                        QStringList ids; for (int id : removed) ids << QString::number(id);
                        parts << QString("Removed: %1").arg(ids.join(", "));
                    }
                    details = parts.join("; ");
                }
            }

            QList<QStandardItem*> rowItems;
            rowItems << new QStandardItem(QString::number(f));
            rowItems << new QStandardItem(eventType);
            rowItems << new QStandardItem(details);
            m_mergeSplitModel->appendRow(rowItems);

            prevGroup = currGroup;
        }
    });

    // Enable/disable delete button based on selection state
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [this](const QItemSelection &selected, const QItemSelection &deselected) {
        ui->deleteButton->setEnabled(!selected.isEmpty());
    });

    // Table View Selection -> MiniLoader (update crop when selection changes)
    // Keep this connected so the primary miniLoader (display on Tracking tab) is updated.
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [this](const QItemSelection &selected, const QItemSelection &deselected) {
        Q_UNUSED(selected)
        Q_UNUSED(deselected)
        // When selection changes, update the primary miniLoader crop with current frame
        if (ui->videoLoader && ui->videoLoader->isVideoLoaded()) {
            int currentFrame = ui->videoLoader->getCurrentFrameNumber();
            QImage currentImage = ui->videoLoader->getCurrentQImageFrame();
            if (!currentImage.isNull()) {
                updateMiniLoaderCrop(currentFrame, currentImage);
            }
        }
    });

    // Playback speed control
    connect(ui->comboPlaybackSpeed, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onPlaybackSpeedChanged);
    // Mirror playback speed control for the secondary combobox (comboPlaybackSpeed_2)
    connect(ui->comboPlaybackSpeed_2, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onPlaybackSpeedChanged);

    // Update combobox when VideoLoader speed changes (optional - for consistency)
    connect(ui->videoLoader, &VideoLoader::playbackSpeedChanged,
            this, &MainWindow::updatePlaybackSpeedComboBox);

    // Connect to headerClicked signal for handling Show/Hide column header click
    connect(ui->wormTableView->horizontalHeader(), &QHeaderView::sectionClicked,
            [this](int logicalIndex) {
        if (logicalIndex == BlobTableModel::Column::Show) {
            QVariant checkState = m_blobTableModel->headerData(
                BlobTableModel::Column::Show, Qt::Horizontal, Qt::CheckStateRole);

            if (checkState.isValid()) {
                Qt::CheckState newState;
                if (checkState.toInt() == Qt::Unchecked) {
                    newState = Qt::Checked;
                } else {
                    newState = Qt::Unchecked;
                }

                m_blobTableModel->setHeaderData(
                    BlobTableModel::Column::Show, Qt::Horizontal,
                    newState, Qt::CheckStateRole);
            }
        }
    });

    // Video File Tree View -> VideoLoader
    connect(ui->videoTreeView, &VideoFileTreeView::videoFileDoubleClicked, ui->videoLoader, &VideoLoader::loadVideo);

    // Tracking Process
    connect(ui->trackingDialogButton, &QPushButton::clicked, this, &MainWindow::onStartTrackingActionTriggered);
    // Listen for tracksUpdated from AppController (controller stores tracks into storage and emits this)
    connect(m_appController, &AppController::tracksUpdated, this, &MainWindow::acceptTracksFromManager);

    // Annotation table selection - removed (no view). If UI regains an annotation view, reconnect here.

    // Connect header data changes to trigger UI update
    connect(m_blobTableModel, &QAbstractItemModel::headerDataChanged,
            this, [this](Qt::Orientation orientation, int first, int last) {
        if (orientation == Qt::Horizontal && first <= BlobTableModel::Column::Show && last >= BlobTableModel::Column::Show) {
            // Update the table view when header checkbox state changes
            ui->wormTableView->update();
        }
    });

    // Initial call to setVisibleTrackIDs with all item IDs
    QSet<int> initialItemIDs;
    for (const auto& item : m_blobTableModel->getAllItems()) {
        initialItemIDs.insert(item.id);
    }
    ui->videoLoader->setVisibleTrackIDs(initialItemIDs);

    // Only update primary miniLoader display; do NOT update cleanup-tab mini loaders here.
    if (ui->miniLoader) ui->miniLoader->update();

    // Start a short polling timer as a fallback in case signals are missed or paint() hasn't yet emitted.
    // This keeps an internal list of visible IDs in sync with the primary miniLoader only.
    m_miniLoaderPollTimer = new QTimer(this);
    connect(m_miniLoaderPollTimer, &QTimer::timeout, this, &MainWindow::onMiniLoaderPollTimeout);
    m_miniLoaderPollTimer->setInterval(100); // 100 ms poll interval
    m_miniLoaderPollTimer->start();
    qDebug() << "MainWindow: started miniLoader poll timer (100ms)";

    // Debug control keyboard shortcut
    QShortcut* debugToggle = new QShortcut(QKeySequence("Ctrl+D"), this);
    connect(debugToggle, &QShortcut::activated, this, &MainWindow::toggleTrackingDebug);

    // Connections for TrackingProgressDialog are made when it's created/shown
}

void MainWindow::initializeUIStates() {
    QString initialDirectory = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    ui->videoTreeView->setRootDirectory(initialDirectory);
    ui->dirSelected->setText(initialDirectory);

    ui->framePosition->setKeyboardTracking(false);
    // Ensure the duplicated spinbox does not emit continuous keyboard-tracking updates
    ui->framePosition_2->setKeyboardTracking(false);
    ui->frameSlider->setMinimum(0);
    ui->frameSlider->setSingleStep(10);
    ui->frameSlider->setPageStep(100);
    // Initialize play button to the "play" state and icon without emitting signals
    setPlayButtonsState(false, /*blockSignals=*/false);

    // Initialize delete button to disabled state since there are no items selected initially
    ui->deleteButton->setEnabled(false);

    ui->adaptiveTypeCombo->setCurrentIndex(0);
    setAdaptiveThresholdType(ui->adaptiveTypeCombo->currentIndex());

    ui->blockSizeSpin->setValue(11);
    setAdaptiveBlockSize(ui->blockSizeSpin->value());

    ui->tuningDoubleSpin->setValue(2.0);
    setAdaptiveCValue(ui->tuningDoubleSpin->value());

    ui->blurCheck->setChecked(false);
    setBlurEnabled(ui->blurCheck->isChecked());
    ui->blurKernelSpin->setValue(5);
    setBlurKernel(ui->blurKernelSpin->value());

    ui->bgCombo->setCurrentIndex(0);
    setBackgroundAssumption(ui->bgCombo->currentIndex());
    ui->globalRadio->setChecked(true);
    ui->globalThreshSlider->setValue(100);
    ui->globalThreshAutoCheck->setChecked(false);
    qDebug() << "Setting to global" << ui->globalThreshAutoCheck->checkState();
    updateThresholdAlgorithmSettings();

    // Initialize ROI factor spinbox
    ui->roiFactorSpinBoxD->setValue(roiFactorSpinBoxD);
    ui->roiFactorSpinBoxD->setSingleStep(0.05);
    // Set initial value in the model
    m_blobTableModel->updateRoiSizeMultiplier(roiFactorSpinBoxD);

    // Initial button states will be set by sync slots when VideoLoader emits initial modes
}

void MainWindow::resizeTableColumns()
{
    if (!m_blobTableModel || m_blobTableModel->columnCount() == 0) {
        return;
    }

    // Get available viewport width and column count - we'll need these throughout the method
    int viewportWidth = ui->wormTableView->viewport()->width();
    int columnCount = m_blobTableModel->columnCount();

    // Ensure header is visible even when table is empty
    ui->wormTableView->horizontalHeader()->setVisible(true);
    ui->wormTableView->horizontalHeader()->setStretchLastSection(false);

    // Set minimum width for each column to ensure readability
    ui->wormTableView->horizontalHeader()->setMinimumSectionSize(10);

    // First, resize all columns to fit their contents
    int totalContentWidth = 0;
    QVector<int> contentWidths(columnCount);

    for (int i = 0; i < columnCount; ++i) {
        ui->wormTableView->resizeColumnToContents(i);
        contentWidths[i] = ui->wormTableView->horizontalHeader()->sectionSize(i);
        totalContentWidth += contentWidths[i];
    }

    // Use the viewport width we already calculated

    // Decide whether to expand columns or use scrollbar
    if (viewportWidth > totalContentWidth && totalContentWidth > 0) {
        // Extra space available - expand columns proportionally
        float expansionRatio = static_cast<float>(viewportWidth) / totalContentWidth;

        for (int i = 0; i < columnCount; ++i) {
            int newWidth = qRound(contentWidths[i] * expansionRatio);
            ui->wormTableView->horizontalHeader()->setSectionResizeMode(i, QHeaderView::Interactive);
            ui->wormTableView->horizontalHeader()->resizeSection(i, newWidth);
        }
    } else {
        // Content requires more space than available - keep content width and enable scrollbar
        for (int i = 0; i < columnCount; ++i) {
            ui->wormTableView->horizontalHeader()->setSectionResizeMode(i, QHeaderView::Interactive);
            // Keep the content width we already set with resizeColumnToContents
        }
        // Make sure horizontal scrollbar is enabled when needed
        ui->wormTableView->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    }
}


// --- Mode Toggling Slots ---
void MainWindow::panModeButtonClicked() {
    ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::PanZoom);
}
void MainWindow::roiModeButtonClicked() {
    ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::DrawROI);
}
void MainWindow::cropModeButtonClicked() {
    ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::Crop);
}
void MainWindow::editBlobsModeButtonClicked() {
    ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::EditBlobs);
    // Ensure threshold view is active for blob editing if it's a prerequisite
    if (!ui->videoLoader->getActiveViewModes().testFlag(VideoLoader::ViewModeOption::Threshold)) {
        ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Threshold, true);
    }
    if (!ui->videoLoader->getActiveViewModes().testFlag(VideoLoader::ViewModeOption::Blobs)) {
        ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Blobs, true);
    }

    // Provide mode-specific feedback
    QString modeMessage;
    if (m_hasCompletedTracking) {
        modeMessage = "Blob Edit Mode: Click on blobs to add Fix markers";
    } else {
        modeMessage = "Blob Edit Mode: Click on blobs to add Worms for tracking";
    }
    statusBar()->showMessage(modeMessage, 5000);
}
void MainWindow::editTracksModeButtonClicked() {
    ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::EditTracks);
    // Ensure tracks view is active
    if (!ui->videoLoader->getActiveViewModes().testFlag(VideoLoader::ViewModeOption::Tracks)) {
        ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Tracks, true);
    }
}

// --- View Mode Option Toggle Slots (New) ---
void MainWindow::onViewThresholdToggled(bool checked) {
    ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Threshold, checked);
}
void MainWindow::onViewBlobsToggled(bool checked) {
    ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Blobs, checked);
}
void MainWindow::onViewTracksToggled(bool checked) {
    ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Tracks, checked);
}


// --- Slots to sync UI buttons with VideoLoader's state ---
void MainWindow::syncInteractionModeButtons(VideoLoader::InteractionMode newMode) {
    ui->panModeButton->setChecked(newMode == VideoLoader::InteractionMode::PanZoom);
    ui->roiModeButton->setChecked(newMode == VideoLoader::InteractionMode::DrawROI);
    ui->cropModeButton->setChecked(newMode == VideoLoader::InteractionMode::Crop);
    ui->selectionModeButton->setChecked(newMode == VideoLoader::InteractionMode::EditBlobs);
    ui->trackModeButton->setChecked(newMode == VideoLoader::InteractionMode::EditTracks);
    qDebug() << "MainWindow: Interaction mode UI synced to" << static_cast<int>(newMode);
}

void MainWindow::syncViewModeOptionButtons(VideoLoader::ViewModeOptions newModes) {
    ui->viewThreshButton->setChecked(newModes.testFlag(VideoLoader::ViewModeOption::Threshold));
    ui->viewBlobsButton->setChecked(newModes.testFlag(VideoLoader::ViewModeOption::Blobs));
    ui->viewTracksButton->setChecked(newModes.testFlag(VideoLoader::ViewModeOption::Tracks));
    qDebug() << "MainWindow: View mode UI synced. Flags:" << QString::number(static_cast<int>(newModes), 16);
}


// --- Thresholding Callbacks ---
void MainWindow::updateThresholdAlgorithmSettings() {
    bool isAdaptive = ui->adaptiveRadio->isChecked();
    ui->globalRadio->setChecked(!isAdaptive);
    ui->adaptiveGroupBox->setEnabled(isAdaptive);
    ui->globalGroupBox->setEnabled(!isAdaptive);

    if (isAdaptive) {
        setAdaptiveThresholdType(ui->adaptiveTypeCombo->currentIndex());
    } else {
        setGlobalThresholdType(ui->globalThreshAutoCheck->isChecked());
    }
}

void MainWindow::setGlobalThresholdType(bool isAuto) {
    ui->globalThreshSlider->setDisabled(isAuto);
    ui->globalThreshValueSpin->setDisabled(isAuto);
    if (isAuto) {
        ui->videoLoader->setThresholdAlgorithm(Thresholding::ThresholdAlgorithm::Otsu);
    } else {
        ui->videoLoader->setThresholdAlgorithm(Thresholding::ThresholdAlgorithm::Global);
        ui->videoLoader->setThresholdValue(ui->globalThreshSlider->value());
    }
}

void MainWindow::setAdaptiveThresholdType(int index) {
    if (index == 0) {
        ui->videoLoader->setThresholdAlgorithm(Thresholding::ThresholdAlgorithm::AdaptiveGaussian);
    } else {
        ui->videoLoader->setThresholdAlgorithm(Thresholding::ThresholdAlgorithm::AdaptiveMean);
    }
}
void MainWindow::setGlobalThresholdValue(int value) {
    if (sender() == ui->globalThreshSlider) ui->globalThreshValueSpin->setValue(value);
    else if (sender() == ui->globalThreshValueSpin) ui->globalThreshSlider->setValue(value);
    else { ui->globalThreshSlider->setValue(value); ui->globalThreshValueSpin->setValue(value); }
    if (!ui->globalThreshAutoCheck->isChecked()) {
        ui->videoLoader->setThresholdValue(value);
    }
}
void MainWindow::setAdaptiveBlockSize(int value) {
    int validValue = value; if (validValue < 3) validValue = 3; if (validValue % 2 == 0) validValue++;
    if (ui->blockSizeSpin->value() != validValue) ui->blockSizeSpin->setValue(validValue);
    ui->videoLoader->setAdaptiveThresholdBlockSize(validValue);
}
void MainWindow::setAdaptiveCValue(double value) {
    ui->videoLoader->setAdaptiveThresholdC(value);
}
void MainWindow::setBlurEnabled(bool checked) {
    ui->videoLoader->setEnableBlur(checked);
    ui->blurKernelSpin->setEnabled(checked);
}
void MainWindow::setBlurKernel(int value) {
    int validValue = value; if (validValue < 3) validValue = 3; if (validValue % 2 == 0) validValue++;
    if (ui->blurKernelSpin->value() != validValue) ui->blurKernelSpin->setValue(validValue);
    ui->videoLoader->setBlurKernelSize(validValue);
}
void MainWindow::setBackgroundAssumption(int index) {
    ui->videoLoader->setAssumeLightBackground(index == 0);
}

// --- Blob Handling ---
void MainWindow::handleBlobClickedForAddition(const Tracking::DetectedBlob& blobData) {
    if (!ui->videoLoader->isVideoLoaded()) return;
    int currentFrame = ui->videoLoader->getCurrentFrameNumber();

    TableItems::ItemType itemType = TableItems::ItemType::Worm;

    bool added = m_blobTableModel->addItem(blobData.centroid, blobData.boundingBox, currentFrame, itemType);

    if (added) {
        ui->deleteButton->setEnabled(true);

        int lastRow = m_blobTableModel->rowCount() - 1;
        QModelIndex newIndex = m_blobTableModel->index(lastRow, 0);
        ui->wormTableView->setCurrentIndex(newIndex);
        ui->wormTableView->selectionModel()->select(newIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

        resizeTableColumns();

        QString feedbackMessage;
        feedbackMessage = QString("Added Worm blob at frame %1").arg(currentFrame);
        statusBar()->showMessage(feedbackMessage, 3000);
    }
}

void MainWindow::handleRoiDefined(const QRectF& roi) {
    if (!ui->videoLoader->isVideoLoaded()) return;
    int currentFrame = ui->videoLoader->getCurrentFrameNumber();

    QPointF centroid = QPointF(roi.x() + roi.width() / 2.0, roi.y() + roi.height() / 2.0);
    QRectF boundingBox = roi;

    bool added = m_blobTableModel->addItem(centroid, boundingBox, currentFrame, TableItems::ItemType::ROI);

    if (added) {
        ui->deleteButton->setEnabled(true);
        int lastRow = m_blobTableModel->rowCount() - 1;
        QModelIndex newIndex = m_blobTableModel->index(lastRow, 0);
        ui->wormTableView->setCurrentIndex(newIndex);
        ui->wormTableView->selectionModel()->select(newIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
        resizeTableColumns();
        statusBar()->showMessage(QString("Added ROI at frame %1").arg(currentFrame), 3000);
    }
}

void MainWindow::handleRemoveBlobsClicked() {
    m_blobTableModel->removeRows(0, m_blobTableModel->getAllItems().length());
    ui->deleteButton->setEnabled(false);
    ui->videoLoader->updateItemsToDisplay(QList<TableItems::ClickedItem>());
    ui->videoLoader->setVisibleTrackIDs(QSet<int>());
    resizeTableColumns();
}

void MainWindow::handleDeleteSelectedBlobClicked() {
    QModelIndexList selectedIndexes = ui->wormTableView->selectionModel()->selectedIndexes();

    if (!selectedIndexes.isEmpty()) {
        int selectedRow = selectedIndexes.first().row();
        m_blobTableModel->removeRows(selectedRow, 1);

        if (m_blobTableModel->rowCount() > 0) {
            int newRow = (selectedRow < m_blobTableModel->rowCount()) ? selectedRow : m_blobTableModel->rowCount() - 1;
            QModelIndex newIndex = m_blobTableModel->index(newRow, 0);
            ui->wormTableView->setCurrentIndex(newIndex);
            ui->wormTableView->selectionModel()->select(newIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
        }

        resizeTableColumns();
    }
}

// --- Video Playback and Frame Navigation ---
void MainWindow::chooseWorkingDirectory() {
    QString currDir = ui->dirSelected->text();
    QString selectedDir = QFileDialog::getExistingDirectory(this, "Choose working directory", currDir,  QFileDialog::ShowDirsOnly );
    if (!selectedDir.isEmpty()) {
        ui->dirSelected->setText(QFileInfo(selectedDir).absoluteFilePath());
        ui->videoTreeView->setRootDirectory(QFileInfo(selectedDir).absoluteFilePath());
    }
}

void MainWindow::initiateFrameDisplay(const QString& filePath, int totalFrames, double fps, QSize frameSize) {
    ui->frameSlider->setMaximum(totalFrames > 0 ? totalFrames - 1 : 0);
    ui->frameSlider->setValue(0);
    ui->framePosition->setMaximum(totalFrames > 0 ? totalFrames - 1 : 0);
    ui->framePosition->setValue(0);
    ui->framePosition_2->setMaximum(totalFrames > 0 ? totalFrames - 1 : 0);
    ui->framePosition_2->setValue(0);
    ui->fpsLabel->setText(QString::number(fps, 'f', 2) + " fps");
    ui->videoNameLabel->setText(QFileInfo(filePath).fileName());
}

void MainWindow::updateFrameDisplay(int currentFrameNumber, const QImage& currentFrame) {
    Q_UNUSED(currentFrame);
    if (!ui->frameSlider->isSliderDown()) {
        ui->frameSlider->setValue(currentFrameNumber);
    }
    ui->framePosition->setValue(currentFrameNumber);
    ui->framePosition_2->setValue(currentFrameNumber);
}

// Update only the primary miniLoader (on the Tracking tab). Cleanup-tab mini loaders are intentionally NOT updated here.
void MainWindow::updateMiniLoaderCrop(int currentFrameNumber, const QImage& currentFrame) {
    if (!ui->videoLoader || !ui->videoLoader->isVideoLoaded()) {
        return;
    }

    static int m_lastMiniLoaderUpdateFrame = -1;
    if (m_lastMiniLoaderUpdateFrame == currentFrameNumber) {
        qDebug() << "MainWindow::updateMiniLoaderCrop - skipping redundant update for frame" << currentFrameNumber;
        return;
    }
    m_lastMiniLoaderUpdateFrame = currentFrameNumber;

    QSizeF cropSize = m_blobTableModel->getCurrentFixedRoiSize();
    if (cropSize.isEmpty()) {
        cropSize = QSizeF(100, 100); // fallback size
    }

    // Determine selected worm (if any) so we can center primary miniLoader on it
    int selectedWormId = -1;
    QModelIndexList selectedIndexes = ui->wormTableView->selectionModel()->selectedIndexes();
    if (!selectedIndexes.isEmpty()) {
        int selectedRow = selectedIndexes.first().row();
        if (selectedRow >= 0 && selectedRow < m_blobTableModel->rowCount()) {
            const TableItems::ClickedItem& selectedItem = m_blobTableModel->getItem(selectedRow);
            selectedWormId = selectedItem.id;
        }
    }

    QPointF centerPoint;
    bool foundCenterPoint = false;

    if (selectedWormId != -1) {
        QPointF wormPosition;
        QRectF wormRoi;
        if (m_trackingDataStorage->getWormDataForFrame(selectedWormId, currentFrameNumber, wormPosition, wormRoi)) {
            centerPoint = wormPosition;
            foundCenterPoint = true;
        } else if (m_trackingDataStorage->getLastKnownPositionBefore(selectedWormId, currentFrameNumber, wormPosition, wormRoi)) {
            centerPoint = wormPosition;
            foundCenterPoint = true;
        }
    }

    if (!foundCenterPoint) {
        centerPoint = QPointF(currentFrame.width() / 2.0, currentFrame.height() / 2.0);
    }

    if (ui->miniLoader) {
        qDebug() << "MainWindow::updateMiniLoaderCrop - setting expected frame" << currentFrameNumber << "for primary miniLoader" << ui->miniLoader;
        ui->miniLoader->setExpectedFrame(currentFrameNumber, cropSize, centerPoint, ui->videoLoader);
    }
}

void MainWindow::frameSliderMoved(int value) {
    ui->videoLoader->seekToFrame(value, false);
    if (!ui->framePosition->hasFocus()) {
        ui->framePosition->setValue(value);
    }
}

// Slot: receive visible worm IDs from MiniLoader. Update cached set and rebuild merge history text.
// Emitted frequently (on repaint), so keep this lightweight.
void MainWindow::onMiniLoaderVisibleWormsUpdated(const QList<int>& visibleIds) {
    qDebug() << "MainWindow::onMiniLoaderVisibleWormsUpdated received visibleIds:" << visibleIds;

    m_lastPolledVisibleIds.clear();
    for (int id : visibleIds) m_lastPolledVisibleIds.insert(id);
}

// Poll timer fallback handler: periodically query the primary MiniLoader instance for visible IDs and update
// MainWindow state if the polled set changes.
void MainWindow::onMiniLoaderPollTimeout() {
    QSet<int> polledSet;
    // Query primary miniLoader instance if present
    if (ui->miniLoader) {
        const QList<int> vis = ui->miniLoader->getVisibleWormIds();
        for (int id : vis) polledSet.insert(id);
    }

    if (polledSet != m_lastPolledVisibleIds) {
        m_lastPolledVisibleIds = polledSet;
        QList<int> list;
        list.reserve(polledSet.size());
        for (int id : polledSet) list.append(id);
        qDebug() << "MainWindow::onMiniLoaderPollTimeout polled visible IDs:" << list;
        onMiniLoaderVisibleWormsUpdated(list);
    }
}

void MainWindow::seekFrame(int frame) {
    ui->videoLoader->seekToFrame(frame, false);
    if (!ui->frameSlider->isSliderDown()) {
        ui->frameSlider->setValue(frame);
    }
}

void MainWindow::goToFirstFrame() {
    ui->videoLoader->seekToFrame(0, false);
    if (!ui->frameSlider->isSliderDown()) {
        ui->frameSlider->setValue(0);
    }
    if (!ui->framePosition->hasFocus()) {
        ui->framePosition->setValue(0);
    }
    if (!ui->framePosition_2->hasFocus()) {
        ui->framePosition_2->setValue(0);
    }
}

void MainWindow::goToLastFrame() {
    int totalFrames = ui->videoLoader->getTotalFrames();
    int lastFrame = totalFrames - 1; // Assuming 0-based indexing
    ui->videoLoader->seekToFrame(lastFrame, false);
    if (!ui->frameSlider->isSliderDown()) {
        ui->frameSlider->setValue(lastFrame);
    }
    if (!ui->framePosition->hasFocus()) {
        ui->framePosition->setValue(lastFrame);
    }
    if (!ui->framePosition_2->hasFocus()) {
        ui->framePosition_2->setValue(lastFrame);
    }
    if (arePlayButtonsChecked())
    {
        // Use helper to pause and sync play button
        setPlayButtonsState(false);
    }
}

// --- Tracking Process ---
void MainWindow::onStartTrackingActionTriggered() {
    if (!ui->videoLoader || !ui->videoLoader->isVideoLoaded()) {
        QMessageBox::warning(this, "Tracking Setup", "No video loaded."); return;
    }
    QString videoPath = ui->videoLoader->getCurrentVideoPath();
    if (videoPath.isEmpty()) {
        QMessageBox::critical(this, "Error", "Video path is empty."); return;
    }

    int wormCount = 0;
    int wormsWithTracks = 0;
    if (m_appController) {
        wormCount = m_appController->countWormItems();
        wormsWithTracks = m_appController->countItemsWithTracks();
    } else if (m_blobTableModel) {
        const QList<TableItems::ClickedItem>& items = m_blobTableModel->getAllItems();
        for (const auto& it : items) if (it.type == TableItems::ItemType::Worm) ++wormCount;
        if (m_trackingDataStorage) wormsWithTracks = m_trackingDataStorage->getItemsWithTracks().size();
    }

    if (wormCount == 0) {
        QMessageBox::information(this, "Tracking", "No items marked as 'Worm' in the table to track.");
        return;
    }

    int keyFrame = ui->videoLoader->getCurrentFrameNumber();
    Thresholding::ThresholdSettings settings = ui->videoLoader->getCurrentThresholdSettings();
    int totalFrames = ui->videoLoader->getTotalFrames();

    if (m_appController) {
        m_appController->showTrackingDialog(videoPath, keyFrame, settings, /*onlyTrackMissing=*/true, totalFrames, this);
    } else {
        QMessageBox::critical(this, "Error", "Internal error: Controller missing.");
    }
}

void MainWindow::acceptTracksFromManager(const Tracking::AllWormTracks& tracks) {
    qDebug() << "MainWindow: Received" << tracks.size() << "tracks (assumed stored by AppController).";

    if (m_trackingDataStorage) {
        qDebug() << "MainWindow: TrackingDataStorage now has" << m_trackingDataStorage->getAllTracks().size() << "tracks";
    }

    if (m_annotationTableModel) {
        m_annotationTableModel->refreshAnnotations();
        qDebug() << "MainWindow: Refreshed annotation model with" << m_annotationTableModel->rowCount() << "annotations";
    }

    if (m_trackingDataStorage) {
        const auto& allTracks = m_trackingDataStorage->getAllTracks();
        qDebug() << "MainWindow: Supplying VideoLoader with" << allTracks.size() << "total tracks from storage";
        ui->videoLoader->setTracksToDisplay(allTracks);
    } else {
        ui->videoLoader->setTracksToDisplay(tracks);
    }

    if (!tracks.empty()) {
        ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Tracks, true);
        ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::PanZoom);
        ui->wormTableView->selectAll();
    }

    m_hasCompletedTracking = true;

    statusBar()->showMessage("Tracking completed", 4000);

    performPostTrackingMemoryCleanup();
}

void MainWindow::setupPlaybackSpeedComboBox() {
    ui->comboPlaybackSpeed->clear();
    ui->comboPlaybackSpeed_2->clear();

    auto populate = [](QComboBox* cb) {
        cb->addItem("0.25x", 0.25);
        cb->addItem("0.5x", 0.5);
        cb->addItem("0.75x", 0.75);
        cb->addItem("1.0x", 1.0);
        cb->addItem("1.5x", 1.5);
        cb->addItem("2.0x", 2.0);
        cb->addItem("2.5x", 2.5);
        cb->addItem("5.0x", 5.0);
        cb->addItem("10.0x", 10.0);
        cb->addItem("20.0x", 20.0);
        cb->setCurrentIndex(3);
    };

    populate(ui->comboPlaybackSpeed);
    populate(ui->comboPlaybackSpeed_2);

    qDebug() << "MainWindow: Playback speed comboboxes (primary and secondary) initialized";
}

void MainWindow::onPlaybackSpeedChanged(int index) {
    QComboBox* senderCombo = qobject_cast<QComboBox*>(sender());
    if (!senderCombo) senderCombo = ui->comboPlaybackSpeed;

    if (index < 0 || index >= senderCombo->count()) {
        return;
    }

    double speedMultiplier = senderCombo->itemData(index).toDouble();

    qDebug() << "MainWindow: Setting playback speed to" << speedMultiplier << "x";

    ui->videoLoader->setPlaybackSpeed(speedMultiplier);

    updatePlaybackSpeedComboBox(speedMultiplier);
}

void MainWindow::updatePlaybackSpeedComboBox(double speedMultiplier) {
    auto updateOne = [&](QComboBox* cb) {
        for (int i = 0; i < cb->count(); ++i) {
            double itemSpeed = cb->itemData(i).toDouble();
            if (qFuzzyCompare(itemSpeed, speedMultiplier)) {
                cb->blockSignals(true);
                cb->setCurrentIndex(i);
                cb->blockSignals(false);
                break;
            }
        }
    };

    updateOne(ui->comboPlaybackSpeed);
    updateOne(ui->comboPlaybackSpeed_2);
}

void MainWindow::onAnnotationTableClicked(const QModelIndex& index) {
    if (!index.isValid() || !m_annotationTableModel) {
        return;
    }

    const AnnotationTableModel::AnnotationEntry* annotation = m_annotationTableModel->getAnnotationAtRow(index.row());
    if (!annotation) {
        qWarning() << "MainWindow: Could not get annotation for row" << index.row();
        return;
    }

    if (arePlayButtonsChecked()) {
        setPlayButtonsState(false);
    }

    int targetFrame = annotation->startFrame;
    int targetWormId = annotation->wormId;

    seekFrame(targetFrame);

    if (!ui->framePosition->hasFocus()) {
        ui->framePosition->setValue(targetFrame);
    }
    if (!ui->framePosition_2->hasFocus()) {
        ui->framePosition_2->setValue(targetFrame);
    }

    int blobTableRow = -1;
    for (int i = 0; i < m_blobTableModel->rowCount(); ++i) {
        const TableItems::ClickedItem& item = m_blobTableModel->getItem(i);
        if (item.id == targetWormId) {
            blobTableRow = i;
            break;
        }
    }

    if (blobTableRow >= 0) {
        QModelIndex blobIndex = m_blobTableModel->index(blobTableRow,
