/**
 * @file mainwindow.cpp
 * @brief MainWindow UI shell: widget wiring, UI logic, delegates, and binding to AppController.
 *
 * Responsibilities:
 *  - Manage and wire UI widgets (VideoLoader, MiniLoader, table views, delegates).
 *  - Bind controller-provided models (BlobTableModel, AnnotationTableModel) to views.
 *  - Handle user interactions: file selection, playback, ROI creation, threshold controls.
 *  - Keep UI in sync with VideoLoader interaction/view modes and visible tracks.
 *
 * Non-responsibilities:
 *  - Non-UI orchestration and tracking algorithms (owned by AppController/TrackingManager).
 *  - Thread management and long-running workers.
 *
 * Notes:
 *  - Lives on the GUI thread; holds a pointer to AppController.
 *  - Prefer AppController::showTrackingDialog(...) to construct/wire progress dialogs.
 */
#include "mainwindow.h"
#include "../utils/loggingcategories.h"
#include <QTimer>
#include <QShortcut>
#include "debugutils.h"
#include "ui_mainwindow.h"
#include "miniloader.h"
#include "annotationtablemodel.h"
#include "blobtablemodel.h"
#include "itemtypefilterproxymodel.h"
#include "colordelegate.h"
#include "itemtypedelegate.h"
// #include "retrackingdialog.h" // Deprecated: retracking dialog removed; references commented out to allow build
#include "analysisdialog.h"
#include "trackingprogressdialog.h"
#include "../core/appcontroller.h"
#include "../core/centerlineworker.h"
#include "../debug/debugexporter.h"
#include "trackingmanager.h"
#include "trackingdatastorage.h"
#include "version.h"
#include "wormtimeline.h"
#include "../utils/thresholdingutils.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// No need to include videoloader.h again if it's in mainwindow.h, but good practice for .cpp
// #include "videoloader.h"
// #include "trackingcommon.h"
// MergeViewer removed from MainWindow implementation

#include <QStandardPaths>
#include <QFileDialog>
#include <QIcon>
#include <QMessageBox>
#include <QDebug>
#include <QApplication>
#include <QDoubleSpinBox>
#include <QButtonGroup> // For m_interactionModeButtonGroup
#include <QSet>
#include <QGraphicsOpacityEffect>
#include <QAction>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMenu>
#include <QMenuBar>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_blobTableModel(nullptr)
    , m_colorDelegate(nullptr)
    , m_itemTypeDelegate(nullptr)
    , m_appController(nullptr)
    , m_interactionModeButtonGroup(new QButtonGroup(this))
    , m_trackingDataStorage(nullptr)
    , m_hasCompletedTracking(false)
    , roiFactorSpinBoxD(1.5) // Initialize ROI factor to default value 1.5
{
    ui->setupUi(this);
    ui->pointButton->setText("SE");
    ui->pointButton->setToolTip("Select Start/End points");
    ui->pointButton->setStatusTip("Click once for Start, then once for End");
    ui->pointButton->setCheckable(true);
    ui->pointButton->setAutoRaise(true);
    ui->pointButton->setIcon(QIcon());
    ui->pointButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    ui->pointButton->setMinimumWidth(36);
    ui->skeletonButton->setText("CL");
    ui->skeletonButton->setToolTip("Show/Hide centerline");
    ui->skeletonButton->setStatusTip("Show/hide skeletonized centerline overlay");
    ui->skeletonButton->setCheckable(true);
    ui->skeletonButton->setAutoRaise(true);
    ui->skeletonButton->setIcon(QIcon());
    ui->skeletonButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    ui->skeletonButton->setMinimumWidth(36);

    if (ui->tabWidget && ui->cleanupTab) {
        const int debugTabIndex = ui->tabWidget->indexOf(ui->cleanupTab);
        if (debugTabIndex >= 0) {
            ui->tabWidget->setTabVisible(debugTabIndex, DebugUtils::isDebugCaptureEnabled());
        }
    }

    // Initialize AppController which owns storage, manager and models.
    m_appController = new AppController(this);
    m_appController->setPixelSizePixelsPerUm(ui->pixelSizeSpinBoxD->value());

    // Obtain models and storage from the controller (controller manages lifetimes).
    m_blobTableModel = m_appController->blobTableModel();

    m_wormProxyModel = new ItemTypeFilterProxyModel(this);
    m_wormProxyModel->setSourceModel(m_blobTableModel);
    m_wormProxyModel->setAllowedTypes(QSet<TableItems::ItemType>{
        TableItems::ItemType::Worm,
        TableItems::ItemType::Fix
    });
    ui->wormTableView->setModel(m_wormProxyModel);

    m_roiProxyModel = new ItemTypeFilterProxyModel(this);
    m_roiProxyModel->setSourceModel(m_blobTableModel);
    m_roiProxyModel->setAllowedTypes(QSet<TableItems::ItemType>{
        TableItems::ItemType::ROI,
        TableItems::ItemType::StartPoint,
        TableItems::ItemType::EndPoint,
        TableItems::ItemType::ControlPoint,
        TableItems::ItemType::CenterPoint
    });
    ui->roiTableView->setModel(m_roiProxyModel);

    m_annotationTableModel = m_appController->annotationTableModel();
    // ui->annoTableView->setModel(m_annotationTableModel);

    m_trackingDataStorage = m_appController->trackingDataStorage();
    connect(m_trackingDataStorage, &TrackingDataStorage::itemsChanged,
            this, [this](const QList<TableItems::ClickedItem>&) { updateWormTimeline(); });

    // Set up MiniLoader instances
    ui->miniLoader->setTrackingDataStorage(m_trackingDataStorage);
    ui->miniLoader->setShowOverlays(false);

    if (ui->miniLoaderOverlay) {
        ui->miniLoaderOverlay->setTrackingDataStorage(m_trackingDataStorage);
        ui->miniLoaderOverlay->setShowOverlays(true);
    }
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

    YAWT_INFO(lcGuiMainWindow) << "AppController created and models bound";

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

    if (ui->wormTimeline) {
        connect(ui->wormTimeline, &WormTimeline::eventClicked,
                this, [this](int frame, const QList<int>& wormIds) {
                    seekFrame(frame);
                    if (!wormIds.isEmpty() && m_wormProxyModel) {
                        int chosenId = *std::min_element(wormIds.begin(), wormIds.end());
                        const int idCol = static_cast<int>(BlobTableModel::Column::ID);
                        for (int row = 0; row < m_wormProxyModel->rowCount(); ++row) {
                            QModelIndex idx = m_wormProxyModel->index(row, idCol);
                            if (m_wormProxyModel->data(idx, Qt::DisplayRole).toInt() == chosenId) {
                                ui->wormTableView->selectRow(row);
                                break;
                            }
                        }
                    }
                });
        connect(ui->wormTimeline, &WormTimeline::frameScrubbed,
                this, [this](int frame) { seekFrame(frame); });
    }

    // Jump to frame when a merge/split row is clicked
    connect(ui->mergeSplitTableView, &QTableView::clicked, this, [this](const QModelIndex &index){
        if (!index.isValid()) return;
        // Determine currently selected worm in the worm table (if any) so overlay shows correct worm
        int selectedWormId = -1;
        if (ui->wormTableView->selectionModel() && !ui->wormTableView->selectionModel()->selectedIndexes().isEmpty()
            && m_wormProxyModel) {
            int selRow = ui->wormTableView->selectionModel()->selectedIndexes().first().row();
            QModelIndex proxyIdx = m_wormProxyModel->index(selRow, 0);
            QModelIndex srcIdx = m_wormProxyModel->mapToSource(proxyIdx);
            if (srcIdx.isValid()) selectedWormId = m_blobTableModel->getItem(srcIdx.row()).id;
        }



        // Frame number is in column 0
        QModelIndex frameIdx = m_mergeSplitModel->index(index.row(), 0);
        QVariant v = m_mergeSplitModel->data(frameIdx, Qt::DisplayRole);
        bool ok = false;
        int frame = v.toInt(&ok);
        if (ok) seekFrame(frame);
    });

    m_itemTypeDelegate = new ItemTypeDelegate(this);
    ui->wormTableView->setItemDelegateForColumn(BlobTableModel::Column::Type, m_itemTypeDelegate);
    ui->roiTableView->setItemDelegateForColumn(BlobTableModel::Column::Type, m_itemTypeDelegate);

    m_colorDelegate = new ColorDelegate(this);
    ui->wormTableView->setItemDelegateForColumn(BlobTableModel::Column::Color, m_colorDelegate);
    ui->roiTableView->setItemDelegateForColumn(BlobTableModel::Column::Color, m_colorDelegate);

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
    ui->wormTableView->setEditTriggers(QAbstractItemView::DoubleClicked |
                                       QAbstractItemView::SelectedClicked |
                                       QAbstractItemView::EditKeyPressed);

    // Configure Show/Hide column with checkboxes
    ui->wormTableView->setItemDelegateForColumn(BlobTableModel::Column::Show, nullptr); // Use default delegate for checkboxes
    // Allow checking checkboxes in the header
    ui->wormTableView->horizontalHeader()->setSectionsClickable(true);
    ui->wormTableView->horizontalHeader()->setSectionResizeMode(BlobTableModel::Column::Show, QHeaderView::ResizeToContents);

    ui->roiTableView->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    ui->roiTableView->horizontalHeader()->setStretchLastSection(false);
    ui->roiTableView->horizontalHeader()->setVisible(true);
    ui->roiTableView->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    ui->roiTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->roiTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->roiTableView->setEditTriggers(QAbstractItemView::DoubleClicked |
                                      QAbstractItemView::SelectedClicked |
                                      QAbstractItemView::EditKeyPressed);
    ui->roiTableView->setItemDelegateForColumn(BlobTableModel::Column::Show, nullptr);
    ui->roiTableView->horizontalHeader()->setSectionsClickable(true);
    ui->roiTableView->horizontalHeader()->setSectionResizeMode(BlobTableModel::Column::Show, QHeaderView::ResizeToContents);
    ui->roiTableView->setColumnHidden(BlobTableModel::Column::Frame, true);

    // Configure annotation table view
    // ui->annoTableView->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    // ui->annoTableView->horizontalHeader()->setStretchLastSection(false);
    // ui->annoTableView->horizontalHeader()->setVisible(true);
    // ui->annoTableView->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    // ui->annoTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    // ui->annoTableView->setSelectionMode(QAbstractItemView::SingleSelection);

    // Set column resize modes for annotation table
    // ui->annoTableView->horizontalHeader()->setSectionResizeMode(AnnotationTableModel::ID, QHeaderView::ResizeToContents);
    // ui->annoTableView->horizontalHeader()->setSectionResizeMode(AnnotationTableModel::Type, QHeaderView::ResizeToContents);
    // ui->annoTableView->horizontalHeader()->setSectionResizeMode(AnnotationTableModel::Frames, QHeaderView::Stretch);

    // Add hover effects and cursor styling to indicate clickability
    // ui->annoTableView->setMouseTracking(true);
    // ui->annoTableView->viewport()->setCursor(Qt::PointingHandCursor);
    // ui->annoTableView->setStyleSheet(
    //     "QTableView::item:hover { "
    //     "    background-color: #e3f2fd; "
    //     "    border: 1px solid #2196f3; "
    //     "} "
    //     "QTableView::item:selected { "
    //     "    background-color: #bbdefb; "
    //     "    color: #000; "
    //     "}"
    // );

    // YAWT_DEBUG(lcGuiMainWindow) << "Annotation table view configured successfully";

    resizeTableColumns();

    // TrackingManager is now owned by AppController; MainWindow must not create it.

    // Pass data storage to VideoLoader
    ui->videoLoader->setTrackingDataStorage(m_trackingDataStorage);

    setupInteractionModeButtonGroup(); // Setup for exclusive interaction mode buttons
    setupPlaybackSpeedComboBox(); // Initialize playback speed options
    setupConnections();
    initializeUIStates();

    QMenu* fileMenu = menuBar()->addMenu("File");
    QAction* loadRunAction = new QAction("Load Run...", this);
    fileMenu->addAction(loadRunAction);
    connect(loadRunAction, &QAction::triggered, this, &MainWindow::loadRunFromDirectory);

    // Set initial modes in VideoLoader (after connections are set up)
    // Interaction mode buttons will be synced by syncInteractionModeButtons via the signal
    ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::PanZoom);

    // View mode buttons will be synced by syncViewModeOptionButtons via the signal
    // Set initial active view modes in VideoLoader if desired, e.g., show Blobs by default
    ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Blobs, true);
    // ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::None, true); // Or start with nothing

    YAWT_DEBUG(lcGuiMainWindow) << "Setting to global" << ui->globalThreshAutoCheck->checkState();
    ui->statusbar->showMessage(QString("Welcome to YAWT version ")+QString(PROJECT_VERSION)+" !", 10000);
}

/**
 * @brief Destructor. Ensures UI-owned resources are cleaned up and QObject ownership is respected.
 *
 * This destructor should not manage non-UI resources (e.g., TrackingManager threads),
 * which are owned and orchestrated by AppController.
 */
MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setPlayButtonsState(bool playing, bool blockSignals) {
    // Helper: set playback state and keep the play button consistent.
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
    m_interactionModeButtonGroup->addButton(ui->pointButton);
    m_interactionModeButtonGroup->addButton(ui->roiModeButton);
    m_interactionModeButtonGroup->addButton(ui->cropModeButton);
    m_interactionModeButtonGroup->addButton(ui->selectionModeButton);
    //m_interactionModeButtonGroup->addButton(ui->trackModeButton);
    m_interactionModeButtonGroup->setExclusive(true);
    // No QButtonGroup for view modes as they are independent toggles now
}


// Override resize event to handle table column resizing
/**
 * @brief Handle window resize to keep views/layouts in sync.
 * @param event Resize event carrying new size; used to adjust table columns and viewports.
 *
 * This function should remain lightweight and avoid heavy computations.
 */
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
    // Connect ROI factor spinbox to BlobTableModel
    connect(ui->roiFactorSpinBoxD, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_blobTableModel, &BlobTableModel::updateRoiSizeMultiplier);
    connect(ui->pixelSizeSpinBoxD, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_appController, &AppController::setPixelSizePixelsPerUm);

    // File/Directory
    connect(ui->selectDirButton, &QToolButton::clicked, this, &MainWindow::chooseWorkingDirectory);

    // VideoLoader basic signals
    connect(ui->videoLoader, &VideoLoader::videoLoaded, this, &MainWindow::initiateFrameDisplay);
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, &MainWindow::updateFrameDisplay);
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, &MainWindow::updateMiniLoaderCrop);
    connect(ui->videoLoader, &VideoLoader::interactionModeChanged, this, &MainWindow::syncInteractionModeButtons);
    connect(ui->videoLoader, &VideoLoader::activeViewModesChanged, this, &MainWindow::syncViewModeOptionButtons); // Updated signal
    // When an ROI is drawn in VideoLoader, add it as an ROI item in the BlobTableModel
    connect(ui->videoLoader, &VideoLoader::roiDefined, this, &MainWindow::handleRoiDefined);
    connect(ui->videoLoader, &VideoLoader::pointDefined, this, &MainWindow::handlePointDefined);
    connect(ui->videoLoader, &VideoLoader::playbackStateChanged, this, &MainWindow::onPlaybackStateChanged);

    // Playback controls
    connect(ui->playPauseButton, &QToolButton::toggled, this, [this](bool checked) { setPlayButtonsState(checked); });
    connect(ui->firstFrameButton, &QToolButton::clicked, this, &MainWindow::goToFirstFrame);
    connect(ui->lastFrameButton, &QToolButton::clicked, this, &MainWindow::goToLastFrame);
    connect(ui->framePosition, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::seekFrame);
    connect(ui->frameSlider, &QAbstractSlider::valueChanged, this, &MainWindow::frameSliderMoved);

    // Interaction Mode Buttons -> VideoLoader (via slots that call VideoLoader)
    connect(ui->panModeButton, &QToolButton::clicked, this, &MainWindow::panModeButtonClicked);
    connect(ui->pointButton, &QToolButton::clicked, this, &MainWindow::pointModeButtonClicked);
    connect(ui->roiModeButton, &QToolButton::clicked, this, &MainWindow::roiModeButtonClicked);
    connect(ui->cropModeButton, &QToolButton::clicked, this, &MainWindow::cropModeButtonClicked);
    connect(ui->selectionModeButton, &QToolButton::clicked, this, &MainWindow::editBlobsModeButtonClicked);
    connect(ui->resultsButton, &QToolButton::clicked, this, &MainWindow::resultsButtonClicked);

    // View Mode Option Buttons (Checkable QToolButtons or QCheckBoxes) -> VideoLoader
    // Assuming ui->showThreshButton is checkable
    connect(ui->viewThreshButton, &QToolButton::toggled, this, &MainWindow::onViewThresholdToggled);
    // Add these connections once you have the buttons in your UI:
    connect(ui->viewBlobsButton, &QToolButton::toggled, this, &MainWindow::onViewBlobsToggled);
    connect(ui->viewTracksButton, &QToolButton::toggled, this, &MainWindow::onViewTracksToggled);
    connect(ui->skeletonButton, &QToolButton::toggled, this, &MainWindow::onViewSkeletonsToggled);


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
    // Wire the overlay mini loader to the same model if present
    if (ui->miniLoaderOverlay && ui->miniLoaderOverlay->showOverlays()) {
        // When the blob list changes, instruct overlay to repaint
        connect(m_blobTableModel, &BlobTableModel::itemsChanged, ui->miniLoaderOverlay, qOverload<>(&MiniLoader::update));
        // Item visibility changes should also trigger a repaint
        connect(m_blobTableModel, &BlobTableModel::itemVisibilityChanged, ui->miniLoaderOverlay, qOverload<>(&MiniLoader::update));

        // Allow the overlay to respond to selection changes
        connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
                ui->miniLoaderOverlay, [this](const QItemSelection &selected, const QItemSelection&) {
                    // Build the list of selected ClickedItem(s) and call overlay's handler
                    QList<TableItems::ClickedItem> selItems;
                    QModelIndexList indexes = selected.indexes();
                    if (!indexes.isEmpty() && m_wormProxyModel) {
                        QModelIndex proxyIdx = indexes.first();
                        QModelIndex srcIdx = m_wormProxyModel->mapToSource(proxyIdx);
                        if (srcIdx.isValid()) selItems.append(m_blobTableModel->getItem(srcIdx.row()));
                    }
                    ui->miniLoaderOverlay->onWormSelectionChanged(selItems);
                });
    }
    connect(ui->clearAllButton, &QPushButton::clicked, this, &MainWindow::handleRemoveBlobsClicked);
    connect(ui->deleteButton, &QPushButton::clicked, this, &MainWindow::handleDeleteSelectedBlobClicked);

    // Retracking UI removed - no connections

    // Auto-resize table columns when model data changes
    connect(m_blobTableModel, &BlobTableModel::dataChanged, this, &MainWindow::resizeTableColumns);
    connect(m_blobTableModel, &BlobTableModel::rowsInserted, this, &MainWindow::resizeTableColumns);
    connect(m_blobTableModel, &BlobTableModel::rowsRemoved, this, &MainWindow::resizeTableColumns);

    // Retracking combo removed

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
        if (!m_wormProxyModel) return;
        QModelIndex proxyIdx = selected.indexes().first();
        QModelIndex srcIdx = m_wormProxyModel->mapToSource(proxyIdx);
        if (!srcIdx.isValid()) return;
        const TableItems::ClickedItem& sel = m_blobTableModel->getItem(srcIdx.row());
        // Populate merge/split table for this worm
        m_mergeSplitModel->removeRows(0, m_mergeSplitModel->rowCount());
        // We'll iterate through stored merge history and split resolution maps in TrackingManager via storage/APIs
        // For now, collect merge starts from storage's merge history and track point qualities for splits
        // Iterate frames in ascending order and add the first merged frame before a split
        QList<int> frames = m_trackingDataStorage->getMergeGroupsForFrame(0).isEmpty() ? QList<int>() : QList<int>();
        // Simple (but potentially slow) approach: scan all frames present in merge history
        for (auto it = m_trackingDataStorage->getMergeGroupsForFrame(0).begin(); it != m_trackingDataStorage->getMergeGroupsForFrame(0).end(); ++it) {
            Q_UNUSED(it);
        }
        // Instead, we will query the entire merge history map directly via an accessor - but since none exists, read internal map via getAllTracks as a proxy
        // Practical approach: scan frames from 0..currentFrame and check if this worm appears in merge groups
        // Determine frame range from video loader
        int maxFrame = ui->videoLoader->getTotalFrames();
        // New approach: compare merge-group membership for the selected worm between consecutive frames.
        // Whenever the group's membership changes we emit an event row (Merge / Split / Merge/Split).
        QList<int> prevGroup; // empty means not merged on previous frame
        for (int f = 0; f < maxFrame; ++f) {
            QList<QList<int>> groups = m_trackingDataStorage->getMergeGroupsForFrame(f);
            QList<int> currGroup;
            for (const QList<int>& g : groups) {
                if (g.contains(sel.id)) { currGroup = g; break; }
            }

            // Compare prevGroup and currGroup as sets
            QSet<int> prevSet;
            for (int id : prevGroup) prevSet.insert(id);
            QSet<int> currSet;
            for (int id : currGroup) currSet.insert(id);
            if (prevSet == currSet) {
                prevGroup = currGroup;
                continue; // no change
            }

            // Membership changed - decide event type(s)
            bool prevEmpty = prevSet.isEmpty();
            bool currEmpty = currSet.isEmpty();

            QString eventType;
            QString details;

            if (prevEmpty && !currEmpty) {
                // Newly merged
                eventType = "Merge";
                QSet<int> others = currSet;
                others.remove(sel.id);
                if (others.isEmpty()) details = "Merged (no other ids)";
                else {
                    QStringList ids;
                    for (int id : others) ids << QString::number(id);
                    details = QString("Merged with worm(s) %1").arg(ids.join(", "));
                }
            } else if (!prevEmpty && currEmpty) {
                // Split from previous merged group
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
                // Both non-empty and different: detect additions/removals
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
    auto updateDeleteState = [this]() {
        const bool wormSelected = ui->wormTableView->selectionModel() &&
                                  ui->wormTableView->selectionModel()->hasSelection();
        const bool roiSelected = ui->roiTableView->selectionModel() &&
                                 ui->roiTableView->selectionModel()->hasSelection();
        ui->deleteButton->setEnabled(wormSelected || roiSelected);
    };
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [updateDeleteState](const QItemSelection&, const QItemSelection&) {
        updateDeleteState();
    });
    connect(ui->roiTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [updateDeleteState](const QItemSelection&, const QItemSelection&) {
        updateDeleteState();
    });

    // Table View Selection -> MiniLoader Overlays
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [this](const QItemSelection &selected, const QItemSelection &deselected) {
        Q_UNUSED(deselected)

        if (selected.isEmpty()) {
            // No selection - clear the mini loaders
            if (ui->miniLoaderOverlay && ui->miniLoaderOverlay->showOverlays()) ui->miniLoaderOverlay->clearSelection();
        } else {
            // Get the first selected row and extract the worm ID
            QModelIndexList selectedIndexes = selected.indexes();
            if (!selectedIndexes.isEmpty() && m_wormProxyModel) {
                QModelIndex proxyIdx = selectedIndexes.first();
                QModelIndex srcIdx = m_wormProxyModel->mapToSource(proxyIdx);
                if (srcIdx.isValid()) {
                    const TableItems::ClickedItem& selectedItem = m_blobTableModel->getItem(srcIdx.row());
                    if (ui->miniLoaderOverlay && ui->miniLoaderOverlay->showOverlays()) ui->miniLoaderOverlay->setSelectedWorm(selectedItem.id);

                    // Auto-center video on worm position if zoomed in
                    double currentZoom = ui->videoLoader->getZoomFactor();
                    qDebug() << "MainWindow: Blob selection - zoom factor:" << currentZoom << "worm ID:" << selectedItem.id;
                    if (currentZoom > 1.5) {
                        qDebug() << "MainWindow: Zoom factor > 1.5, attempting auto-center for worm" << selectedItem.id;
                        int currentFrame = ui->videoLoader->getCurrentFrameNumber();
                        QPointF wormPosition;
                        QRectF wormRoi;
                        bool centered = false;
                        QString statusMessage;



                        // Try to get worm position for current frame
                        qDebug() << "MainWindow: Trying to get worm data for frame" << currentFrame << "worm ID" << selectedItem.id;
                        if (m_trackingDataStorage->getWormDataForFrame(selectedItem.id, currentFrame, wormPosition, wormRoi)) {
                            qDebug() << "MainWindow: Found worm data - position:" << wormPosition << "ROI:" << wormRoi;
                            if (!wormPosition.isNull() && wormPosition.x() >= 0 && wormPosition.y() >= 0) {
                                qDebug() << "MainWindow: Calling centerOnVideoPoint with position" << wormPosition;
                                ui->videoLoader->centerOnVideoPoint(wormPosition);
                                statusMessage = QString("Centered on Worm %1 at frame %2 (zoom: %3x)")
                                               .arg(selectedItem.id).arg(currentFrame).arg(currentZoom, 0, 'f', 1);
                                centered = true;
                                qDebug() << "MainWindow: Centered video on worm" << selectedItem.id << "at tracked position" << wormPosition;
                            } else {
                                qDebug() << "MainWindow: Worm position invalid:" << wormPosition;
                            }
                        }

                        if (!centered) {
                            // Try to get the last known position before this frame (for lost tracking)
                            qDebug() << "MainWindow: Trying to get last known position before frame" << currentFrame << "for worm" << selectedItem.id;
                            if (m_trackingDataStorage->getLastKnownPositionBefore(selectedItem.id, currentFrame, wormPosition, wormRoi)) {
                                qDebug() << "MainWindow: Found last known position:" << wormPosition << "ROI:" << wormRoi;
                                if (!wormPosition.isNull() && wormPosition.x() >= 0 && wormPosition.y() >= 0) {
                                    qDebug() << "MainWindow: Calling centerOnVideoPoint with last known position" << wormPosition;
                                    ui->videoLoader->centerOnVideoPoint(wormPosition);
                                    statusMessage = QString("Centered on Worm %1 last known position (zoom: %2x)")
                                                   .arg(selectedItem.id).arg(currentZoom, 0, 'f', 1);
                                    centered = true;
                                    qDebug() << "MainWindow: Centered video on worm" << selectedItem.id << "at last known position" << wormPosition;
                                } else {
                                    qDebug() << "MainWindow: Last known position invalid:" << wormPosition;
                                }
                            }
                        }

                        if (!centered) {
                            // If no tracking data, use initial position from blob as final fallback
                            QPointF initialPos = selectedItem.initialCentroid;
                            qDebug() << "MainWindow: No last known position found, using initial position:" << initialPos;
                            if (!initialPos.isNull() && initialPos.x() >= 0 && initialPos.y() >= 0) {
                                qDebug() << "MainWindow: Calling centerOnVideoPoint with initial position" << initialPos;
                                ui->videoLoader->centerOnVideoPoint(initialPos);
                                statusMessage = QString("Centered on Worm %1 initial position (zoom: %2x)")
                                               .arg(selectedItem.id).arg(currentZoom, 0, 'f', 1);
                                centered = true;
                                qDebug() << "MainWindow: Centered video on worm" << selectedItem.id << "initial position" << initialPos;
                            } else {
                                qDebug() << "MainWindow: Initial position also invalid:" << initialPos;
                            }
                        }

                        if (centered) {
                            statusBar()->showMessage(statusMessage, 3000);
                        } else {
                            qDebug() << "MainWindow: Auto-center failed - no valid position found";
                        }
                    } else {
                        qDebug() << "MainWindow: Zoom factor too low for auto-center:" << currentZoom;
                    }
                }
            }
        }
    });





    // Table View Selection -> MiniLoader (update crop when selection changes)
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [this](const QItemSelection &selected, const QItemSelection &deselected) {
        Q_UNUSED(selected)
        Q_UNUSED(deselected)
        // When selection changes, update the miniLoader crop with current frame
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

    // Update combobox when VideoLoader speed changes (optional - for consistency)
    connect(ui->videoLoader, &VideoLoader::playbackSpeedChanged,
            this, &MainWindow::updatePlaybackSpeedComboBox);

    auto toggleVisibilityForTypes = [this](const QSet<TableItems::ItemType>& types) {
        if (!m_trackingDataStorage) return;
        int total = 0;
        int visibleCount = 0;
        const QList<TableItems::ClickedItem>& items = m_trackingDataStorage->getAllItems();
        for (const auto& item : items) {
            if (!types.contains(item.type)) continue;
            total++;
            if (item.visible) visibleCount++;
        }
        if (total == 0) return;
        const bool setVisible = (visibleCount == 0);
        for (const auto& item : items) {
            if (types.contains(item.type)) {
                m_trackingDataStorage->setItemVisibility(item.id, setVisible);
            }
        }
    };

    connect(ui->wormTableView->horizontalHeader(), &QHeaderView::sectionClicked,
            [this, toggleVisibilityForTypes](int logicalIndex) {
        if (logicalIndex != BlobTableModel::Column::Show) return;
        toggleVisibilityForTypes(QSet<TableItems::ItemType>{
            TableItems::ItemType::Worm,
            TableItems::ItemType::Fix
        });
    });

    connect(ui->roiTableView->horizontalHeader(), &QHeaderView::sectionClicked,
            [this, toggleVisibilityForTypes](int logicalIndex) {
        if (logicalIndex != BlobTableModel::Column::Show) return;
        toggleVisibilityForTypes(QSet<TableItems::ItemType>{
            TableItems::ItemType::ROI,
            TableItems::ItemType::StartPoint,
            TableItems::ItemType::EndPoint,
            TableItems::ItemType::ControlPoint
        });
    });

    // Video File Tree View -> VideoLoader
    connect(ui->videoTreeView, &VideoFileTreeView::videoFileDoubleClicked, ui->videoLoader, &VideoLoader::loadVideo);
    connect(ui->videoTreeView, &VideoFileTreeView::runDirectoryDoubleClicked, this, &MainWindow::loadRunFromDirectoryPath);

    // Tracking Process
    connect(ui->trackingDialogButton, &QPushButton::clicked, this, &MainWindow::onStartTrackingActionTriggered);
    if (m_appController) {
        connect(m_appController, &AppController::tracksUpdated, this, &MainWindow::acceptTracksFromManager);
        connect(m_appController, &AppController::trackingFinished, this, &MainWindow::updateWormTimeline);
    }

    // Debug tab — Rerun Centerline button
    connect(ui->rerunCenterlineButton, &QPushButton::clicked,
            this, &MainWindow::onRerunCenterlineClicked);
    connect(ui->exportProcessButton, &QPushButton::clicked,
            this, &MainWindow::onExportProcessClicked);
    if (m_appController) {
        connect(m_appController, &AppController::centerlineProgress,
                this, &MainWindow::onDebugCenterlineProgress);
        connect(m_appController, &AppController::centerlineFinished,
                this, &MainWindow::onDebugCenterlineFinished);
    }

    // Debug tab — Show-tip-candidates overlay toggle. Pure render-time switch;
    // the underlying tip data is computed unconditionally on every centerline
    // pass and stashed on each DetectedBlob, so toggling this only changes
    // what's drawn, not what's computed.
    if (auto* tipChk = ui->showTipCandidatesCheck) {
        connect(tipChk, &QCheckBox::toggled, this, [this](bool checked) {
            ui->videoLoader->setViewModeOption(
                VideoLoader::ViewModeOption::TipCandidates, checked);
            ui->videoLoader->update();
        });
    }

    // Annotation table selection
    // connect(ui->annoTableView, &QTableView::clicked, this, &MainWindow::onAnnotationTableClicked);

    // Connect header data changes to trigger UI update
    connect(m_blobTableModel, &QAbstractItemModel::headerDataChanged,
            this, [this](Qt::Orientation orientation, int first, int last) {
        if (orientation == Qt::Horizontal && first <= BlobTableModel::Column::Show && last >= BlobTableModel::Column::Show) {
            // Update the table view when header checkbox state changes
            ui->wormTableView->update();
            ui->roiTableView->update();
        }
    });

    // Initial call to setVisibleTrackIDs with all item IDs
    QSet<int> initialItemIDs;
    for (const auto& item : m_blobTableModel->getAllItems()) {
        if (item.type == TableItems::ItemType::Worm || item.type == TableItems::ItemType::Fix) {
            initialItemIDs.insert(item.id);
        }
    }
    ui->videoLoader->setVisibleTrackIDs(initialItemIDs);
    if (ui->miniLoaderOverlay) ui->miniLoaderOverlay->update();

    // Connect MiniLoader visible worms signal to MainWindow so we can filter merge history live.
    // Also connect the overlay instance if present so we receive visible updates from either widget.
    // Always check ui->miniLoader presence since some UI permutations may not include it.
    if (ui->miniLoader) {
        // Primary miniLoader is display-only (zoom view). Per the new flow it should NOT be
        // connected to visibility signals or used as the source of truth for visibility.
        qDebug() << "MainWindow: primary miniLoader present but will NOT be connected to visibility signals (display-only).";
    }
    // If an overlay widget exists, connect its visibleWormsUpdated signal as well so we don't miss updates.
    if (ui->miniLoaderOverlay) {
        QMetaObject::Connection c2 = connect(ui->miniLoaderOverlay, &MiniLoader::visibleWormsUpdated,
                this, &MainWindow::onMiniLoaderVisibleWormsUpdated);
        qDebug() << "MainWindow: connect miniLoaderOverlay visibleWormsUpdated -> slot, overlay ptr=" << ui->miniLoaderOverlay
                 << " connection valid:" << bool(c2);

        // Mirror-to-MergeViewer functionality removed.
        // Previously we forwarded MiniLoader overlay per-frame visibility maps to a MergeViewer widget.
        // The MergeViewer has been removed from the UI and related forwarding logic is intentionally omitted.
    }

    // Start a short polling timer as a fallback in case signals are missed or paint() hasn't yet emitted.
    // This keeps the mergeHistoryText in sync with miniLoader(s) reliably.
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
    ui->frameSlider->setMinimum(0);
    ui->frameSlider->setSingleStep(10);
    ui->frameSlider->setPageStep(100);
    // Initialize both play buttons to the "play" state and icon without emitting signals
    setPlayButtonsState(false, /*blockSignals=*/false);

    // Initialize delete button to disabled state since there are no items selected initially
    ui->deleteButton->setEnabled(false);

    // Initialize Clear Fix button to disabled state until tracking is completed

    // Retrack controls removed from UI

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
    YAWT_DEBUG(lcGuiMainWindow) << "Setting to global" << ui->globalThreshAutoCheck->checkState();
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

    auto resizeView = [this](QTableView* view) {
        if (!view) return;
        int viewportWidth = view->viewport()->width();
        int columnCount = m_blobTableModel->columnCount();

        view->horizontalHeader()->setVisible(true);
        view->horizontalHeader()->setStretchLastSection(false);
        view->horizontalHeader()->setMinimumSectionSize(10);

        int totalContentWidth = 0;
        QVector<int> contentWidths(columnCount);

        for (int i = 0; i < columnCount; ++i) {
            view->resizeColumnToContents(i);
            contentWidths[i] = view->horizontalHeader()->sectionSize(i);
            totalContentWidth += contentWidths[i];
        }

        if (viewportWidth > totalContentWidth && totalContentWidth > 0) {
            float expansionRatio = static_cast<float>(viewportWidth) / totalContentWidth;
            for (int i = 0; i < columnCount; ++i) {
                int newWidth = qRound(contentWidths[i] * expansionRatio);
                view->horizontalHeader()->setSectionResizeMode(i, QHeaderView::Interactive);
                view->horizontalHeader()->resizeSection(i, newWidth);
            }
        } else {
            for (int i = 0; i < columnCount; ++i) {
                view->horizontalHeader()->setSectionResizeMode(i, QHeaderView::Interactive);
            }
            view->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        }
    };

    resizeView(ui->wormTableView);
    resizeView(ui->roiTableView);
}

void MainWindow::updateWormTimeline()
{
    if (!ui->wormTimeline || !m_blobTableModel || !m_trackingDataStorage) return;

    QMap<int, QColor> idColors;
    const QList<TableItems::ClickedItem> items = m_blobTableModel->getAllItems();
    for (const auto& item : items) {
        if (item.type == TableItems::ItemType::Worm || item.type == TableItems::ItemType::Fix) {
            idColors.insert(item.id, item.color);
        }
    }

    int totalFrames = 0;
    if (ui->videoLoader) totalFrames = ui->videoLoader->getTotalFrames();

    ui->wormTimeline->setTotalFrames(totalFrames);
    ui->wormTimeline->setKeyframeFrame(0);
    ui->wormTimeline->setWormColors(idColors);
    ui->wormTimeline->setMergeGroupsByFrame(m_trackingDataStorage->getAllMergeGroups());
}


// --- Mode Toggling Slots ---
void MainWindow::panModeButtonClicked() {
    ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::PanZoom);
}
void MainWindow::pointModeButtonClicked() {
    m_startEndSelectionActive = true;
    m_nextStartEndPointType = TableItems::ItemType::StartPoint;

    if (!ui->videoLoader->getActiveViewModes().testFlag(VideoLoader::ViewModeOption::Blobs)) {
        ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Blobs, true);
    }

    ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::Point);
    statusBar()->showMessage("Start/End Mode: click the Start point, then the End point", 5000);
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
    // Assuming you have a ui->viewBlobsButton that is checkable
    ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Blobs, checked);
}
void MainWindow::onViewTracksToggled(bool checked) {
    // Assuming you have a ui->viewTracksButton that is checkable
    ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Tracks, checked);
}
void MainWindow::onViewSkeletonsToggled(bool checked) {
    ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Skeletons, checked);
}
// Optional:
// void MainWindow::onViewNoneClicked() {
//     ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Threshold, false);
//     ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Blobs, false);
//     ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Tracks, false);
// }


// --- Slots to sync UI buttons with VideoLoader's state ---
void MainWindow::syncInteractionModeButtons(VideoLoader::InteractionMode newMode) {
    if (newMode != VideoLoader::InteractionMode::Point) {
        m_startEndSelectionActive = false;
        m_nextStartEndPointType = TableItems::ItemType::StartPoint;
    }

    // This will be handled by QButtonGroup if buttons are added to it correctly
    // Or, if not using QButtonGroup for these specific buttons for some reason:
    ui->panModeButton->setChecked(newMode == VideoLoader::InteractionMode::PanZoom);
    ui->pointButton->setChecked(newMode == VideoLoader::InteractionMode::Point);
    ui->roiModeButton->setChecked(newMode == VideoLoader::InteractionMode::DrawROI);
    ui->cropModeButton->setChecked(newMode == VideoLoader::InteractionMode::Crop);
    ui->selectionModeButton->setChecked(newMode == VideoLoader::InteractionMode::EditBlobs);
    //ui->trackModeButton->setChecked(newMode == VideoLoader::InteractionMode::EditTracks);
    YAWT_DEBUG(lcGuiMainWindow) << "Interaction mode UI synced to" << static_cast<int>(newMode);
}

void MainWindow::syncViewModeOptionButtons(VideoLoader::ViewModeOptions newModes) {
    // Update the checked state of your independent view mode buttons
    ui->viewThreshButton->setChecked(newModes.testFlag(VideoLoader::ViewModeOption::Threshold));
    // Assuming you have ui->viewBlobsButton and ui->viewTracksButton:
    ui->viewBlobsButton->setChecked(newModes.testFlag(VideoLoader::ViewModeOption::Blobs));
    ui->viewTracksButton->setChecked(newModes.testFlag(VideoLoader::ViewModeOption::Tracks));
    ui->skeletonButton->setChecked(newModes.testFlag(VideoLoader::ViewModeOption::Skeletons));
    YAWT_DEBUG(lcGuiMainWindow) << "View mode UI synced. Flags:" << QString::number(static_cast<int>(newModes), 16);

    const bool showOverlays = newModes.testFlag(VideoLoader::ViewModeOption::Blobs);
    if (ui->miniLoader) ui->miniLoader->setShowOverlays(showOverlays);
    if (ui->miniLoaderOverlay) ui->miniLoaderOverlay->setShowOverlays(showOverlays);
    if (ui->mlon2) ui->mlon2->setShowOverlays(showOverlays);
    if (ui->mlon1) ui->mlon1->setShowOverlays(showOverlays);
    if (ui->mlop1) ui->mlop1->setShowOverlays(showOverlays);
    if (ui->mlop2) ui->mlop2->setShowOverlays(showOverlays);

    if (ui->videoLoader && ui->videoLoader->isVideoLoaded()) {
        int currentFrame = ui->videoLoader->getCurrentFrameNumber();
        QImage currentImage = ui->videoLoader->getCurrentQImageFrame();
        if (!currentImage.isNull()) {
            updateMiniLoaderCrop(currentFrame, currentImage);
        }
    }
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

    // Determine item type based on tracking state
    // Legacy behavior previously turned newly added items into 'Fix' blobs after tracking completed.
    // That behavior is disabled: always add items as regular Worm blobs.
    TableItems::ItemType itemType = TableItems::ItemType::Worm;

    // Now adding through the data storage via the model
    bool added = m_blobTableModel->addItem(blobData.centroid, blobData.boundingBox, currentFrame, itemType);

    if (added) {
        // Enable the delete button since we now have an item
        ui->deleteButton->setEnabled(true);

        // Select the newly added row
        int lastRow = m_blobTableModel->rowCount() - 1;
        if (m_wormProxyModel) {
            QModelIndex srcIndex = m_blobTableModel->index(lastRow, 0);
            QModelIndex proxyIndex = m_wormProxyModel->mapFromSource(srcIndex);
            if (proxyIndex.isValid()) {
                ui->wormTableView->setCurrentIndex(proxyIndex);
                ui->wormTableView->selectionModel()->select(proxyIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
            }
        }

        // Resize columns to fit the new content
        resizeTableColumns();

        // Provide user feedback based on blob type
        QString feedbackMessage;
        // Always report as a Worm blob since auto-Fix behavior is disabled
        feedbackMessage = QString("Added Worm blob at frame %1").arg(currentFrame);
        statusBar()->showMessage(feedbackMessage, 3000);

    // No retrack combo functionality in this build
    }
}

void MainWindow::handleRoiDefined(const QRectF& roi) {
    if (!ui->videoLoader->isVideoLoaded()) return;
    if (roi.isNull() || roi.isEmpty() || roi.width() <= 0.0 || roi.height() <= 0.0) return;
    int currentFrame = ui->videoLoader->getCurrentFrameNumber();

    // Convert ROI rect to centroid and bounding box in video coords
    QPointF centroid = QPointF(roi.x() + roi.width() / 2.0, roi.y() + roi.height() / 2.0);
    QRectF boundingBox = roi;

    // Add as ROI type to the model
    bool added = m_blobTableModel->addItem(centroid, boundingBox, currentFrame, TableItems::ItemType::ROI);

    if (added) {
        ui->deleteButton->setEnabled(true);
        int lastRow = m_blobTableModel->rowCount() - 1;
        if (m_roiProxyModel) {
            QModelIndex srcIndex = m_blobTableModel->index(lastRow, 0);
            QModelIndex proxyIndex = m_roiProxyModel->mapFromSource(srcIndex);
            if (proxyIndex.isValid()) {
                ui->roiTableView->setCurrentIndex(proxyIndex);
                ui->roiTableView->selectionModel()->select(proxyIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
            }
        }
        resizeTableColumns();
        statusBar()->showMessage(QString("Added ROI at frame %1").arg(currentFrame), 3000);
    }
}

void MainWindow::handlePointDefined(const QPointF& point) {
    if (!ui->videoLoader->isVideoLoaded()) return;
    if (point.x() < 0.0 || point.y() < 0.0) return;
    if (!m_startEndSelectionActive || !m_trackingDataStorage) return;

    int currentFrame = ui->videoLoader->getCurrentFrameNumber();

    const auto removeExistingPointType = [this](TableItems::ItemType type) {
        QList<int> idsToRemove;
        const QList<TableItems::ClickedItem>& items = m_blobTableModel->getAllItems();
        for (const TableItems::ClickedItem& item : items) {
            if (item.type == type) {
                idsToRemove.append(item.id);
            }
        }

        for (int itemId : idsToRemove) {
            m_trackingDataStorage->removeItem(itemId);
        }
    };

    const auto selectPointRow = [this](int itemId) {
        if (!m_roiProxyModel) {
            return;
        }

        const int sourceRow = m_trackingDataStorage->getIndexFromId(itemId);
        if (sourceRow < 0) {
            return;
        }

        const QModelIndex sourceIndex = m_blobTableModel->index(sourceRow, 0);
        const QModelIndex proxyIndex = m_roiProxyModel->mapFromSource(sourceIndex);
        if (!proxyIndex.isValid()) {
            return;
        }

        ui->roiTableView->setCurrentIndex(proxyIndex);
        ui->roiTableView->selectionModel()->select(proxyIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
    };

    const TableItems::ItemType pointType = m_nextStartEndPointType;
    removeExistingPointType(pointType);

    QRectF pointBox(point.x(), point.y(), 0.0, 0.0);
    int newId = m_trackingDataStorage->addItem(point, pointBox, currentFrame, pointType);
    if (newId <= 0) return;

    const QColor pointColor = (pointType == TableItems::ItemType::StartPoint) ? QColor(Qt::green) : QColor(Qt::red);
    m_trackingDataStorage->setItemColor(newId, pointColor);

    ui->deleteButton->setEnabled(true);
    selectPointRow(newId);
    resizeTableColumns();

    if (pointType == TableItems::ItemType::StartPoint) {
        m_nextStartEndPointType = TableItems::ItemType::EndPoint;
        statusBar()->showMessage("Start point set. Click the End point.", 5000);
        return;
    }

    m_startEndSelectionActive = false;
    m_nextStartEndPointType = TableItems::ItemType::StartPoint;
    ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::PanZoom);
    statusBar()->showMessage("Start/End points updated.", 3000);
}

void MainWindow::handleRemoveBlobsClicked() {
    m_blobTableModel->removeRows(0, m_blobTableModel->getAllItems().length());
    ui->deleteButton->setEnabled(false); // Disable delete button after clearing all items
    // VideoLoader will get updates from storage, but keep these for backward compatibility
    ui->videoLoader->updateItemsToDisplay(QList<TableItems::ClickedItem>());
    ui->videoLoader->setVisibleTrackIDs(QSet<int>());
    // Ensure table columns are properly sized after clearing
    resizeTableColumns();
}


void MainWindow::handleDeleteSelectedBlobClicked() {
    QTableView* activeView = nullptr;
    ItemTypeFilterProxyModel* activeProxy = nullptr;

    if (ui->wormTableView->selectionModel() && ui->wormTableView->selectionModel()->hasSelection()) {
        activeView = ui->wormTableView;
        activeProxy = m_wormProxyModel;
    } else if (ui->roiTableView->selectionModel() && ui->roiTableView->selectionModel()->hasSelection()) {
        activeView = ui->roiTableView;
        activeProxy = m_roiProxyModel;
    }

    if (!activeView || !activeProxy) return;

    QModelIndexList selectedIndexes = activeView->selectionModel()->selectedIndexes();
    if (selectedIndexes.isEmpty()) return;

    QModelIndex proxyIndex = selectedIndexes.first();
    QModelIndex srcIndex = activeProxy->mapToSource(proxyIndex);
    if (!srcIndex.isValid()) return;

    const int selectedProxyRow = proxyIndex.row();
    m_blobTableModel->removeRows(srcIndex.row(), 1);

    if (activeProxy->rowCount() > 0) {
        int newRow = (selectedProxyRow < activeProxy->rowCount()) ? selectedProxyRow : activeProxy->rowCount() - 1;
        QModelIndex newIndex = activeProxy->index(newRow, 0);
        activeView->setCurrentIndex(newIndex);
        activeView->selectionModel()->select(newIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
    }

    resizeTableColumns();
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

void MainWindow::loadRunFromDirectory() {
    QString startDir = ui->dirSelected->text();
    if (startDir.isEmpty()) {
        startDir = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    }

    QString selectedDir = QFileDialog::getExistingDirectory(this, "Select run directory", startDir, QFileDialog::ShowDirsOnly);
    if (selectedDir.isEmpty()) {
        return;
    }
    loadRunFromDirectoryInternal(selectedDir);
}

void MainWindow::loadRunFromDirectoryPath(const QString& directoryPath) {
    if (directoryPath.isEmpty()) {
        return;
    }
    loadRunFromDirectoryInternal(directoryPath);
}

bool MainWindow::loadRunFromDirectoryInternal(const QString& selectedDir) {
    if (selectedDir.isEmpty()) {
        return false;
    }

    QDir procDir(selectedDir);
    QString wormsPath = procDir.absoluteFilePath("worms.json");
    QString thresholdPath = procDir.absoluteFilePath("thresholding.json");
    QString roiPath = procDir.absoluteFilePath("roi_points.json");
    QStringList trackFiles = procDir.entryList(QStringList() << "*_tracks.csv" << "*_tracks.xlsx", QDir::Files);

    if (!QFileInfo::exists(wormsPath) || !QFileInfo::exists(thresholdPath) || trackFiles.isEmpty()) {
        QMessageBox::warning(this, "Load Run", "Selected folder is missing required files (worms.json, thresholding.json, and a *_tracks.csv or *_tracks.xlsx export).");
        return false;
    }

    QDir cursor(procDir);
    QString yawtPath;
    while (true) {
        if (cursor.dirName() == "yawt") {
            yawtPath = cursor.absolutePath();
            break;
        }
        if (!cursor.cdUp()) break;
    }
    if (yawtPath.isEmpty()) {
        QMessageBox::warning(this, "Load Run", "Could not locate a parent 'yawt' directory for the selected run.");
        return false;
    }

    QDir yawtDir(yawtPath);
    if (!yawtDir.cdUp()) {
        QMessageBox::warning(this, "Load Run", "Could not locate the video directory above 'yawt'.");
        return false;
    }
    QString videoDirPath = yawtDir.absolutePath();

    QString videoBaseName = QFileInfo(procDir.absolutePath()).dir().dirName();
    QDir videoDir(videoDirPath);
    QString videoPath;
    QStringList videoFiles = videoDir.entryList(QDir::Files | QDir::Readable);
    for (const QString& fileName : videoFiles) {
        QFileInfo fi(videoDir.absoluteFilePath(fileName));
        if (fi.completeBaseName() == videoBaseName) {
            videoPath = fi.absoluteFilePath();
            break;
        }
    }
    if (videoPath.isEmpty()) {
        QMessageBox::warning(this, "Load Run", "Could not find a video matching the run name in the directory above 'yawt'.");
        return false;
    }

    if (ui->videoLoader) {
        ui->videoLoader->pause();
    }
    if (m_trackingDataStorage) {
        m_trackingDataStorage->clearAllData();
    }
    if (ui->videoLoader) {
        ui->videoLoader->updateItemsToDisplay(QList<TableItems::ClickedItem>());
        ui->videoLoader->setTracksToDisplay(Tracking::AllWormTracks());
        ui->videoLoader->setVisibleTrackIDs(QSet<int>());
    }
    m_hasCompletedTracking = false;

    ui->dirSelected->setText(QFileInfo(videoDirPath).absoluteFilePath());
    ui->videoTreeView->setRootDirectory(QFileInfo(videoDirPath).absoluteFilePath());

    if (!ui->videoLoader->loadVideo(videoPath)) {
        QMessageBox::warning(this, "Load Run", "Failed to load the associated video.");
        return false;
    }

    if (!applyThresholdSettingsFromJsonFile(thresholdPath)) {
        QMessageBox::warning(this, "Load Run", "Failed to load thresholding settings.");
        return false;
    }

    if (!m_trackingDataStorage || !m_trackingDataStorage->loadFromWormsJson(wormsPath)) {
        QMessageBox::warning(this, "Load Run", "Failed to load worms.json.");
        return false;
    }
    if (QFileInfo::exists(roiPath)) {
        if (!m_trackingDataStorage->loadFromRoiJson(roiPath)) {
            QMessageBox::warning(this, "Load Run", "Failed to load roi_points.json.");
            return false;
        }
    }

    ui->videoLoader->updateItemsToDisplay(m_trackingDataStorage->getAllItems());
    ui->videoLoader->setTracksToDisplay(m_trackingDataStorage->getAllTracks());
    ui->videoLoader->setVisibleTrackIDs(m_trackingDataStorage->getAllItemIds());

    resizeTableColumns();
    updateWormTimeline();
    return true;
}

void MainWindow::initiateFrameDisplay(const QString& filePath, int totalFrames, double fps, QSize frameSize) {
    ui->frameSlider->setMaximum(totalFrames > 0 ? totalFrames - 1 : 0);
    ui->frameSlider->setValue(0);
    ui->framePosition->setMaximum(totalFrames > 0 ? totalFrames - 1 : 0);
    ui->framePosition->setValue(0);
    m_videoFps = fps;
    ui->fpsLabel->setText(QString::number(fps, 'f', 2) + " fps");
    ui->videoNameLabel->setText(QFileInfo(filePath).fileName());
    updateWormTimeline();

    if (m_analysisDialog) {
        m_analysisDialog->setVideoFps(m_videoFps);
    }
}

void MainWindow::updateFrameDisplay(int currentFrameNumber, const QImage& currentFrame) {
    Q_UNUSED(currentFrame);
    if (!ui->frameSlider->isSliderDown()) {
        ui->frameSlider->setValue(currentFrameNumber);
    }
    ui->framePosition->setValue(currentFrameNumber);

    if (ui->wormTimeline) {
        ui->wormTimeline->setCurrentFrame(currentFrameNumber);
    }

    if (ui->focusCheckBox->isChecked() && m_trackingDataStorage) {
        QModelIndexList selectedIndexes = ui->wormTableView->selectionModel()->selectedIndexes();
        if (!selectedIndexes.isEmpty() && m_wormProxyModel) {
            QModelIndex srcIdx = m_wormProxyModel->mapToSource(selectedIndexes.first());
            if (srcIdx.isValid()) {
                const TableItems::ClickedItem& selectedItem = m_blobTableModel->getItem(srcIdx.row());
                QPointF wormPosition;
                QRectF wormRoi;
                bool found = m_trackingDataStorage->getWormDataForFrame(selectedItem.id, currentFrameNumber, wormPosition, wormRoi);
                if (!found)
                    found = m_trackingDataStorage->getLastKnownPositionBefore(selectedItem.id, currentFrameNumber, wormPosition, wormRoi);
                if (!found && !selectedItem.initialCentroid.isNull()) {
                    wormPosition = selectedItem.initialCentroid;
                    found = true;
                }
                if (found && wormPosition.x() >= 0 && wormPosition.y() >= 0)
                    ui->videoLoader->centerOnVideoPoint(wormPosition);
            }
        }
    }
}

void MainWindow::updateMiniLoaderCrop(int currentFrameNumber, const QImage& currentFrame) {
    if (!ui->miniLoader || currentFrame.isNull()) {
        return;
    }

    if (!m_isVideoPlaying && ui->videoLoader) {
        if (m_lastMiniLoaderFrame < 0) {
            ui->videoLoader->cacheWindowAroundFrame(currentFrameNumber, 2);
        } else if (currentFrameNumber > m_lastMiniLoaderFrame) {
            if (currentFrameNumber - m_lastMiniLoaderFrame == 1) {
                if (!ui->videoLoader->prefetchNextSequentialFrame()) {
                    ui->videoLoader->cacheWindowAroundFrame(currentFrameNumber, 2);
                }
            } else {
                ui->videoLoader->cacheWindowAroundFrame(currentFrameNumber, 2);
            }
        } else if (currentFrameNumber < m_lastMiniLoaderFrame) {
            ui->videoLoader->cacheWindowAroundFrame(currentFrameNumber, 2);
        }
        m_lastMiniLoaderFrame = currentFrameNumber;
    }

    // Get the crop size from BlobTableModel
    QSizeF cropSize = m_blobTableModel->getCurrentFixedRoiSize();
    if (cropSize.isEmpty()) {
        cropSize = QSizeF(100, 100); // fallback size
    }

    // Determine center point for cropping (use only the central frame to pick center)
    QPointF centerPoint;
    bool foundCenterPoint = false;

    // First priority: if a worm is selected, use its centroid (for the current/central frame)
    QModelIndexList selectedIndexes = ui->wormTableView->selectionModel()->selectedIndexes();
    if (!selectedIndexes.isEmpty() && m_wormProxyModel) {
        QModelIndex proxyIdx = selectedIndexes.first();
        QModelIndex srcIdx = m_wormProxyModel->mapToSource(proxyIdx);
        if (srcIdx.isValid()) {
            const TableItems::ClickedItem& selectedItem = m_blobTableModel->getItem(srcIdx.row());
            int wormId = selectedItem.id;

            QPointF wormPosition;
            QRectF wormRoi;
            if (m_trackingDataStorage->getWormDataForFrame(wormId, currentFrameNumber, wormPosition, wormRoi)) {
                centerPoint = wormPosition;
                foundCenterPoint = true;
            } else if (m_trackingDataStorage->getLastKnownPositionBefore(wormId, currentFrameNumber, wormPosition, wormRoi)) {
                centerPoint = wormPosition;
                foundCenterPoint = true;
            }
        }
    }

    // Second priority: use MiniLoader's last known center point
    if (!foundCenterPoint) {
       const QPointF lastCenter = ui->miniLoader->getLastCenterPoint();
        if (!lastCenter.isNull() && lastCenter.x() >= 0 && lastCenter.y() >= 0) {
            centerPoint = lastCenter;
            foundCenterPoint = true;
        }
    }

    // Third priority: use center of the provided (central) image
    if (!foundCenterPoint) {
        centerPoint = QPointF(currentFrame.width() / 2.0, currentFrame.height() / 2.0);
    }

    // Calculate the crop rectangle (same for all frames we will send)
    double cropWidth = cropSize.width();
    double cropHeight = cropSize.height();
    double left = centerPoint.x() - cropWidth / 2.0;
    double top = centerPoint.y() - cropHeight / 2.0;

    // Clamp to image bounds (use currentFrame dimensions as reference)
    left = qBound(0.0, left, currentFrame.width() - cropWidth);
    top = qBound(0.0, top, currentFrame.height() - cropHeight);

    const QPointF cropOffset(left, top);
    QRect cropRect(static_cast<int>(std::round(left)), static_cast<int>(std::round(top)),
                   static_cast<int>(std::round(cropWidth)), static_cast<int>(std::round(cropHeight)));

    // Intersect with image bounds
    QRect imageBounds(0, 0, currentFrame.width(), currentFrame.height());
    cropRect = cropRect.intersected(imageBounds);
    const QSizeF newSize(cropRect.width(), cropRect.height());

    if (cropRect.isEmpty()) {
        return;
    }

    // Send one cropped frame to each of the five mini loaders instead of batching +/-2 frames to a single overlay.
    // Mapping:
    //   mlon2 -> currentFrameNumber - 2
    //   mlon1 -> currentFrameNumber - 1
    //   miniLoaderOverlay -> currentFrameNumber
    //   mlop1 -> currentFrameNumber + 1
    //   mlop2 -> currentFrameNumber + 2
    {
        int totalFrames = 0;
        if (ui->videoLoader) totalFrames = ui->videoLoader->getTotalFrames();
        const bool thresholdView = ui->videoLoader
            && ui->videoLoader->getActiveViewModes().testFlag(VideoLoader::ViewModeOption::Threshold);
        Thresholding::ThresholdSettings threshSettings;
        if (thresholdView && ui->videoLoader) {
            threshSettings = ui->videoLoader->getCurrentThresholdSettings();
        }

        // Helper lambda to get a cropped frame (real or black placeholder) for a specific absolute frame.
        auto buildCroppedForFrame = [&](int frameNum) -> std::tuple<QImage, QPointF, QSizeF> {
            QRect localCrop = cropRect;
            QImage img;
            if (ui->videoLoader && frameNum >= 0 && (totalFrames == 0 || frameNum < totalFrames)) {
                img = ui->videoLoader->getQImageForFrame(frameNum);
            }
            if (img.isNull() && frameNum == currentFrameNumber) {
                img = currentFrame;
            }

            if (thresholdView && !img.isNull()) {
                QImage bgr = img.convertToFormat(QImage::Format_BGR888);
                cv::Mat inputMat(bgr.height(), bgr.width(), CV_8UC3,
                                 const_cast<uchar*>(bgr.bits()), bgr.bytesPerLine());
                cv::Mat threshMat;
                ThresholdingUtils::applyThresholding(inputMat, threshMat, threshSettings);
                if (!threshMat.empty()) {
                    QImage gray(threshMat.data, threshMat.cols, threshMat.rows,
                                threshMat.step, QImage::Format_Grayscale8);
                    img = gray.copy();
                }
            }

            if (!img.isNull()) {
                QRect imgBounds(0, 0, img.width(), img.height());
                localCrop = localCrop.intersected(imgBounds);
            }
            if (localCrop.isEmpty()) localCrop = cropRect;
            QImage cf;
            if (!img.isNull()) cf = img.copy(localCrop);
            else {
                cf = QImage(localCrop.size(), QImage::Format_RGB32);
                cf.fill(Qt::black);
            }
            QPointF offset(localCrop.left() + 0.0, localCrop.top() + 0.0);
            QSizeF size(localCrop.width(), localCrop.height());
            return std::make_tuple(cf, offset, size);
        };

        // Always update the overlay (center frame)
        if (ui->miniLoaderOverlay) {
            QImage cf;
            QPointF off;
            QSizeF sz;
            std::tie(cf, off, sz) = buildCroppedForFrame(currentFrameNumber);
            ui->miniLoaderOverlay->updateWithCroppedFrame(currentFrameNumber, cf, off, sz, centerPoint);
            YAWT_DEBUG(lcGuiMainWindow) << "updateMiniLoaderCrop - sent cropped frame" << currentFrameNumber << "to widget miniLoaderOverlay"
                     << "size:" << cf.size();
        }

        // Only update +/- frames when playback is not running.
        if (!m_isVideoPlaying) {
            int offsets[4] = { -2, -1, 1, 2 };
            MiniLoader* widgetTargets[4] = {
                (ui->mlon2) ? ui->mlon2 : nullptr,
                (ui->mlon1) ? ui->mlon1 : nullptr,
                (ui->mlop1) ? ui->mlop1 : nullptr,
                (ui->mlop2) ? ui->mlop2 : nullptr
            };

            auto buildBlankForCrop = [&]() -> std::tuple<QImage, QPointF, QSizeF> {
                QRect localCrop = cropRect;
                if (localCrop.isEmpty()) localCrop = cropRect;
                QImage cf(localCrop.size(), QImage::Format_RGB32);
                cf.fill(Qt::black);
                QPointF offset(localCrop.left() + 0.0, localCrop.top() + 0.0);
                QSizeF size(localCrop.width(), localCrop.height());
                return std::make_tuple(cf, offset, size);
            };

            for (int i = 0; i < 4; ++i) {
                int targetFrame = currentFrameNumber + offsets[i];
                MiniLoader* target = widgetTargets[i];
                if (!target) continue;

                QImage cf;
                QPointF off;
                QSizeF sz;
                if (targetFrame < 0 || (totalFrames > 0 && targetFrame >= totalFrames)) {
                    std::tie(cf, off, sz) = buildBlankForCrop();
                } else {
                    std::tie(cf, off, sz) = buildCroppedForFrame(targetFrame);
                }

                target->updateWithCroppedFrame(targetFrame, cf, off, sz, centerPoint);
                YAWT_DEBUG(lcGuiMainWindow) << "updateMiniLoaderCrop - sent cropped frame" << targetFrame << "to widget"
                         << (i==0?QString("mlon2"):(i==1?QString("mlon1"):(i==2?QString("mlop1"):QString("mlop2"))))
                         << "size:" << cf.size();
            }
        }

        // Primary miniLoader (zoom-only) should receive the central frame for display consistency
        if (ui->miniLoader) {
            QImage cf_center;
            QPointF off_center;
            QSizeF sz_center;
            std::tie(cf_center, off_center, sz_center) = buildCroppedForFrame(currentFrameNumber);
            ui->miniLoader->updateWithCroppedFrame(currentFrameNumber, cf_center, off_center, sz_center, centerPoint);
            YAWT_DEBUG(lcGuiMainWindow) << "updateMiniLoaderCrop - sent center cropped frame to primary miniLoader, frame:" << currentFrameNumber;
        }

        // If we sent to an overlay, let it emit visibility signals as before. If no overlay, clear visible set.
        if (ui->miniLoaderOverlay) {
            // Allow overlay to compute visibility for its single frame and emit signals as needed.
            onMiniLoaderVisibleWormsUpdated(ui->miniLoaderOverlay->getVisibleWormIds());
        } else {
            onMiniLoaderVisibleWormsUpdated(QList<int>()); // clear visible set so merge history shows all (or callers adapt)
        }
    }

    // The batching path has been replaced above. No additional fallback logic required here.
    // All five mini loader widgets (mlon2, mlon1, miniLoaderOverlay, mlop1, mlop2) were handled.
    // No-op placeholder to preserve function structure.
}

bool MainWindow::applyThresholdSettingsFromJsonFile(const QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        return false;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        return false;
    }

    Thresholding::ThresholdSettings settings = ui->videoLoader->getCurrentThresholdSettings();
    QJsonObject obj = doc.object();
    settings.algorithm = static_cast<Thresholding::ThresholdAlgorithm>(
        obj.value("algorithm").toInt(static_cast<int>(settings.algorithm)));
    settings.globalThresholdValue = obj.value("globalThresholdValue").toInt(settings.globalThresholdValue);
    settings.adaptiveBlockSize = obj.value("adaptiveBlockSize").toInt(settings.adaptiveBlockSize);
    settings.adaptiveCValue = obj.value("adaptiveCValue").toDouble(settings.adaptiveCValue);
    settings.assumeLightBackground = obj.value("assumeLightBackground").toBool(settings.assumeLightBackground);
    settings.enableBlur = obj.value("enableBlur").toBool(settings.enableBlur);
    settings.blurKernelSize = obj.value("blurKernelSize").toInt(settings.blurKernelSize);
    settings.blurSigmaX = obj.value("blurSigmaX").toDouble(settings.blurSigmaX);

    ui->bgCombo->setCurrentIndex(settings.assumeLightBackground ? 0 : 1);

    if (settings.algorithm == Thresholding::ThresholdAlgorithm::Global ||
        settings.algorithm == Thresholding::ThresholdAlgorithm::Otsu) {
        ui->globalRadio->setChecked(true);
        ui->adaptiveRadio->setChecked(false);
        ui->globalThreshAutoCheck->setChecked(settings.algorithm == Thresholding::ThresholdAlgorithm::Otsu);
        ui->globalThreshSlider->setValue(settings.globalThresholdValue);
        ui->globalThreshValueSpin->setValue(settings.globalThresholdValue);
        updateThresholdAlgorithmSettings();
        setGlobalThresholdType(settings.algorithm == Thresholding::ThresholdAlgorithm::Otsu);
    } else {
        ui->adaptiveRadio->setChecked(true);
        ui->globalRadio->setChecked(false);
        updateThresholdAlgorithmSettings();
        ui->adaptiveTypeCombo->setCurrentIndex(settings.algorithm == Thresholding::ThresholdAlgorithm::AdaptiveGaussian ? 0 : 1);
        ui->blockSizeSpin->setValue(settings.adaptiveBlockSize);
        ui->tuningDoubleSpin->setValue(settings.adaptiveCValue);
        setAdaptiveThresholdType(ui->adaptiveTypeCombo->currentIndex());
    }

    ui->blurCheck->setChecked(settings.enableBlur);
    ui->blurKernelSpin->setValue(settings.blurKernelSize);
    ui->videoLoader->setBlurSigmaX(settings.blurSigmaX);

    return true;
}

void MainWindow::onPlaybackStateChanged(bool isPlaying, double currentSpeed) {
    Q_UNUSED(currentSpeed);
    m_isVideoPlaying = isPlaying;
    setSideMiniLoadersPaused(isPlaying);
    if (isPlaying) {
        m_lastMiniLoaderFrame = -1;
    }

    if (!isPlaying && ui->videoLoader && ui->videoLoader->isVideoLoaded()) {
        int currentFrame = ui->videoLoader->getCurrentFrameNumber();
        QImage currentImage = ui->videoLoader->getCurrentQImageFrame();
        if (!currentImage.isNull()) {
            updateMiniLoaderCrop(currentFrame, currentImage);
        }
    }
}

void MainWindow::resultsButtonClicked() {
    if (m_analysisDialog) {
        m_analysisDialog->close();
        return;
    }

    m_analysisDialog = new AnalysisDialog(m_trackingDataStorage, this);
    ui->resultsButton->setChecked(true);
    m_analysisDialog->setPixelSizeUmPerPixel(ui->pixelSizeSpinBoxD->value());
    m_analysisDialog->setVideoFps(m_videoFps);
    connect(ui->pixelSizeSpinBoxD, qOverload<double>(&QDoubleSpinBox::valueChanged),
            m_analysisDialog, &AnalysisDialog::setPixelSizeUmPerPixel);
    connect(m_analysisDialog, &QDialog::finished, this, [this](int) {
        ui->resultsButton->setChecked(false);
        m_analysisDialog = nullptr;
    });
    m_analysisDialog->show();
    m_analysisDialog->raise();
    m_analysisDialog->activateWindow();
}

void MainWindow::setSideMiniLoadersPaused(bool paused) {
    MiniLoader* targets[4] = {
        ui->mlon2 ? ui->mlon2 : nullptr,
        ui->mlon1 ? ui->mlon1 : nullptr,
        ui->mlop1 ? ui->mlop1 : nullptr,
        ui->mlop2 ? ui->mlop2 : nullptr
    };

    for (MiniLoader* target : targets) {
        if (!target) continue;
        target->setEnabled(!paused);

        if (paused) {
            QGraphicsOpacityEffect* effect = qobject_cast<QGraphicsOpacityEffect*>(target->graphicsEffect());
            if (!effect) {
                effect = new QGraphicsOpacityEffect(target);
                target->setGraphicsEffect(effect);
            }
            effect->setOpacity(0.35);
        } else if (target->graphicsEffect()) {
            target->setGraphicsEffect(nullptr);
        }
    }
}

void MainWindow::frameSliderMoved(int value) {
    ui->videoLoader->seekToFrame(value, false);
    if (!ui->framePosition->hasFocus()) {
        ui->framePosition->setValue(value);
    }
    // Also update the mirrored spinbox if it's not being edited
}

// Slot: receive visible worm IDs from MiniLoader. Update cached set and rebuild merge history text.
// Emitted frequently (on repaint), so keep this lightweight.
void MainWindow::onMiniLoaderVisibleWormsUpdated(const QList<int>& visibleIds) {
    // Log received visible IDs. Merge history text widget and MergeViewer were removed from the UI,
    // so we no longer update them here. Keep the last-polled set updated so polling fallback remains usable.
    YAWT_DEBUG(lcGuiMainWindow) << "onMiniLoaderVisibleWormsUpdated received visibleIds:" << visibleIds;

    m_lastPolledVisibleIds.clear();
    for (int id : visibleIds) m_lastPolledVisibleIds.insert(id);
    // Note: any higher-level consumers that previously relied on MainWindow to forward these IDs
    // should now connect directly to MiniLoader or the appropriate model/storage.
}

// Poll timer fallback handler: periodically query any MiniLoader instances for visible IDs and update
// MainWindow state if the polled set changes. This complements the signal-based updates and prevents
// missed updates due to paint/emit ordering differences.
void MainWindow::onMiniLoaderPollTimeout() {
    QSet<int> polledSet;
    // Query primary miniLoader instance if present
    if (ui->miniLoader) {
        const QList<int> vis = ui->miniLoader->getVisibleWormIds();
        for (int id : vis) polledSet.insert(id);
    }
    // Query overlay miniLoader instance if present
    if (ui->miniLoaderOverlay) {
        const QList<int> vis = ui->miniLoaderOverlay->getVisibleWormIds();
        for (int id : vis) polledSet.insert(id);
    }

    // If polled set differs from the last polled set, invoke the visible-IDs handler to update UI.
    if (polledSet != m_lastPolledVisibleIds) {
        m_lastPolledVisibleIds = polledSet;
        QList<int> list;
        list.reserve(polledSet.size());
        for (int id : polledSet) list.append(id);
        YAWT_DEBUG(lcGuiMainWindow) << "onMiniLoaderPollTimeout polled visible IDs:" << list;
        onMiniLoaderVisibleWormsUpdated(list);
    }
}

// Helper: build the textual merge history block for a given frame using a provided visible set.
// If visibleSet is empty, treat it as \"no filtering\" (show all merges).
/* buildMergeHistoryText removed from MainWindow.
   Merge history textual output is no longer maintained by MainWindow.
   Reintroduce a separate component or helper if this functionality is required again. */

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
    if (arePlayButtonsChecked())
    {
        // Use helper to pause and sync both play buttons
        setPlayButtonsState(false);
    }
}

// --- Tracking Process ---
/**
 * @brief UI entry point for starting tracking.
 *
 * Gathers current video context (path, keyframe, threshold settings, total frames) from the UI
 * and delegates orchestration to AppController by showing the controller-owned dialog or
 * starting tracking directly as configured.
 */
void MainWindow::onStartTrackingActionTriggered() {
    if (!ui->videoLoader || !ui->videoLoader->isVideoLoaded()) {
        QMessageBox::warning(this, "Tracking Setup", "No video loaded."); return;
    }
    QString videoPath = ui->videoLoader->getCurrentVideoPath();
    if (videoPath.isEmpty()) {
        QMessageBox::critical(this, "Error", "Video path is empty."); return;
    }

    std::vector<Tracking::InitialWormInfo> initialWorms;
    const QList<TableItems::ClickedItem>& items = m_blobTableModel->getAllItems();
    for(const TableItems::ClickedItem& item : items) {
        if(item.type == TableItems::ItemType::Worm) { // Ensure you have a way to designate items as actual worms for tracking
            Tracking::InitialWormInfo info;
            info.id = item.id;
            info.initialRoi = item.initialBoundingBox;
            info.color = item.color;
            initialWorms.push_back(info);
        }
    }
    if (initialWorms.empty()) {
        QMessageBox::information(this, "Tracking", "No items marked as 'Worm' in the table to track."); return;
    }

    int keyFrame = ui->videoLoader->getCurrentFrameNumber();
    Thresholding::ThresholdSettings settings = ui->videoLoader->getCurrentThresholdSettings();
    int totalFrames = ui->videoLoader->getTotalFrames();

    // Delegate dialog creation and orchestration to AppController.
    if (m_appController) {
        bool onlyTrackMissing = true; // Default behavior; dialog can override when created by controller.
        QString dataDirectory = ui->videoLoader ? ui->videoLoader->getDataDirectory() : QString();
        m_appController->showTrackingDialog(videoPath, keyFrame, settings, onlyTrackMissing, totalFrames, dataDirectory, this);
    } else {
        QMessageBox::critical(this, "Error", "Internal error: AppController missing.");
    }
}

void MainWindow::handleBeginTrackingFromDialog() {
    if (!m_appController || !ui->videoLoader || !ui->videoLoader->isVideoLoaded()) {
        QMessageBox::critical(this, "Error", "Internal error: Components missing.");
        return;
    }

    QString videoPath = ui->videoLoader->getCurrentVideoPath();
    int keyFrame = ui->videoLoader->getCurrentFrameNumber();
    Thresholding::ThresholdSettings settings = ui->videoLoader->getCurrentThresholdSettings();
    int totalFrames = ui->videoLoader->getTotalFrames();

    // Delegate building initial worms and starting tracking to the AppController.
    // Let the controller handle filtering (only-missing) and orchestration.
    bool onlyTrackMissing = true;
    QString dataDirectory = ui->videoLoader ? ui->videoLoader->getDataDirectory() : QString();
    m_appController->beginTrackingFromModel(videoPath, keyFrame, settings, onlyTrackMissing, totalFrames, dataDirectory);
}

void MainWindow::handleCancelTrackingFromDialog() {
    if (m_appController) m_appController->cancelTracking();
}

void MainWindow::acceptTracksFromManager(const Tracking::AllWormTracks& tracks) {
    YAWT_INFO(lcGuiMainWindow) << "Received" << tracks.size() << "tracks.";

    // Store tracks in the central data storage
    for (auto it = tracks.begin(); it != tracks.end(); ++it) {
        YAWT_DEBUG(lcGuiMainWindow) << "Storing track for worm" << it->first << "with" << it->second.size() << "points";
        m_trackingDataStorage->setTrackForItem(it->first, it->second);
    }

    // Debug: Verify tracks were stored
    YAWT_DEBUG(lcGuiMainWindow) << "TrackingDataStorage now has" << m_trackingDataStorage->getAllTracks().size() << "tracks";

    // Refresh annotation table to show lost tracking events
    if (m_annotationTableModel) {
        m_annotationTableModel->refreshAnnotations();
        YAWT_DEBUG(lcGuiMainWindow) << "Refreshed annotation table with" << m_annotationTableModel->rowCount() << "annotations";
    }

    // VideoLoader still needs direct track data for backward compatibility
    // It will also get data from storage now
    //    ui->videoLoader->setTracksToDisplay(tracks);
    // Use the union of all tracks in central storage so we don't overwrite previously saved tracks
    if (m_trackingDataStorage) {
        const auto& allTracks = m_trackingDataStorage->getAllTracks();
        qDebug() << "MainWindow: Supplying VideoLoader with" << allTracks.size() << "total tracks from storage";
        ui->videoLoader->setTracksToDisplay(allTracks);
    } else {
        // Fallback if storage not available
        ui->videoLoader->setTracksToDisplay(tracks);
    }

    if (!tracks.empty()) { // Optionally switch to tracks view
        ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Tracks, true);
        ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::PanZoom);
        ui->wormTableView->selectAll();
        // ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::EditTracks); // If desired
    }

    // Keep track data in storage for MiniLoader overlays and other components
    // Memory cleanup will be handled elsewhere if needed

    // Mark that we have completed tracking
    m_hasCompletedTracking = true;

    statusBar()->showMessage("Tracking completed", 4000);

    // Perform memory cleanup after tracking is complete
    performPostTrackingMemoryCleanup();
    updateWormTimeline();
}

void MainWindow::setupPlaybackSpeedComboBox() {
    // Clear any existing items on the combobox
    ui->comboPlaybackSpeed->clear();

    // Helper lambda to populate a combobox with identical entries
    auto populate = [](QComboBox* cb) {
        cb->addItem("0.25x", 0.25);
        cb->addItem("0.5x", 0.5);
        cb->addItem("0.75x", 0.75);
        cb->addItem("1.0x", 1.0);
        //cb->addItem("1.25x", 1.25);
        cb->addItem("1.5x", 1.5);
        cb->addItem("2.0x", 2.0);
        cb->addItem("2.5x", 2.5);
        //cb->addItem("3.0x", 3.0);
        cb->addItem("5.0x", 5.0);
        cb->addItem("10.0x", 10.0);
        cb->addItem("20.0x", 20.0);
        // Default index (1.0x)
        cb->setCurrentIndex(3);
    };

    populate(ui->comboPlaybackSpeed);
    qDebug() << "MainWindow: Playback speed combobox initialized";
}

void MainWindow::onPlaybackSpeedChanged(int index) {
    // Determine which combobox sent the signal so we use the correct itemData
    QComboBox* senderCombo = qobject_cast<QComboBox*>(sender());
    if (!senderCombo) senderCombo = ui->comboPlaybackSpeed;

    if (index < 0 || index >= senderCombo->count()) {
        return;
    }

    // Get the speed multiplier from the sending combobox's item data
    double speedMultiplier = senderCombo->itemData(index).toDouble();

    qDebug() << "MainWindow: Setting playback speed to" << speedMultiplier << "x";

    // Set the speed in VideoLoader
    ui->videoLoader->setPlaybackSpeed(speedMultiplier);

    // Keep both comboboxes visually in sync
    updatePlaybackSpeedComboBox(speedMultiplier);
}

void MainWindow::updatePlaybackSpeedComboBox(double speedMultiplier) {
    for (int i = 0; i < ui->comboPlaybackSpeed->count(); ++i) {
        double itemSpeed = ui->comboPlaybackSpeed->itemData(i).toDouble();
        if (qFuzzyCompare(itemSpeed, speedMultiplier)) {
            // Block signals to avoid recursive calls
            ui->comboPlaybackSpeed->blockSignals(true);
            ui->comboPlaybackSpeed->setCurrentIndex(i);
            ui->comboPlaybackSpeed->blockSignals(false);
            break;
        }
    }
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

    // DEBUG: Check zoom factor at start of annotation click
    double initialZoom = ui->videoLoader->getZoomFactor();
    qDebug() << "MainWindow: Annotation click START - zoom factor:" << initialZoom;

    // Pause playback if it's currently playing (either play button)
    if (arePlayButtonsChecked()) {
        // Use helper to pause and sync both play buttons
        setPlayButtonsState(false);
        qDebug() << "MainWindow: Paused playback for annotation navigation";
    }

    // Seek to the start frame of the annotation
    int targetFrame = annotation->startFrame;
    int targetWormId = annotation->wormId;
    qDebug() << "MainWindow: About to seek to frame" << targetFrame;
    qDebug() << "MainWindow: Seeking to annotation frame" << targetFrame << "for worm" << targetWormId;

    seekFrame(targetFrame);

    // DEBUG: Check zoom factor after seeking
    double zoomAfterSeek = ui->videoLoader->getZoomFactor();
    qDebug() << "MainWindow: After seekFrame - zoom factor:" << zoomAfterSeek;
    qDebug() << "MainWindow: About to find blob table row for worm" << targetWormId;

    // Update the frame position display
    if (!ui->framePosition->hasFocus()) {
        ui->framePosition->setValue(targetFrame);
    }

    // Select the corresponding worm in the blob table
    int blobTableRow = -1;

    // DEBUG: Check zoom factor before blob selection
    double zoomBeforeBlobSelection = ui->videoLoader->getZoomFactor();
    qDebug() << "MainWindow: Before blob selection - zoom factor:" << zoomBeforeBlobSelection;

    if (m_wormProxyModel) {
        const int idCol = static_cast<int>(BlobTableModel::Column::ID);
        for (int i = 0; i < m_wormProxyModel->rowCount(); ++i) {
            QModelIndex idx = m_wormProxyModel->index(i, idCol);
            if (m_wormProxyModel->data(idx, Qt::DisplayRole).toInt() == targetWormId) {
                blobTableRow = i;
                break;
            }
        }
    }

    // Select the row in the worm table if found
    if (blobTableRow >= 0 && m_wormProxyModel) {
        QModelIndex blobIndex = m_wormProxyModel->index(blobTableRow, 0);
        ui->wormTableView->setCurrentIndex(blobIndex);
        ui->wormTableView->selectionModel()->select(blobIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
        ui->wormTableView->scrollTo(blobIndex, QAbstractItemView::EnsureVisible);
        qDebug() << "MainWindow: Selected worm" << targetWormId << "in worm table (row" << blobTableRow << ")";
    } else {
        // Clear selection if worm not found in worm table
        ui->wormTableView->clearSelection();
        qDebug() << "MainWindow: Worm" << targetWormId << "not found in worm table, cleared selection";
    }

    // Show status message
    QString statusMessage;
    if (annotation->startFrame == annotation->endFrame) {
        statusMessage = QString("Navigated to frame %1 - Worm %2 tracking lost")
                       .arg(targetFrame).arg(annotation->wormId);
    } else {
        statusMessage = QString("Navigated to frame %1 - Worm %2 tracking lost (frames %1-%3)")
                       .arg(targetFrame).arg(annotation->wormId).arg(annotation->endFrame);
    }
    statusBar()->showMessage(statusMessage, 4000);
}

// --- Table View and VideoLoader Sync ---
void MainWindow::updateVisibleTracksInVideoLoader(const QItemSelection &selected, const QItemSelection &deselected) {
    Q_UNUSED(selected);
    Q_UNUSED(deselected);

    QSet<int> wormItemIDs;
    if (m_trackingDataStorage) {
        const QList<TableItems::ClickedItem>& allItems = m_trackingDataStorage->getAllItems();
        for (const auto& item : allItems) {
            if (item.type == TableItems::ItemType::Worm || item.type == TableItems::ItemType::Fix) {
                wormItemIDs.insert(item.id);
            }
        }
    } else if (m_blobTableModel) {
        const QList<TableItems::ClickedItem>& allItems = m_blobTableModel->getAllItems();
        for (const auto& item : allItems) {
            if (item.type == TableItems::ItemType::Worm || item.type == TableItems::ItemType::Fix) {
                wormItemIDs.insert(item.id);
            }
        }
    }

    qDebug() << "MainWindow: Setting worm item IDs as visible in VideoLoader:" << wormItemIDs;
    ui->videoLoader->setVisibleTrackIDs(wormItemIDs);
}

void MainWindow::performPostTrackingMemoryCleanup() {
    YAWT_INFO(lcGuiMainWindow) << "Performing post-tracking memory cleanup...";

    // Get memory usage before cleanup
    double cacheHitRate = ui->videoLoader->getCacheHitRate();
    int cacheSize = ui->videoLoader->getCacheSize();

    YAWT_INFO(lcGuiMainWindow) << "VideoLoader cache status before cleanup - Size:" << cacheSize
             << "frames, Hit rate:" << QString::number(cacheHitRate, 'f', 1) << "%";

    // Reduce VideoLoader frame cache size significantly after tracking
    // During tracking, we don't need as many cached frames since we're not seeking rapidly
    int originalCacheSize = 50; // Default cache size
    int reducedCacheSize = 10;  // Smaller cache for post-tracking

    if (cacheSize > reducedCacheSize) {
        ui->videoLoader->setCacheSize(reducedCacheSize);
        YAWT_INFO(lcGuiMainWindow) << "Reduced VideoLoader cache from" << cacheSize << "to" << reducedCacheSize << "frames";
    }

    // Clear any temporary UI state that might hold large data
    // Model will automatically refresh when needed

    // Report final cache status
    int finalCacheSize = ui->videoLoader->getCacheSize();
    double finalHitRate = ui->videoLoader->getCacheHitRate();

    qDebug() << "MainWindow: Memory cleanup complete - Final cache size:" << finalCacheSize
             << "frames, Hit rate:" << QString::number(finalHitRate, 'f', 1) << "%";

    // Estimate memory freed (rough calculation)
    int framesFreed = cacheSize - finalCacheSize;
    if (framesFreed > 0) {
        // Assume ~1MB per frame for rough estimate (depends on resolution)
        double estimatedMBFreed = framesFreed * 1.0;
        qDebug() << "MainWindow: Estimated" << QString::number(estimatedMBFreed, 'f', 1)
                 << "MB freed from VideoLoader cache reduction";
    }
}

// --- Debug Control ---
void MainWindow::toggleTrackingDebug() {
    QString msg = "Command-line logging uses --verbose; Debug tab capture uses --debug.";
    qDebug().noquote() << "MainWindow:" << msg;
    statusBar()->showMessage(msg, 4000);
}

// ── Debug tab: Rerun Centerline ───────────────────────────────────────────────

void MainWindow::onRerunCenterlineClicked()
{
    if (!m_appController) return;

    // Read current spinbox values from the Debug tab into a params struct.
    Centerline::CenterlineSnakeParams params;
    if (auto* g = ui->snakeDebugGroup) {
        params.enabled = g->isChecked();
    }
    if (auto* sb = ui->snakeAlphaSpin)        params.alpha                    = sb->value();
    if (auto* sb = ui->snakeBetaSpin)         params.beta                     = sb->value();
    if (auto* sb = ui->snakeLambdaSpin)       params.lambda                   = sb->value();
    if (auto* sb = ui->snakeItersSpin)        params.iterations               = sb->value();
    if (auto* sb = ui->snakeStepSpin)         params.stepSize                 = sb->value();
    if (auto* sb = ui->snakeOrientThreshSpin) params.orientationAngleThreshold= sb->value();
    if (auto* sb = ui->snakeNPointsSpin)      params.nPoints                   = sb->value();

    // Disable the button while running to prevent double-triggers.
    ui->rerunCenterlineButton->setEnabled(false);
    ui->centerlineDebugProgressBar->setValue(0);
    ui->centerlineDebugProgressBar->setVisible(true);
    ui->centerlineDebugStatusLabel->setText("Running centerline pass...");

    // This stores the params on the controller and immediately starts a
    // background CenterlineWorker pass. Results are written back to storage;
    // the videoloader re-draws on its next paint event (triggered by the
    // finished slot below).
    m_appController->rerunCenterline(params);
}

void MainWindow::onDebugCenterlineProgress(int percentage)
{
    if (ui->centerlineDebugProgressBar->isVisible())
        ui->centerlineDebugProgressBar->setValue(percentage);
}

void MainWindow::onDebugCenterlineFinished()
{
    ui->rerunCenterlineButton->setEnabled(true);
    ui->centerlineDebugProgressBar->setVisible(false);
    ui->centerlineDebugStatusLabel->setText("Done — viewer updated.");

    // Force the videoloader to repaint so the new centerlines appear immediately
    // on the current frame without requiring a frame seek.
    if (ui->videoLoader) {
        ui->videoLoader->update();
    }

    // Refresh the per-worm tip-baseline readout. Each row reports the worm's
    // running curvature, width, and body-length distributions accumulated from
    // clean-topology frames during this centerline pass, plus per-state frame
    // counts (Phase C.2) showing how often each worm is in each topological
    // state. A ✓ marks worms whose sample count is high enough for the
    // discriminator to be useful.
    if (auto* edit = ui->tipBaselineTextEdit) {
        if (m_trackingDataStorage) {
            const QMap<int, Centerline::TipFeatureBaseline> baselines =
                m_trackingDataStorage->getAllTipBaselines();
            if (baselines.isEmpty()) {
                edit->setPlainText("No clean-topology frames found in this pass.");
            } else {
                // Tally per-worm topology-state counts by sweeping all stored
                // blobs once. Cheaper than threading state counts through the
                // worker's pass loop, and keeps the worker's job purely about
                // computation.
                QMap<int, std::array<int, 5>> stateCounts; // [Unknown, Clean, SelfCrossed, Merged, Lost]
                const Tracking::AllWormTracks& tracks =
                    m_trackingDataStorage->getAllTracks();
                for (auto it = tracks.begin(); it != tracks.end(); ++it) {
                    const int wormId = it->first;
                    auto& counts = stateCounts[wormId];
                    counts.fill(0);
                    for (const auto& tp : it->second) {
                        const auto blobs =
                            m_trackingDataStorage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
                        if (!blobs.contains(wormId)) continue;
                        const int idx = static_cast<int>(blobs[wormId].topologyState);
                        if (idx >= 0 && idx < 5) ++counts[idx];
                    }
                }

                QString report;
                report += QStringLiteral(
                    "ID  |κ| mean ± sd  (n)   width mean ± sd  (n)   length mean ± sd  (n)   Clean SCross Merg Lost  ok\n");
                report += QStringLiteral(
                    "─── ──────────────────── ──────────────────── ──────────────────── ───── ────── ──── ────  ──\n");
                for (auto it = baselines.constBegin(); it != baselines.constEnd(); ++it) {
                    const int wormId = it.key();
                    const Centerline::TipFeatureBaseline& b = it.value();
                    const auto& c = stateCounts.value(wormId, {0, 0, 0, 0, 0});
                    report += QString::asprintf(
                        "%-3d %5.3f ± %5.3f (%4d) %5.2f ± %4.2f (%4d) %6.1f ± %5.1f (%4d) %5d %6d %4d %4d  %s\n",
                        wormId,
                        b.meanAbsCurvature, b.curvatureStdDev(), b.curvatureSamples,
                        b.meanWidth,        b.widthStdDev(),     b.widthSamples,
                        b.meanBodyLength,   b.bodyLengthStdDev(), b.lengthSamples,
                        c[static_cast<int>(Tracking::TopologyState::Clean)],
                        c[static_cast<int>(Tracking::TopologyState::SelfCrossed)],
                        c[static_cast<int>(Tracking::TopologyState::Merged)],
                        c[static_cast<int>(Tracking::TopologyState::Lost)],
                        b.isReliable() ? "✓" : "·");
                }
                edit->setPlainText(report);
            }
        }
    }
}

void MainWindow::onExportProcessClicked()
{
    // Resolve the currently selected worm in the worm table.
    int wormId = -1;
    if (ui->wormTableView->selectionModel() && m_wormProxyModel && m_blobTableModel) {
        const QModelIndexList sel =
            ui->wormTableView->selectionModel()->selectedIndexes();
        if (!sel.isEmpty()) {
            const QModelIndex proxyIdx = m_wormProxyModel->index(sel.first().row(), 0);
            const QModelIndex srcIdx = m_wormProxyModel->mapToSource(proxyIdx);
            if (srcIdx.isValid())
                wormId = m_blobTableModel->getItem(srcIdx.row()).id;
        }
    }
    if (wormId < 0) {
        QMessageBox::information(this, "Export Process",
            "Select a worm in the worm table first.");
        return;
    }

    const int frame = ui->videoLoader ? ui->videoLoader->getCurrentFrameNumber() : -1;
    if (frame < 0) {
        QMessageBox::warning(this, "Export Process", "No current frame available.");
        return;
    }

    const QString dataDir = ui->videoLoader ? ui->videoLoader->getDataDirectory() : QString();
    if (dataDir.isEmpty()) {
        QMessageBox::warning(this, "Export Process",
            "No yawt data directory available. Load a video first.");
        return;
    }

    const QString videoPath = ui->videoLoader ? ui->videoLoader->getCurrentVideoPath() : QString();
    const QString videoBaseName = QFileInfo(videoPath).completeBaseName();
    if (videoBaseName.isEmpty()) {
        QMessageBox::warning(this, "Export Process",
            "No current video name available.");
        return;
    }

    const QString videoSpecificDir = QDir(dataDir).absoluteFilePath(videoBaseName);
    QDir videoDir(videoSpecificDir);
    const QStringList procDirs = videoDir.entryList(QStringList() << "PROC_*",
                                                    QDir::Dirs | QDir::NoDotAndDotDot,
                                                    QDir::Name);
    if (procDirs.isEmpty()) {
        QMessageBox::warning(this, "Export Process",
            QString("No processing output directory found under:\n%1\n\nRun tracking first.")
                .arg(videoSpecificDir));
        return;
    }

    const QString processingDir = videoDir.absoluteFilePath(procDirs.constLast());
    const QString outDir = QDir(processingDir).absoluteFilePath(
        QString("DEBUG/worm%1_frame%2").arg(wormId).arg(frame));
    if (!QDir().mkpath(outDir)) {
        QMessageBox::warning(this, "Export Process",
            QString("Could not create output directory:\n%1").arg(outDir));
        return;
    }

    ui->exportProcessButton->setEnabled(false);
    ui->centerlineDebugStatusLabel->setText(
        QString("Exporting worm %1 frame %2...").arg(wormId).arg(frame));
    QApplication::processEvents();

    QString err;
    const bool ok = Debug::DebugExporter::exportCenterlineFrame(
        m_trackingDataStorage,
        m_appController ? m_appController->debugDataStore() : nullptr,
        wormId,
        frame,
        outDir,
        &err);

    ui->exportProcessButton->setEnabled(true);
    if (ok) {
        ui->centerlineDebugStatusLabel->setText(
            QString("Exported to %1").arg(outDir));
    } else {
        ui->centerlineDebugStatusLabel->setText(QString("Export failed: %1").arg(err));
        QMessageBox::warning(this, "Export Process",
            QString("Export failed: %1").arg(err));
    }
}

// Retracking UI and logic removed
