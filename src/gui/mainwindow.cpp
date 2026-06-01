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
#include "analysispanel.h"
#include "trackingprogressdialog.h"
#include "../core/appcontroller.h"
#include "../core/centerlineworker.h"
#include "../debug/debugexporter.h"
#include "trackingmanager.h"
#include "trackingdatastorage.h"
#include "version.h"
#include "wormtimeline.h"
#include "capturepanel.h"
#include "scaledialog.h"
#include "../data/videometadatastore.h"
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
#include <QActionGroup>
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

    // Capture tab: wire the QObject controller to the widgets defined in mainwindow.ui.
    {
        m_capturePanel = new CapturePanel(this);
        CapturePanel::Widgets cw;
        cw.liveView          = ui->captureVideoView;
        cw.drawRoiButton     = ui->captureDrawRoiButton;
        cw.clearRoiButton    = ui->captureClearRoiButton;
        cw.roiLabel          = ui->captureRoiLabel;
        cw.cameraCombo       = ui->captureCameraCombo;
        cw.scanButton        = ui->captureScanButton;
        cw.resolutionCombo   = ui->captureResolutionCombo;
        cw.connectButton     = ui->captureConnectButton;
        cw.disconnectButton  = ui->captureDisconnectButton;
        cw.recordButton      = ui->captureRecordButton;
        cw.outputPathEdit    = ui->captureOutputPathEdit;
        cw.statusLabel       = ui->captureStatusLabel;
        cw.exposureSpin      = ui->captureExposureSpin;
        cw.exposureAutoCheck = ui->captureExposureAutoCheck;
        cw.exposureLabel     = ui->captureExposureLabel;
        cw.gainSpin          = ui->captureGainSpin;
        cw.gainAutoCheck     = ui->captureGainAutoCheck;
        cw.gainLabel         = ui->captureGainLabel;
        cw.gammaSpin         = ui->captureGammaSpin;
        cw.gammaLabel        = ui->captureGammaLabel;
        cw.fpsSpin           = ui->captureFpsSpin;
        cw.fpsLabel          = ui->captureFpsLabel;
        cw.brightnessSpin    = ui->captureBrightnessSpin;
        cw.brightnessLabel   = ui->captureBrightnessLabel;
        cw.setScaleButton    = ui->setScaleButton;
        cw.scaleLabel        = ui->captureScaleLabel;
        m_capturePanel->setup(cw);
        connect(ui->dirSelected, &QLineEdit::textChanged,
                m_capturePanel, &CapturePanel::setOutputDirectory);

        // Convert the scale result to µm/pixel and push it into the spinbox.
        // The existing valueChanged→AppController connection handles the rest.
        connect(m_capturePanel, &CapturePanel::pixelScaleSet,
                this, [this](double pixelsPerUnit, const QString& unit) {
            double umFactor = 0.0;  // converts user unit → µm
            if      (unit == "mm")   umFactor = 1000.0;
            else if (unit == "µm")   umFactor = 1.0;
            else if (unit == "cm")   umFactor = 10000.0;
            else if (unit == "inch") umFactor = 25400.0;
            if (umFactor > 0 && pixelsPerUnit > 0)
                ui->pixelSizeSpinBoxD->setValue(umFactor / pixelsPerUnit);
        });

        // Camera scanning is intentionally manual — the probe briefly opens camera
        // handles which triggers macOS's privacy sound even when muted. The user
        // initiates it explicitly via the Scan button.

        // Double-clicking the output path field opens the same folder dialog as
        // selectDirButton, keeping both fields in sync.
        ui->captureOutputPathEdit->installEventFilter(this);
    }

    if (ui->tabWidget && ui->debugTab) {
        const int debugTabIndex = ui->tabWidget->indexOf(ui->debugTab);
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

    YAWT_INFO(lcGuiMainWindow) << "AppController created and models bound";

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

    // Wire the Analysis tab panel to the widgets defined in mainwindow.ui.
    {
        m_analysisPanel = new AnalysisPanel(m_trackingDataStorage, this);
        AnalysisPanel::Widgets aw;
        aw.arenaShapeCombo    = ui->analysisArenaShapeCombo;
        aw.arenaSizeSpin      = ui->analysisArenaSizeSpin;
        aw.speedRangeCheck    = ui->analysisSpeedRangeCheck;
        aw.speedRangeMinSpin  = ui->analysisSpeedRangeMinSpin;
        aw.speedRangeMaxSpin  = ui->analysisSpeedRangeMaxSpin;
        aw.wormListView       = ui->analysisWormListView;
        aw.addGroupBtn        = ui->analysisAddGroupBtn;
        aw.refreshBtn         = ui->analysisRefreshBtn;
        aw.plotSelector       = ui->analysisPlotSelector;
        aw.mdiArea            = ui->analysisMdiArea;
        aw.splitter           = ui->analysisSplitter;
        m_analysisPanel->setup(aw);
        m_analysisPanel->setPixelSizeUmPerPixel(ui->pixelSizeSpinBoxD->value());
        m_analysisPanel->setVideoFps(m_videoFps);

        connect(ui->pixelSizeSpinBoxD, qOverload<double>(&QDoubleSpinBox::valueChanged),
                m_analysisPanel, &AnalysisPanel::setPixelSizeUmPerPixel);
        connect(m_analysisPanel, &AnalysisPanel::wormSelectionChanged,
                this, [this](const QSet<int>& ids) {
                    if (m_analysisTabActive) {
                        // Sync wormTableView selection without re-entering the analysis-tab handler
                        m_analysisTabActive = false;
                        QItemSelection sel;
                        const int nCols = m_wormProxyModel->columnCount();
                        for (int row = 0; row < m_wormProxyModel->rowCount(); ++row) {
                            QModelIndex srcIdx = m_wormProxyModel->mapToSource(
                                m_wormProxyModel->index(row, 0));
                            if (srcIdx.isValid()) {
                                const int id = m_blobTableModel->getItem(srcIdx.row()).id;
                                if (ids.contains(id))
                                    sel.select(m_wormProxyModel->index(row, 0),
                                               m_wormProxyModel->index(row, nCols - 1));
                            }
                        }
                        ui->wormTableView->selectionModel()->select(
                            sel, QItemSelectionModel::ClearAndSelect);
                        m_analysisTabActive = true;
                    }
                });
    }

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

bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
    if (obj == ui->captureOutputPathEdit
        && event->type() == QEvent::MouseButtonPress) {
        chooseWorkingDirectory();
        return true; // consumed
    }
    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::onPixelSizeSpinEditingFinished()
{
    if (m_currentVideoDataDir.isEmpty() || m_currentVideoBaseName.isEmpty()) return;
    const double umPerPixel = ui->pixelSizeSpinBoxD->value();
    VideoMetadataStore::saveUmPerPixel(
        m_currentVideoDataDir, m_currentVideoBaseName, umPerPixel);
}

void MainWindow::onMeasureButtonClicked()
{
    if (!ui->videoLoader->isVideoLoaded()) {
        // Nothing to measure against — silently ignore.
        return;
    }
    ScaleDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted) return;

    // Stash dialog values so onVideoScaleMeasured can use them.
    m_pendingScalePhysical = dlg.physicalValue();
    m_pendingScaleUnit     = dlg.unit();

    ui->videoLoader->setScaleMeasureMode(true);
    ui->processingScaleLabel->setText(
        QString("Draw a line = %1 %2")
            .arg(m_pendingScalePhysical, 0, 'g', 4)
            .arg(m_pendingScaleUnit));
}

void MainWindow::onVideoScaleMeasured(double pixelLength)
{
    if (pixelLength < 1.0 || m_pendingScaleUnit.isEmpty()) return;

    const double pixelsPerUnit = pixelLength / m_pendingScalePhysical;

    // Convert to µm/pixel for the spinbox.
    auto unitToUm = [](const QString& u) -> double {
        if (u == "mm")   return 1000.0;
        if (u == "µm")   return 1.0;
        if (u == "cm")   return 10000.0;
        if (u == "inch") return 25400.0;
        return 0.0;
    };
    const double factor     = unitToUm(m_pendingScaleUnit);
    const double umPerPixel = (factor > 0 && pixelsPerUnit > 0) ? factor / pixelsPerUnit : 0.0;

    if (umPerPixel > 0) {
        ui->pixelSizeSpinBoxD->setValue(umPerPixel);   // triggers valueChanged → AppController
        ui->processingScaleLabel->setText(
            QString("Scale: %1 µm/px").arg(umPerPixel, 0, 'f', 2));

        // Persist to this video's metadata JSON immediately.
        if (!m_currentVideoDataDir.isEmpty() && !m_currentVideoBaseName.isEmpty())
            VideoMetadataStore::saveUmPerPixel(
                m_currentVideoDataDir, m_currentVideoBaseName, umPerPixel);
    }

    m_pendingScalePhysical = 0.0;
    m_pendingScaleUnit.clear();
}

void MainWindow::setupConnections() {
    // Connect ROI factor spinbox to BlobTableModel
    connect(ui->roiFactorSpinBoxD, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_blobTableModel, &BlobTableModel::updateRoiSizeMultiplier);
    // Spinbox now shows µm/pixel; AppController still takes pixels/µm — invert at boundary.
    connect(ui->pixelSizeSpinBoxD, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double umPerPixel) {
                const double pixelsPerUm = (umPerPixel > 0.0) ? 1.0 / umPerPixel : 0.0;
                m_appController->setPixelSizePixelsPerUm(pixelsPerUm);
            });
    // Save to video metadata JSON when the user manually finishes editing the spinbox.
    connect(ui->pixelSizeSpinBoxD, &QDoubleSpinBox::editingFinished,
            this, &MainWindow::onPixelSizeSpinEditingFinished);

    // Processing tab ruler calibration
    connect(ui->measureButton, &QToolButton::clicked,
            this, &MainWindow::onMeasureButtonClicked);
    connect(ui->videoLoader,   &VideoLoader::scaleMeasured,
            this, &MainWindow::onVideoScaleMeasured);

    // File/Directory
    connect(ui->selectDirButton, &QToolButton::clicked, this, &MainWindow::chooseWorkingDirectory);

    // When the project folder changes, populate the Analysis tab from the yawt
    // subfolder immediately — no need to open a video first.
    connect(ui->dirSelected, &QLineEdit::textChanged,
            this, [this](const QString& dir) {
        if (!m_analysisPanel || dir.isEmpty()) return;
        const QString yawtDir = QDir(dir).absoluteFilePath("yawt");
        if (QDir(yawtDir).exists())
            m_analysisPanel->setYawtDirectory(yawtDir);
    });

    // VideoLoader basic signals
    connect(ui->videoLoader, &VideoLoader::videoLoaded, this, &MainWindow::initiateFrameDisplay);
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, &MainWindow::updateFrameDisplay);
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, &MainWindow::updateMiniLoaderCrop);
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, [this](int, const QImage&) {
        if (m_debugTabActive) runDebugExport(true);
    });
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

    // Long-press menu on viewTracksButton to choose track display mode
    {
        QMenu* trackMenu = new QMenu(ui->viewTracksButton);
        QActionGroup* trackModeGroup = new QActionGroup(trackMenu);
        trackModeGroup->setExclusive(true);

        QAction* centroidAction = trackMenu->addAction("Centroid track");
        centroidAction->setCheckable(true);
        trackModeGroup->addAction(centroidAction);

        QAction* clMidAction = trackMenu->addAction("Centerline midpoint track");
        clMidAction->setCheckable(true);
        clMidAction->setChecked(true);
        trackModeGroup->addAction(clMidAction);

        ui->viewTracksButton->setMenu(trackMenu);
        ui->viewTracksButton->setPopupMode(QToolButton::DelayedPopup);

        connect(centroidAction, &QAction::triggered, this, [this]() {
            ui->videoLoader->setTrackDisplayMode(VideoLoader::TrackDisplayMode::Centroid);
        });
        connect(clMidAction, &QAction::triggered, this, [this]() {
            ui->videoLoader->setTrackDisplayMode(VideoLoader::TrackDisplayMode::CenterlineMidpoint);
        });
    }
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

    // Table View Selection -> auto-center video on worm position if zoomed in
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [this](const QItemSelection &selected, const QItemSelection &deselected) {
        Q_UNUSED(deselected)

        if (!selected.isEmpty()) {
            QModelIndexList selectedIndexes = selected.indexes();
            if (!selectedIndexes.isEmpty() && m_wormProxyModel) {
                QModelIndex proxyIdx = selectedIndexes.first();
                QModelIndex srcIdx = m_wormProxyModel->mapToSource(proxyIdx);
                if (srcIdx.isValid()) {
                    const TableItems::ClickedItem& selectedItem = m_blobTableModel->getItem(srcIdx.row());

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

    // Debug tab auto-export on worm selection change
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [this](const QItemSelection&, const QItemSelection&) {
        if (m_debugTabActive) runDebugExport(true);
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
    connect(ui->debugImageTable, &QTableWidget::itemSelectionChanged,
            this, &MainWindow::onDebugImageTableSelectionChanged);
    connect(ui->tabWidget, &QTabWidget::currentChanged, this, [this](int index) {
        const bool isDebug = (ui->tabWidget->widget(index) == ui->debugTab);
        onDebugTabChanged(isDebug);
    });
    if (m_appController) {
        connect(m_appController, &AppController::centerlineProgress,
                this, &MainWindow::onDebugCenterlineProgress);
        connect(m_appController, &AppController::centerlineFinished,
                this, &MainWindow::onDebugCenterlineFinished);
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

    // Debug control keyboard shortcut
    QShortcut* debugToggle = new QShortcut(QKeySequence("Ctrl+D"), this);
    connect(debugToggle, &QShortcut::activated, this, &MainWindow::toggleTrackingDebug);

    // Main tab switching: Analysis tab gets multi-select; others get single-select
    connect(ui->mainTabWidget, &QTabWidget::currentChanged,
            this, &MainWindow::onMainTabChanged);

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
    if (ui->miniLoader) ui->miniLoader->setShowSkeleton(checked);
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
    if (ui->miniLoader) ui->miniLoader->setShowSkeleton(newModes.testFlag(VideoLoader::ViewModeOption::Skeletons));

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

    // Load pixel size from this video's metadata JSON (or reset to 0 if absent).
    // VideoLoader has already created the data directory before emitting videoLoaded.
    m_currentVideoDataDir  = ui->videoLoader->getDataDirectory();
    m_currentVideoBaseName = QFileInfo(filePath).completeBaseName();
    {
        QSignalBlocker blocker(ui->pixelSizeSpinBoxD);
        double umPerPixel = 0.0;
        if (!m_currentVideoDataDir.isEmpty()
            && VideoMetadataStore::loadUmPerPixel(
                   m_currentVideoDataDir, m_currentVideoBaseName, umPerPixel)) {
            ui->pixelSizeSpinBoxD->setValue(umPerPixel);
        } else {
            ui->pixelSizeSpinBoxD->setValue(0.0);
        }
    }

    if (m_analysisDialog)
        m_analysisDialog->setVideoFps(m_videoFps);
    if (m_analysisPanel) {
        m_analysisPanel->setVideoFps(m_videoFps);
        // Trigger a (re-)scan of the yawt directory so the Analysis tree reflects
        // all available proc runs for this video's sibling videos.
        if (!m_currentVideoDataDir.isEmpty())
            m_analysisPanel->setYawtDirectory(m_currentVideoDataDir);
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
    if (!ui->miniLoader || currentFrame.isNull() || m_debugTabActive) {
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

    if (!foundCenterPoint) {
        ui->miniLoader->updateWithCroppedFrame(currentFrameNumber, QImage(), QPointF(), QSizeF(), QPointF());
        return;
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

        // Primary miniLoader (zoom-only) should receive the central frame for display consistency
        if (ui->miniLoader) {
            QImage cf_center;
            QPointF off_center;
            QSizeF sz_center;
            std::tie(cf_center, off_center, sz_center) = buildCroppedForFrame(currentFrameNumber);
            ui->miniLoader->updateWithCroppedFrame(currentFrameNumber, cf_center, off_center, sz_center, centerPoint);
            YAWT_DEBUG(lcGuiMainWindow) << "updateMiniLoaderCrop - sent center cropped frame to primary miniLoader, frame:" << currentFrameNumber;
        }

    }
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

void MainWindow::resultsButtonClicked()
{
    // Navigate to the Analysis tab (dialog-based analysis is deprecated)
    const int idx = ui->mainTabWidget->indexOf(ui->analysisTab);
    if (idx >= 0)
        ui->mainTabWidget->setCurrentIndex(idx);
}

void MainWindow::onMainTabChanged(int index)
{
    // The Analysis tab and the Processing tab are fully independent.
    // Disable the processing-side table views while Analysis is active so
    // no selection-change signals cross the tab boundary.
    const bool goingToAnalysis = (ui->mainTabWidget->widget(index) == ui->analysisTab);

    if (goingToAnalysis && !m_analysisTabActive) {
        m_analysisTabActive = true;
        ui->wormTableView->setEnabled(false);
        ui->roiTableView->setEnabled(false);
        // No rescan here — state is preserved across tab switches.
        // A rescan only happens when the data directory changes or tracking completes.

    } else if (!goingToAnalysis && m_analysisTabActive) {
        m_analysisTabActive = false;
        ui->wormTableView->setEnabled(true);
        ui->roiTableView->setEnabled(true);
    }
}

void MainWindow::frameSliderMoved(int value) {
    ui->videoLoader->seekToFrame(value, false);
    if (!ui->framePosition->hasFocus()) {
        ui->framePosition->setValue(value);
    }
    // Also update the mirrored spinbox if it's not being edited
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
        if (m_wormProxyModel && m_wormProxyModel->rowCount() > 0)
            ui->wormTableView->selectRow(0);
        // ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::EditTracks); // If desired
    }

    // Keep track data in storage for MiniLoader overlays and other components
    // Memory cleanup will be handled elsewhere if needed

    // Mark that we have completed tracking
    m_hasCompletedTracking = true;

    // Re-scan the yawt directory so the Analysis tree picks up the new proc folder.
    if (m_analysisPanel && !m_currentVideoDataDir.isEmpty())
        m_analysisPanel->setYawtDirectory(m_currentVideoDataDir);

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

    // Rebuild the centerline midpoint cache from the freshly computed blobs, then
    // repaint. Without this, the cache built at tracking-finish time contains only
    // the preliminary centerlines from the tracking phase, not the refined ones.
    if (ui->videoLoader && m_trackingDataStorage) {
        ui->videoLoader->setTracksToDisplay(m_trackingDataStorage->getAllTracks());
    } else if (ui->videoLoader) {
        ui->videoLoader->update();
    }

}

void MainWindow::runDebugExport(bool silent)
{
    // Resolve selected worm
    int wormId = -1;
    if (ui->wormTableView->selectionModel() && m_wormProxyModel && m_blobTableModel) {
        const QModelIndexList sel = ui->wormTableView->selectionModel()->selectedIndexes();
        if (!sel.isEmpty()) {
            const QModelIndex proxyIdx = m_wormProxyModel->index(sel.first().row(), 0);
            const QModelIndex srcIdx = m_wormProxyModel->mapToSource(proxyIdx);
            if (srcIdx.isValid())
                wormId = m_blobTableModel->getItem(srcIdx.row()).id;
        }
    }
    if (wormId < 0) {
        if (!silent) QMessageBox::information(this, "Export Process",
            "Select a worm in the worm table first.");
        return;
    }

    const int frame = ui->videoLoader ? ui->videoLoader->getCurrentFrameNumber() : -1;
    if (frame < 0) {
        if (!silent) QMessageBox::warning(this, "Export Process", "No current frame available.");
        return;
    }

    const QString dataDir = ui->videoLoader ? ui->videoLoader->getDataDirectory() : QString();
    if (dataDir.isEmpty()) {
        if (!silent) QMessageBox::warning(this, "Export Process",
            "No yawt data directory available. Load a video first.");
        return;
    }

    const QString videoBaseName = QFileInfo(
        ui->videoLoader ? ui->videoLoader->getCurrentVideoPath() : QString()).completeBaseName();
    if (videoBaseName.isEmpty()) {
        if (!silent) QMessageBox::warning(this, "Export Process",
            "No current video name available.");
        return;
    }

    const QString videoSpecificDir = QDir(dataDir).absoluteFilePath(videoBaseName);
    const QStringList procDirs = QDir(videoSpecificDir).entryList(
        QStringList() << "PROC_*", QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    if (procDirs.isEmpty()) {
        if (!silent) QMessageBox::warning(this, "Export Process",
            QString("No processing output directory found under:\n%1\n\nRun tracking first.")
                .arg(videoSpecificDir));
        return;
    }

    const QString outDir = QDir(QDir(videoSpecificDir).absoluteFilePath(procDirs.constLast()))
        .absoluteFilePath(QString("DEBUG/worm%1_frame%2").arg(wormId).arg(frame));
    if (!QDir().mkpath(outDir)) {
        if (!silent) QMessageBox::warning(this, "Export Process",
            QString("Could not create output directory:\n%1").arg(outDir));
        return;
    }

    ui->exportProcessButton->setEnabled(false);
    ui->centerlineDebugStatusLabel->setText(
        QString("worm %1  frame %2").arg(wormId).arg(frame));
    if (!silent) QApplication::processEvents();

    QString err;
    const bool ok = Debug::DebugExporter::exportCenterlineFrame(
        m_trackingDataStorage,
        m_appController ? m_appController->debugDataStore() : nullptr,
        wormId, frame, outDir, &err);

    ui->exportProcessButton->setEnabled(true);
    if (ok) {
        m_debugExportDir = outDir;
        populateDebugImageTable(outDir);
    } else {
        ui->centerlineDebugStatusLabel->setText(QString("Export failed: %1").arg(err));
        if (!silent) QMessageBox::warning(this, "Export Process",
            QString("Export failed: %1").arg(err));
    }
}

void MainWindow::onExportProcessClicked()
{
    runDebugExport(false);
}

void MainWindow::populateDebugImageTable(const QString& dir)
{
    ui->debugImageTable->clearContents();
    ui->debugImageTable->setRowCount(0);

    const QStringList pngFiles = QDir(dir).entryList(
        QStringList() << "*.png", QDir::Files, QDir::Name);

    ui->debugImageTable->setRowCount(pngFiles.size());
    for (int i = 0; i < pngFiles.size(); ++i) {
        auto* item = new QTableWidgetItem(QFileInfo(pngFiles[i]).completeBaseName());
        item->setData(Qt::UserRole, QDir(dir).absoluteFilePath(pngFiles[i]));
        ui->debugImageTable->setItem(i, 0, item);
    }
    ui->debugImageTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);

    if (!pngFiles.isEmpty())
        ui->debugImageTable->selectRow(0);
}

void MainWindow::onDebugImageTableSelectionChanged()
{
    const QList<QTableWidgetItem*> selected = ui->debugImageTable->selectedItems();
    if (selected.isEmpty() || !ui->miniLoader) return;

    const QString path = selected.first()->data(Qt::UserRole).toString();
    if (path.isEmpty()) return;

    const QImage img(path);
    if (img.isNull()) return;

    ui->miniLoader->updateWithCroppedFrame(
        -1, img, QPointF(0, 0),
        QSizeF(img.width(), img.height()), QPointF(img.width() / 2.0, img.height() / 2.0));
}

void MainWindow::onDebugTabChanged(bool active)
{
    m_debugTabActive = active;

    if (active) {
        runDebugExport(true);
    } else {
        // Restore normal miniLoader view
        if (ui->miniLoader && ui->videoLoader && ui->videoLoader->isVideoLoaded()) {
            const int frame = ui->videoLoader->getCurrentFrameNumber();
            const QImage img = ui->videoLoader->getCurrentQImageFrame();
            if (!img.isNull())
                updateMiniLoaderCrop(frame, img);
            else
                ui->miniLoader->updateWithCroppedFrame(-1, QImage(), QPointF(), QSizeF(), QPointF());
        } else if (ui->miniLoader) {
            ui->miniLoader->updateWithCroppedFrame(-1, QImage(), QPointF(), QSizeF(), QPointF());
        }
    }
}

// Retracking UI and logic removed
