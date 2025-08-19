#include "mainwindow.h"
#include <QTimer>
#include <QShortcut>
#include "debugutils.h"
#include "ui_mainwindow.h"
#include "annotationtablemodel.h"
#include "blobtablemodel.h"
#include "colordelegate.h"
#include "itemtypedelegate.h"
#include "retrackingdialog.h"
#include "trackingprogressdialog.h"
#include "trackingmanager.h"
#include "trackingdatastorage.h"
#include "version.h"
// No need to include videoloader.h again if it's in mainwindow.h, but good practice for .cpp
// #include "videoloader.h"
// #include "trackingcommon.h"

#include <QStandardPaths>
#include <QFileDialog>
#include <QIcon>
#include <QMessageBox>
#include <QDebug>
#include <QButtonGroup> // For m_interactionModeButtonGroup

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_blobTableModel(nullptr)
    , m_colorDelegate(nullptr)
    , m_itemTypeDelegate(nullptr)
    , m_trackingProgressDialog(nullptr)
    , m_trackingManager(nullptr)
    , m_interactionModeButtonGroup(new QButtonGroup(this))
    , m_trackingDataStorage(nullptr)
    , m_hasCompletedTracking(false)
    , roiFactorSpinBoxD(1.5) // Initialize ROI factor to default value 1.5
{
    ui->setupUi(this);

    // Initialize central data storage
    m_trackingDataStorage = new TrackingDataStorage(this);

    // Set up MiniVideoLoader with tracking data storage
    ui->miniVideoLoader->setTrackingDataStorage(m_trackingDataStorage);
    ui->miniVideoLoader->setVideoLoader(ui->videoLoader);

    // Model and Delegates
    m_blobTableModel = new BlobTableModel(m_trackingDataStorage, this);
    ui->wormTableView->setModel(m_blobTableModel); // Assuming ui->wormTableView is your QTableView

    // Annotation Table Model
    m_annotationTableModel = new AnnotationTableModel(m_trackingDataStorage, this);
    ui->annoTableView->setModel(m_annotationTableModel);
    qDebug() << "MainWindow: AnnotationTableModel created and connected to annoTableView";

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

    // Configure annotation table view
    ui->annoTableView->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    ui->annoTableView->horizontalHeader()->setStretchLastSection(false);
    ui->annoTableView->horizontalHeader()->setVisible(true);
    ui->annoTableView->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    ui->annoTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->annoTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    
    // Set column resize modes for annotation table
    ui->annoTableView->horizontalHeader()->setSectionResizeMode(AnnotationTableModel::ID, QHeaderView::ResizeToContents);
    ui->annoTableView->horizontalHeader()->setSectionResizeMode(AnnotationTableModel::Type, QHeaderView::ResizeToContents);
    ui->annoTableView->horizontalHeader()->setSectionResizeMode(AnnotationTableModel::Frames, QHeaderView::Stretch);
    
    // Add hover effects and cursor styling to indicate clickability
    ui->annoTableView->setMouseTracking(true);
    ui->annoTableView->viewport()->setCursor(Qt::PointingHandCursor);
    ui->annoTableView->setStyleSheet(
        "QTableView::item:hover { "
        "    background-color: #e3f2fd; "
        "    border: 1px solid #2196f3; "
        "} "
        "QTableView::item:selected { "
        "    background-color: #bbdefb; "
        "    color: #000; "
        "}"
    );
    
    qDebug() << "MainWindow: Annotation table view configured successfully";

    resizeTableColumns();

    // Tracking Manager - pass data storage
    m_trackingManager = new TrackingManager(m_trackingDataStorage, this);

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
    // ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::None, true); // Or start with nothing

    qDebug() << "Setting to global" << ui->globalThreshAutoCheck->checkState();
    ui->statusbar->showMessage(QString("Welcome to YAWT version ")+QString(PROJECT_VERSION)+" !", 10000);
}

MainWindow::~MainWindow()
{
    delete ui;
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
    // Connect ROI factor spinbox to BlobTableModel
    connect(ui->roiFactorSpinBoxD, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_blobTableModel, &BlobTableModel::updateRoiSizeMultiplier);

    // File/Directory
    connect(ui->selectDirButton, &QToolButton::clicked, this, &MainWindow::chooseWorkingDirectory);

    // VideoLoader basic signals
    connect(ui->videoLoader, &VideoLoader::videoLoaded, this, &MainWindow::initiateFrameDisplay);
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, &MainWindow::updateFrameDisplay);
    connect(ui->videoLoader, &VideoLoader::interactionModeChanged, this, &MainWindow::syncInteractionModeButtons);
    connect(ui->videoLoader, &VideoLoader::activeViewModesChanged, this, &MainWindow::syncViewModeOptionButtons); // Updated signal

    // Playback controls
    connect(ui->playPauseButton, &QToolButton::toggled, this, [this](bool checked) {
        if (checked) { ui->videoLoader->play(); ui->playPauseButton->setIcon(QIcon::fromTheme("media-playback-pause", QIcon(":/icons/pause.png"))); }
        else { ui->videoLoader->pause(); ui->playPauseButton->setIcon(QIcon::fromTheme("media-playback-start", QIcon(":/icons/play.png"))); }
    });
    connect(ui->firstFrameButton, &QToolButton::clicked, this, &MainWindow::goToFirstFrame);
    connect(ui->lastFrameButton, &QToolButton::clicked, this, &MainWindow::goToLastFrame);
    connect(ui->framePosition, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::seekFrame);
    connect(ui->frameSlider, &QAbstractSlider::valueChanged, this, &MainWindow::frameSliderMoved);

    // Interaction Mode Buttons -> VideoLoader (via slots that call VideoLoader)
    connect(ui->panModeButton, &QToolButton::clicked, this, &MainWindow::panModeButtonClicked);
    connect(ui->roiModeButton, &QToolButton::clicked, this, &MainWindow::roiModeButtonClicked);
    connect(ui->cropModeButton, &QToolButton::clicked, this, &MainWindow::cropModeButtonClicked);
    connect(ui->selectionModeButton, &QToolButton::clicked, this, &MainWindow::editBlobsModeButtonClicked);
    connect(ui->trackModeButton, &QToolButton::clicked, this, &MainWindow::editTracksModeButtonClicked);

    // View Mode Option Buttons (Checkable QToolButtons or QCheckBoxes) -> VideoLoader
    // Assuming ui->showThreshButton is checkable
    connect(ui->viewThreshButton, &QToolButton::toggled, this, &MainWindow::onViewThresholdToggled);
    // Add these connections once you have the buttons in your UI:
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
    connect(m_blobTableModel, &BlobTableModel::itemColorChanged, ui->videoLoader, &VideoLoader::updateWormColor);
    connect(m_blobTableModel, &BlobTableModel::itemVisibilityChanged,
            [this](int id, bool visible) {
                // When an item's visibility changes, update the VideoLoader with current items
                ui->videoLoader->updateItemsToDisplay(m_blobTableModel->getAllItems());
            });
    connect(ui->clearAllButton, &QPushButton::clicked, this, &MainWindow::handleRemoveBlobsClicked);
    connect(ui->clearFixButton, &QPushButton::clicked, this, &MainWindow::handleClearFixBlobsClicked);
    connect(ui->deleteButton, &QPushButton::clicked, this, &MainWindow::handleDeleteSelectedBlobClicked);

    // Retracking connections
    connect(ui->retrackButton, &QPushButton::clicked, this, &MainWindow::handleRetrackButtonClicked);

    // Auto-resize table columns when model data changes
    connect(m_blobTableModel, &BlobTableModel::dataChanged, this, &MainWindow::resizeTableColumns);
    connect(m_blobTableModel, &BlobTableModel::rowsInserted, this, &MainWindow::resizeTableColumns);
    connect(m_blobTableModel, &BlobTableModel::rowsRemoved, this, &MainWindow::resizeTableColumns);
    
    // Update retrack combo when blob data changes
    connect(m_blobTableModel, &BlobTableModel::rowsInserted, this, &MainWindow::updateRetrackBlobCombo);
    connect(m_blobTableModel, &BlobTableModel::rowsRemoved, this, &MainWindow::updateRetrackBlobCombo);
    connect(m_blobTableModel, &BlobTableModel::dataChanged, this, [this](const QModelIndex&, const QModelIndex&, const QList<int>& roles) {
        // Only update if the type column might have changed
        if (roles.isEmpty() || roles.contains(Qt::EditRole)) {
            updateRetrackBlobCombo();
        }
    });

    // Table View Selection -> VideoLoader
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &MainWindow::updateVisibleTracksInVideoLoader);

    // Enable/disable delete button based on selection state
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [this](const QItemSelection &selected, const QItemSelection &deselected) {
        ui->deleteButton->setEnabled(!selected.isEmpty());
    });

    // Table View Selection -> MiniVideoLoader
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [this](const QItemSelection &selected, const QItemSelection &deselected) {
        Q_UNUSED(deselected)

        if (selected.isEmpty()) {
            // No selection - clear the mini video loader
            ui->miniVideoLoader->clearSelection();
        } else {
            // Get the first selected row and extract the worm ID
            QModelIndexList selectedIndexes = selected.indexes();
            if (!selectedIndexes.isEmpty()) {
                int selectedRow = selectedIndexes.first().row();
                if (selectedRow >= 0 && selectedRow < m_blobTableModel->rowCount()) {
                    const TableItems::ClickedItem& selectedItem = m_blobTableModel->getItem(selectedRow);
                    ui->miniVideoLoader->setSelectedWorm(selectedItem.id);
                    
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



    // Main VideoLoader frame changes -> MiniVideoLoader
    connect(ui->videoLoader, &VideoLoader::frameChanged,
            ui->miniVideoLoader, static_cast<void(MiniVideoLoader::*)(int, const QImage&)>(&MiniVideoLoader::updateFrame));

    // Playback speed control
    connect(ui->comboPlaybackSpeed, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onPlaybackSpeedChanged);

    // Update combobox when VideoLoader speed changes (optional - for consistency)
    connect(ui->videoLoader, &VideoLoader::playbackSpeedChanged,
            this, &MainWindow::updatePlaybackSpeedComboBox);

    // Connect to headerClicked signal for handling Show/Hide column header click
    connect(ui->wormTableView->horizontalHeader(), &QHeaderView::sectionClicked,
            [this](int logicalIndex) {
        if (logicalIndex == BlobTableModel::Column::Show) {
            // Get current header state
            QVariant checkState = m_blobTableModel->headerData(
                BlobTableModel::Column::Show, Qt::Horizontal, Qt::CheckStateRole);

            // Toggle state
            if (checkState.isValid()) {
                Qt::CheckState newState;
                // If all or partial, make all unchecked. If none, make all checked.
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
    connect(m_trackingManager, &TrackingManager::allTracksUpdated, this, &MainWindow::acceptTracksFromManager);

    // Annotation table selection
    connect(ui->annoTableView, &QTableView::clicked, this, &MainWindow::onAnnotationTableClicked);

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
    ui->playPauseButton->setIcon(QIcon::fromTheme("media-playback-start", QIcon(":/icons/play.png")));

    // Initialize delete button to disabled state since there are no items selected initially
    ui->deleteButton->setEnabled(false);
    
    // Initialize Clear Fix button to disabled state until tracking is completed
    ui->clearFixButton->setEnabled(false);
    
    // Initialize retrack controls to disabled state until Fix blobs are available
    ui->comboRetrackBlobs->setEnabled(false);
    ui->retrackButton->setEnabled(false);

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
        modeMessage = "Blob Edit Mode: Click on blobs to add Fix markers for retracking";
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
// Optional:
// void MainWindow::onViewNoneClicked() {
//     ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Threshold, false);
//     ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Blobs, false);
//     ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Tracks, false);
// }


// --- Slots to sync UI buttons with VideoLoader's state ---
void MainWindow::syncInteractionModeButtons(VideoLoader::InteractionMode newMode) {
    // This will be handled by QButtonGroup if buttons are added to it correctly
    // Or, if not using QButtonGroup for these specific buttons for some reason:
    ui->panModeButton->setChecked(newMode == VideoLoader::InteractionMode::PanZoom);
    ui->roiModeButton->setChecked(newMode == VideoLoader::InteractionMode::DrawROI);
    ui->cropModeButton->setChecked(newMode == VideoLoader::InteractionMode::Crop);
    ui->selectionModeButton->setChecked(newMode == VideoLoader::InteractionMode::EditBlobs);
    ui->trackModeButton->setChecked(newMode == VideoLoader::InteractionMode::EditTracks);
    qDebug() << "MainWindow: Interaction mode UI synced to" << static_cast<int>(newMode);
}

void MainWindow::syncViewModeOptionButtons(VideoLoader::ViewModeOptions newModes) {
    // Update the checked state of your independent view mode buttons
    ui->viewThreshButton->setChecked(newModes.testFlag(VideoLoader::ViewModeOption::Threshold));
    // Assuming you have ui->viewBlobsButton and ui->viewTracksButton:
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
        QModelIndex newIndex = m_blobTableModel->index(lastRow, 0);
        ui->wormTableView->setCurrentIndex(newIndex);
        ui->wormTableView->selectionModel()->select(newIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

        // Resize columns to fit the new content
        resizeTableColumns();
        
        // Provide user feedback based on blob type
        QString feedbackMessage;
        // Always report as a Worm blob since auto-Fix behavior is disabled
        feedbackMessage = QString("Added Worm blob at frame %1").arg(currentFrame);
        statusBar()->showMessage(feedbackMessage, 3000);
        
        // Update retrack combo if a Fix blob was added
        // No retrack combo update needed for regular Worm blobs
    }
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

void MainWindow::handleClearFixBlobsClicked() {
    // Get all items from storage
    QList<TableItems::ClickedItem> allItems = m_trackingDataStorage->getAllItems();
    
    // Collect IDs of Fix type items to remove
    QList<int> fixItemIds;
    for (const auto& item : allItems) {
        if (item.type == TableItems::ItemType::Fix) {
            fixItemIds.append(item.id);
        }
    }
    
    // Remove Fix items
    for (int itemId : fixItemIds) {
        m_trackingDataStorage->removeItem(itemId);
    }
    
    // Update UI feedback
    if (!fixItemIds.isEmpty()) {
        statusBar()->showMessage(QString("Cleared %1 Fix blob(s)").arg(fixItemIds.size()), 3000);
    } else {
        statusBar()->showMessage("No Fix blobs to clear", 2000);
    }
    
    // Resize columns after potential changes
    resizeTableColumns();
    
    // Update retrack combo since we removed Fix blobs
    updateRetrackBlobCombo();
}

void MainWindow::handleDeleteSelectedBlobClicked() {
    // Get the currently selected row
    QModelIndexList selectedIndexes = ui->wormTableView->selectionModel()->selectedIndexes();

    // If there's a selection (should be at least one index per row)
    if (!selectedIndexes.isEmpty()) {
        // Get the row of the first selected index (we only allow single row selection)
        int selectedRow = selectedIndexes.first().row();

        // Remove the selected row
        m_blobTableModel->removeRows(selectedRow, 1);

        // Select the next row if available, or the previous row if this was the last one
        if (m_blobTableModel->rowCount() > 0) {
            int newRow = (selectedRow < m_blobTableModel->rowCount()) ? selectedRow : m_blobTableModel->rowCount() - 1;
            QModelIndex newIndex = m_blobTableModel->index(newRow, 0);
            ui->wormTableView->setCurrentIndex(newIndex);
            ui->wormTableView->selectionModel()->select(newIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
        }

        // The delete button will be automatically enabled/disabled by the selection changed handler

        // Make sure the table layout is updated
        resizeTableColumns();
        
        // Update retrack combo in case a Fix blob was deleted
        updateRetrackBlobCombo();
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
    ui->fpsLabel->setText(QString::number(fps, 'f', 2) + " fps");
    ui->videoNameLabel->setText(QFileInfo(filePath).fileName());
}

void MainWindow::updateFrameDisplay(int currentFrameNumber, const QImage& currentFrame) {
    Q_UNUSED(currentFrame);
    if (!ui->frameSlider->isSliderDown()) {
        ui->frameSlider->setValue(currentFrameNumber);
    }
    ui->framePosition->setValue(currentFrameNumber);
}

void MainWindow::frameSliderMoved(int value) {
    ui->videoLoader->seekToFrame(value, false);
    if (!ui->framePosition->hasFocus()) {
        ui->framePosition->setValue(value);
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
    if (ui->playPauseButton->isChecked())
    {
        ui->playPauseButton->setChecked(false);
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

    if (!m_trackingProgressDialog) {
        m_trackingProgressDialog = new TrackingProgressDialog(this);
        connect(m_trackingProgressDialog, &TrackingProgressDialog::beginTrackingRequested, this, &MainWindow::handleBeginTrackingFromDialog);
        connect(m_trackingProgressDialog, &TrackingProgressDialog::cancelTrackingRequested, this, &MainWindow::handleCancelTrackingFromDialog);
        connect(m_trackingManager, &TrackingManager::trackingStatusUpdate, m_trackingProgressDialog, &TrackingProgressDialog::updateStatusMessage);
        connect(m_trackingManager, &TrackingManager::overallTrackingProgress, m_trackingProgressDialog, &TrackingProgressDialog::updateOverallProgress);
        connect(m_trackingManager, &TrackingManager::trackingFinishedSuccessfully, m_trackingProgressDialog, &TrackingProgressDialog::onTrackingSuccessfullyFinished);
        connect(m_trackingManager, &TrackingManager::trackingFailed, m_trackingProgressDialog, &TrackingProgressDialog::onTrackingFailed);
        connect(m_trackingManager, &TrackingManager::trackingCancelled, m_trackingProgressDialog, &TrackingProgressDialog::onTrackingCancelledByManager);
    }
    //m_trackingProgressDialog->resetDialog();
    int wormsWithTracks = 0;
    if (m_trackingDataStorage) {
        QSet<int> itemsWithTracks = m_trackingDataStorage->getItemsWithTracks();
        for (const auto& info : initialWorms) {
            if (itemsWithTracks.contains(info.id)) ++wormsWithTracks;
        }
    }
    m_trackingProgressDialog->setTrackingParameters(videoPath, keyFrame, settings, initialWorms.size(), totalFrames, wormsWithTracks);
    m_trackingProgressDialog->exec();
}

void MainWindow::handleBeginTrackingFromDialog() {
    if (!m_trackingManager || !ui->videoLoader || !ui->videoLoader->isVideoLoaded()) {
        if(m_trackingProgressDialog) m_trackingProgressDialog->onTrackingFailed("Internal error: Components missing.");
        return;
    }
    QString videoPath = ui->videoLoader->getCurrentVideoPath();
    int keyFrame = ui->videoLoader->getCurrentFrameNumber();
    Thresholding::ThresholdSettings settings = ui->videoLoader->getCurrentThresholdSettings();
    int totalFrames = ui->videoLoader->getTotalFrames();

    std::vector<Tracking::InitialWormInfo> initialWorms;
    const QList<TableItems::ClickedItem>& items = m_blobTableModel->getAllItems();
    for(const TableItems::ClickedItem& item : items) {
        if(item.type == TableItems::ItemType::Worm) {
            Tracking::InitialWormInfo info; info.id = item.id; info.initialRoi = item.initialBoundingBox; info.color = item.color;
            initialWorms.push_back(info);
        }
    }
    if (initialWorms.empty()) {
        if(m_trackingProgressDialog) m_trackingProgressDialog->onTrackingFailed("No worms to track."); return;
    }

    // If the dialog requests only missing worms, filter out those that already have tracks.
    if (m_trackingProgressDialog && m_trackingProgressDialog->onlyTrackMissingChecked() && m_trackingDataStorage) {
        QSet<int> itemsWithTracks = m_trackingDataStorage->getItemsWithTracks();
        std::vector<Tracking::InitialWormInfo> filtered;
        filtered.reserve(initialWorms.size());
        for (const auto& iw : initialWorms) {
            if (!itemsWithTracks.contains(iw.id)) {
                filtered.push_back(iw);
            }
        }
        if (filtered.empty()) {
            if (m_trackingProgressDialog) m_trackingProgressDialog->onTrackingFailed("All selected worms already have tracks.");
            return;
        }
        initialWorms.swap(filtered);
    }

    QString dataDirectory = ui->videoLoader->getDataDirectory();
    m_trackingManager->startFullTrackingProcess(videoPath, dataDirectory, keyFrame, initialWorms, settings, totalFrames);
}

void MainWindow::handleCancelTrackingFromDialog() {
    if (m_trackingManager) m_trackingManager->cancelTracking();
}

void MainWindow::acceptTracksFromManager(const Tracking::AllWormTracks& tracks) {
    qDebug() << "MainWindow: Received" << tracks.size() << "tracks.";

    // Store tracks in the central data storage
    for (auto it = tracks.begin(); it != tracks.end(); ++it) {
        qDebug() << "MainWindow: Storing track for worm" << it->first << "with" << it->second.size() << "points";
        m_trackingDataStorage->setTrackForItem(it->first, it->second);
    }

    // Debug: Verify tracks were stored
    qDebug() << "MainWindow: TrackingDataStorage now has" << m_trackingDataStorage->getAllTracks().size() << "tracks";

    // Refresh annotation table to show lost tracking events
    if (m_annotationTableModel) {
        m_annotationTableModel->refreshAnnotations();
        qDebug() << "MainWindow: Refreshed annotation table with" << m_annotationTableModel->rowCount() << "annotations";
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

    // Keep track data in storage for MiniVideoLoader and other components
    // Memory cleanup will be handled elsewhere if needed

    // Mark that we have completed tracking
    m_hasCompletedTracking = true;
    
    // Enable Clear Fix button now that tracking is complete
    ui->clearFixButton->setEnabled(true);
    statusBar()->showMessage("Tracking completed - Fix blobs can now be added for retracking", 4000);
    
    // Update retrack blob combo box in case there are any Fix blobs
    updateRetrackBlobCombo();

    // Perform memory cleanup after tracking is complete
    performPostTrackingMemoryCleanup();
}

void MainWindow::setupPlaybackSpeedComboBox() {
    // Clear any existing items
    ui->comboPlaybackSpeed->clear();

    // Add speed options with display text and actual multiplier values
    ui->comboPlaybackSpeed->addItem("0.25x", 0.25);
    ui->comboPlaybackSpeed->addItem("0.5x", 0.5);
    ui->comboPlaybackSpeed->addItem("0.75x", 0.75);
    ui->comboPlaybackSpeed->addItem("1.0x", 1.0);
    //ui->comboPlaybackSpeed->addItem("1.25x", 1.25);
    ui->comboPlaybackSpeed->addItem("1.5x", 1.5);
    ui->comboPlaybackSpeed->addItem("2.0x", 2.0);
    ui->comboPlaybackSpeed->addItem("2.5x", 2.5);
    //ui->comboPlaybackSpeed->addItem("3.0x", 3.0);
    ui->comboPlaybackSpeed->addItem("5.0x", 5.0);
    ui->comboPlaybackSpeed->addItem("10.0x", 10.0);
    ui->comboPlaybackSpeed->addItem("20.0x", 20.0);
    // Set default to 1.0x (Normal)
    ui->comboPlaybackSpeed->setCurrentIndex(3);

    qDebug() << "MainWindow: Playback speed combobox initialized";
}

void MainWindow::onPlaybackSpeedChanged(int index) {
    if (index < 0 || index >= ui->comboPlaybackSpeed->count()) {
        return;
    }

    // Get the speed multiplier from the item data
    double speedMultiplier = ui->comboPlaybackSpeed->itemData(index).toDouble();

    qDebug() << "MainWindow: Setting playback speed to" << speedMultiplier << "x";

    // Set the speed in VideoLoader
    ui->videoLoader->setPlaybackSpeed(speedMultiplier);
}

void MainWindow::updatePlaybackSpeedComboBox(double speedMultiplier) {
    // Find the combobox item that matches this speed
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
    
    // Pause playback if it's currently playing
    if (ui->playPauseButton->isChecked()) {
        ui->playPauseButton->setChecked(false);
        ui->videoLoader->pause();
        ui->playPauseButton->setIcon(QIcon::fromTheme("media-playback-start", QIcon(":/icons/play.png")));
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
    
    // Find the row with matching worm ID
    for (int i = 0; i < m_blobTableModel->rowCount(); ++i) {
        const TableItems::ClickedItem& item = m_trackingDataStorage->getItemByIndex(i);
        if (item.id == targetWormId) {
            blobTableRow = i;
            break;
        }
    }
    
    // Select the row in the blob table if found
    if (blobTableRow >= 0) {
        QModelIndex blobIndex = m_blobTableModel->index(blobTableRow, 0);
        ui->wormTableView->setCurrentIndex(blobIndex);
        ui->wormTableView->selectionModel()->select(blobIndex, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
        ui->wormTableView->scrollTo(blobIndex, QAbstractItemView::EnsureVisible);
        qDebug() << "MainWindow: Selected worm" << targetWormId << "in blob table (row" << blobTableRow << ")";
    } else {
        // Clear selection if worm not found in blob table
        ui->wormTableView->clearSelection();
        qDebug() << "MainWindow: Worm" << targetWormId << "not found in blob table, cleared selection";
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

    // Create a set with all item IDs from the central data storage
    // Only the checkbox state (visible flag) will control actual display
    QSet<int> allItemIDs;

    if (m_trackingDataStorage) {
        // Get all IDs directly from the storage
        allItemIDs = m_trackingDataStorage->getAllItemIds();
    } else if (m_blobTableModel) {
        // Fallback to model if storage not available
        const QList<TableItems::ClickedItem>& allItems = m_blobTableModel->getAllItems();
        for (const auto& item : allItems) {
            allItemIDs.insert(item.id);
        }
    }

    qDebug() << "MainWindow: Setting all item IDs as visible in VideoLoader:" << allItemIDs;
    ui->videoLoader->setVisibleTrackIDs(allItemIDs);
}

void MainWindow::performPostTrackingMemoryCleanup() {
    qDebug() << "MainWindow: Performing post-tracking memory cleanup...";

    // Get memory usage before cleanup
    double cacheHitRate = ui->videoLoader->getCacheHitRate();
    int cacheSize = ui->videoLoader->getCacheSize();

    qDebug() << "MainWindow: VideoLoader cache status before cleanup - Size:" << cacheSize
             << "frames, Hit rate:" << QString::number(cacheHitRate, 'f', 1) << "%";

    // Reduce VideoLoader frame cache size significantly after tracking
    // During tracking, we don't need as many cached frames since we're not seeking rapidly
    int originalCacheSize = 50; // Default cache size
    int reducedCacheSize = 10;  // Smaller cache for post-tracking

    if (cacheSize > reducedCacheSize) {
        ui->videoLoader->setCacheSize(reducedCacheSize);
        qDebug() << "MainWindow: Reduced VideoLoader cache from" << cacheSize << "to" << reducedCacheSize << "frames";
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
    bool currentState = DebugUtils::isTrackingDebugEnabled();
    DebugUtils::setTrackingDebugEnabled(!currentState);
    QString status = DebugUtils::isTrackingDebugEnabled() ? "enabled" : "disabled";
    qDebug() << "MainWindow: Tracking debug messages" << status;
    statusBar()->showMessage(QString("Tracking debug messages %1").arg(status), 2000);
}

void MainWindow::updateRetrackBlobCombo() {
    // Clear existing items
    ui->comboRetrackBlobs->clear();
    
    // Get all items from storage
    QList<TableItems::ClickedItem> allItems = m_trackingDataStorage->getAllItems();
    
    // Find Fix type blobs and add them to combo
    QList<TableItems::ClickedItem> fixBlobs;
    for (const auto& item : allItems) {
        if (item.type == TableItems::ItemType::Fix) {
            fixBlobs.append(item);
        }
    }
    
    if (fixBlobs.isEmpty()) {
        ui->comboRetrackBlobs->addItem("No Fix blobs available");
        ui->comboRetrackBlobs->setEnabled(false);
        ui->retrackButton->setEnabled(false);
    } else {
        // Sort Fix blobs by ID for consistent display
        std::sort(fixBlobs.begin(), fixBlobs.end(), 
                  [](const TableItems::ClickedItem& a, const TableItems::ClickedItem& b) {
                      return a.id < b.id;
                  });
        
        // Add Fix blobs to combo box
        for (const auto& fixBlob : fixBlobs) {
            QString displayText = QString("Fix Blob ID %1 (Frame %2) at (%3, %4)")
                                    .arg(fixBlob.id)
                                    .arg(fixBlob.frameOfSelection)
                                    .arg(QString::number(fixBlob.initialCentroid.x(), 'f', 1))
                                    .arg(QString::number(fixBlob.initialCentroid.y(), 'f', 1));
            ui->comboRetrackBlobs->addItem(displayText, fixBlob.id); // Store ID as data
        }
        
        ui->comboRetrackBlobs->setEnabled(true);
        ui->retrackButton->setEnabled(true);
    }
}

void MainWindow::handleRetrackButtonClicked() {
    if (ui->comboRetrackBlobs->currentIndex() < 0) {
        statusBar()->showMessage("No Fix blob selected for retracking", 3000);
        return;
    }
    
    // Get the selected Fix blob ID
    bool ok = false;
    int fixBlobId = ui->comboRetrackBlobs->currentData().toInt(&ok);
    
    if (!ok) {
        statusBar()->showMessage("Invalid Fix blob selection", 3000);
        return;
    }
    
    // Find the selected Fix blob
    QList<TableItems::ClickedItem> allItems = m_trackingDataStorage->getAllItems();
    TableItems::ClickedItem selectedFixBlob;
    bool found = false;
    
    for (const auto& item : allItems) {
        if (item.id == fixBlobId && item.type == TableItems::ItemType::Fix) {
            selectedFixBlob = item;
            found = true;
            break;
        }
    }
    
    if (!found) {
        statusBar()->showMessage("Selected Fix blob not found", 3000);
        return;
    }
    
    // Create and show retracking dialog
    RetrackingDialog dialog(this);
    int totalFrames = ui->videoLoader->getTotalFrames();
    dialog.setFixBlobInfo(selectedFixBlob, totalFrames);
    
    if (dialog.exec() == QDialog::Accepted) {
        RetrackingParameters params = dialog.getRetrackingParameters();
        performRetracking(selectedFixBlob, params);
    }
}

void MainWindow::performRetracking(const TableItems::ClickedItem& fixBlob, const RetrackingParameters& params)
{
    statusBar()->showMessage(QString("Starting retracking for Fix Blob ID %1...").arg(fixBlob.id), 3000);
    
    // Check if we have a saved thresholded video to work with
    if (!m_trackingManager) {
        statusBar()->showMessage("No tracking manager available for retracking", 3000);
        return;
    }
    
    QString savedVideoPath = m_trackingManager->getSavedVideoPath();
    if (savedVideoPath.isEmpty()) {
        statusBar()->showMessage("No saved thresholded video available for retracking", 4000);
        QMessageBox::information(this, "Retracking Not Available", 
                                 "Retracking requires a saved thresholded video from the initial tracking process.\n"
                                 "Please run initial tracking first to enable retracking functionality.");
        return;
    }
    
    // Validate frame range
    int totalFrames = ui->videoLoader->getTotalFrames();
    if (params.startFrame >= totalFrames || params.endFrame >= totalFrames || 
        params.startFrame > params.endFrame) {
        statusBar()->showMessage("Invalid frame range for retracking", 3000);
        return;
    }
    
    qDebug() << "MainWindow: Starting retracking for Fix blob" << fixBlob.id 
             << "from frame" << params.startFrame << "to" << params.endFrame
             << "using saved video:" << savedVideoPath;
    
    // Create retracking parameters for the tracking manager
    QRectF initialROI = fixBlob.initialBoundingBox;
    
    // Start retracking process using the tracking manager
    bool retrackingStarted = m_trackingManager->startRetrackingProcess(
        savedVideoPath,
        fixBlob.id,
        initialROI,
        params.startFrame,
        params.endFrame,
        params.replaceExisting,
        params.extendTrack
    );
    
    if (retrackingStarted) {
        statusBar()->showMessage(QString("Retracking initiated for Fix Blob ID %1 (frames %2-%3)")
                                    .arg(fixBlob.id)
                                    .arg(params.startFrame)
                                    .arg(params.endFrame), 4000);
    } else {
        statusBar()->showMessage("Failed to start retracking process", 3000);
        QMessageBox::warning(this, "Retracking Failed", 
                             "Could not start the retracking process. Please check the console for error details.");
    }
}
