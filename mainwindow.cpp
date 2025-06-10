#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "blobtablemodel.h"
#include "colordelegate.h"
#include "itemtypedelegate.h"
#include "trackingprogressdialog.h"
#include "trackingmanager.h"
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
    , roiFactorSpinBoxD(1.5) // Initialize ROI factor to default value 1.5
{
    ui->setupUi(this);

    // Model and Delegates
    m_blobTableModel = new BlobTableModel(this);
    ui->wormTableView->setModel(m_blobTableModel); // Assuming ui->wormTableView is your QTableView

    m_itemTypeDelegate = new ItemTypeDelegate(this);
    ui->wormTableView->setItemDelegateForColumn(BlobTableModel::Column::Type, m_itemTypeDelegate);

    m_colorDelegate = new ColorDelegate(this);
    ui->wormTableView->setItemDelegateForColumn(BlobTableModel::Column::Color, m_colorDelegate);

    ui->wormTableView->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    ui->wormTableView->horizontalHeader()->setStretchLastSection(true);

    // Tracking Manager
    m_trackingManager = new TrackingManager(this);

    setupInteractionModeButtonGroup(); // Setup for exclusive interaction mode buttons
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
    connect(ui->clearAllButton, &QPushButton::clicked, this, &MainWindow::handleRemoveBlobsClicked);

    // Table View Selection -> VideoLoader
    connect(ui->wormTableView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &MainWindow::updateVisibleTracksInVideoLoader);

    // Video File Tree View -> VideoLoader
    connect(ui->videoTreeView, &VideoFileTreeView::videoFileDoubleClicked, ui->videoLoader, &VideoLoader::loadVideo);

    // Tracking Process
    connect(ui->trackingDialogButton, &QPushButton::clicked, this, &MainWindow::onStartTrackingActionTriggered);
    connect(m_trackingManager, &TrackingManager::allTracksUpdated, this, &MainWindow::acceptTracksFromManager);
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
    // Set initial value in the model
    m_blobTableModel->updateRoiSizeMultiplier(roiFactorSpinBoxD);

    // Initial button states will be set by sync slots when VideoLoader emits initial modes
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
    m_blobTableModel->addItem(blobData.centroid, blobData.boundingBox, currentFrame, TableItems::ItemType::Worm);
}

void MainWindow::handleRemoveBlobsClicked() {
    m_blobTableModel->removeRows(0, m_blobTableModel->getAllItems().length());
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
    m_trackingProgressDialog->setTrackingParameters(videoPath, keyFrame, settings, initialWorms.size(), totalFrames);
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
    m_trackingManager->startFullTrackingProcess(videoPath, keyFrame, initialWorms, settings, totalFrames);
}

void MainWindow::handleCancelTrackingFromDialog() {
    if (m_trackingManager) m_trackingManager->cancelTracking();
}

void MainWindow::acceptTracksFromManager(const Tracking::AllWormTracks& tracks) {
    qDebug() << "MainWindow: Received" << tracks.size() << "tracks.";
    ui->videoLoader->setTracksToDisplay(tracks);
    if (!tracks.empty()) { // Optionally switch to tracks view
        ui->videoLoader->setViewModeOption(VideoLoader::ViewModeOption::Tracks, true);
        ui->wormTableView->selectAll();
        // ui->videoLoader->setInteractionMode(VideoLoader::InteractionMode::EditTracks); // If desired
    }
}

// --- Table View and VideoLoader Sync ---
void MainWindow::updateVisibleTracksInVideoLoader(const QItemSelection &selected, const QItemSelection &deselected) {
    Q_UNUSED(deselected);
    QSet<int> selectedItemIDs;
    if (ui->wormTableView && m_blobTableModel) {
        QItemSelectionModel *selectionModel = ui->wormTableView->selectionModel();
        if (selectionModel) {
            QModelIndexList selectedRowsIndexes = selectionModel->selectedRows();
            for (const QModelIndex &rowIdx : selectedRowsIndexes) {
                QModelIndex idModelIndex = m_blobTableModel->index(rowIdx.row(), BlobTableModel::Column::ID);
                if (idModelIndex.isValid()) {
                    bool conversionOk;
                    int itemId = m_blobTableModel->data(idModelIndex, Qt::DisplayRole).toInt(&conversionOk);
                    if (conversionOk) selectedItemIDs.insert(itemId);
                }
            }
        }
    }
    qDebug() << "MainWindow: Setting visible track/item IDs in VideoLoader:" << selectedItemIDs;
    ui->videoLoader->setVisibleTrackIDs(selectedItemIDs);
}
