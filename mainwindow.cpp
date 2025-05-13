#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "itemtypedelegate.h"
#include "trackdata.h"

#include <QComboBox>
#include <QItemDelegate>
#include <QTableView>

#include <QStandardPaths>
#include <QFileDialog>
#include <QIcon>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow),m_trackingProgressDialog(nullptr), m_trackingManager(nullptr)
{
    ui->setupUi(this);
    QString initialDirectory = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    ui->videoTreeView->setRootDirectory(initialDirectory);
    ui->dirSelected->setText(initialDirectory);
    ui->framePosition->setKeyboardTracking(0);
    ui->frameSlider->setMinimum(0);
    ui->frameSlider->setSingleStep(10);
    ui->frameSlider->setPageStep(100);
    ui->playPauseButton->setObjectName("playPauseButton"); // Crucial for QSS selector
    ui->playPauseButton->setCheckable(true); // Make it toggle state
    ui->playPauseButton->setChecked(false);


    // Slots that update main window
    QObject::connect(ui->playPauseButton, &QToolButton::toggled, [&](bool checked) {
        if (checked) {
            qDebug() << "State changed to: Checked (Pause)";
            ui->videoLoader->play();
            ui->playPauseButton->setIcon(QIcon::fromTheme(QIcon::ThemeIcon::MediaPlaybackPause));
        } else {
            qDebug() << "State changed to: Unchecked (Play)";
            ui->videoLoader->pause();
            ui->playPauseButton->setIcon(QIcon::fromTheme(QIcon::ThemeIcon::MediaPlaybackStart));
        }
    });

    connect(ui->selectDirButton, &QToolButton::clicked, this, &MainWindow::chooseWorkingDirectory);
    connect(ui->videoLoader, &VideoLoader::videoLoaded, this, &MainWindow::initiateFrameDisplay);
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, &MainWindow::updateFrameDisplay);

    //Threshold settings slots
    connect(ui->globalRadio, &QRadioButton::clicked, this, &MainWindow::updateThresholdModeSettings);
    connect(ui->adaptiveRadio, &QRadioButton::clicked, this, &MainWindow::updateThresholdModeSettings);
    connect(ui->globalThreshAutoCheck, &QCheckBox::clicked, this, &MainWindow::setGlobalMode);
    connect(ui->globalThreshSlider, &QAbstractSlider::valueChanged, this, &MainWindow::setGlobalThreshValue);
    connect(ui->globalThreshValueSpin, &QSpinBox::valueChanged, this, &MainWindow::setGlobalThreshValue);
    connect(ui->adaptiveTypeCombo, &QComboBox::currentIndexChanged, this, &MainWindow::setAdaptiveMode);
    connect(ui->blockSizeSpin, &QSpinBox::valueChanged, this, &MainWindow::setBlockSize);
    connect(ui->tuningDoubleSpin, &QDoubleSpinBox::valueChanged, this, &MainWindow::setTuning);
    connect(ui->blurCheck, &QCheckBox::clicked, this, &MainWindow::setPreBlur);
    connect(ui->blurKernelSpin, &QSpinBox::valueChanged, this, &MainWindow::setBlurKernel);
    connect(ui->videoLoader, &VideoLoader::wormBlobSelected, this, &MainWindow::onWormBlobDetected);
    connect(ui->trackingDialogButton, &QPushButton::clicked, this, &MainWindow::onStartTrackingActionTriggered);




    // Slots that update the video loader
    connect(ui->videoTreeView, &VideoFileTreeView::videoFileDoubleClicked, ui->videoLoader, &VideoLoader::loadVideo);
    //connect(ui->playPauseButton, &QToolButton::clicked, ui->videoLoader, &VideoLoader::play);
    connect(ui->framePosition, &QSpinBox::valueChanged, this, &MainWindow::seekFrame);
    connect(ui->frameSlider, &QAbstractSlider::valueChanged, this, &MainWindow::frameSliderMoved);
    connect(ui->roiModeButton, &QToolButton::clicked, this, &MainWindow::roiModeToggle);
    connect(ui->panModeButton, &QToolButton::clicked, this, &MainWindow::panModeToggle);
    connect(ui->cropModeButton, &QToolButton::clicked, this, &MainWindow::cropModeToggle);
    connect(ui->threshModeButton, &QToolButton::clicked, this, &MainWindow::threshModeViewToggle);
    connect(ui->selectionModeButton, &QToolButton::clicked, this, &MainWindow::selectionModeToggle);
    connect(ui->bgCombo, &QComboBox::currentIndexChanged, this, &MainWindow::updateBackgroundColor);
    // Close out by updating threshold settings in videoLoader

    ui->globalThreshSlider->setValue(50);
    ui->globalThreshAutoCheck->setChecked(true);
    setGlobalMode(true);
    ui->adaptiveRadio->setChecked(true);
    ui->adaptiveTypeCombo->setCurrentIndex(0);
    ui->blockSizeSpin->setValue(5);
    setBlockSize(ui->blockSizeSpin->value());
    ui->tuningDoubleSpin->setValue(2.00);
    setTuning(ui->blurKernelSpin->value());
    ui->blurKernelSpin->setValue(5);
    setBlurKernel(ui->blurKernelSpin->value());
    ui->blurCheck->setChecked(true);
    setPreBlur(ui->blurCheck->isChecked());
    setAdaptiveMode(0);
    ui->globalGroupBox->setVisible(false);

    m_wormTableModel = new WormTableModel(this);

    ui->wormTableView->setModel(m_wormTableModel);

    ItemTypeDelegate *typeDelegate = new ItemTypeDelegate(this);

    ui->wormTableView->setItemDelegateForColumn(WormTableModel::Column::Type, typeDelegate);
    ui->wormTableView->setSizeAdjustPolicy(QTableView::AdjustToContents);

    m_trackingManager = new TrackingManager(this);

}

// SLOTS

void MainWindow::chooseWorkingDirectory()
// Selects the root directory for the Tree View structure.
{
    QString currDir = ui->dirSelected->text();
    QString selectedDir = QFileDialog::getExistingDirectory(this, "Choose working directory", currDir,  QFileDialog::ShowDirsOnly );

    if (!selectedDir.isEmpty()) {
        QFileInfo fileInfo(selectedDir);
        currDir = fileInfo.absoluteFilePath();
    }

    ui->dirSelected->setText(currDir);
    ui->videoTreeView->setRootDirectory(currDir);

}

void MainWindow::initiateFrameDisplay(const QString& filePath, int totalFrames, double fps, QSize frameSize)
// Called upon loading a video
{
    ui->frameSlider->setMaximum(totalFrames);
    ui->frameSlider->setValue(0);
    ui->framePosition->setMaximum(totalFrames);
    ui->framePosition->setValue(0);
    ui->fpsLabel->setText(QString::number(fps)+" fps");
}

void MainWindow::updateFrameDisplay(int currentFrameNumber, const QImage& currentFrame)
// When the video loader changes the frame (like, during playing)
{
    ui->framePosition->setValue(currentFrameNumber);
    ui->frameSlider->setSliderPosition(currentFrameNumber);
}

void MainWindow::frameSliderMoved(int value)
{
    ui->videoLoader->seekToFrame(value, 1);
    ui->framePosition->setValue(value);
}


void MainWindow::seekFrame(int frame)
{
    ui->videoLoader->seekToFrame(frame, 1);
    ui->frameSlider->setSliderPosition(frame);
}


void MainWindow::panModeToggle()
{
    ui->videoLoader->setInteractionMode(InteractionMode::PanZoom);
}
void MainWindow::roiModeToggle()
{
    ui->videoLoader->setInteractionMode(InteractionMode::DrawROI);
}
void MainWindow::cropModeToggle()
{
    ui->videoLoader->setInteractionMode(InteractionMode::Crop);
}

void MainWindow::threshModeViewToggle()
{
    m_threshModeToggle = !m_threshModeToggle;
    ui->videoLoader->toggleThresholdView(m_threshModeToggle);
}

void MainWindow::selectionModeToggle()
{
    //if(!m_threshModeToggle)
    //{
    //    m_threshModeToggle = true;
    //    ui->videoLoader->toggleThresholdView(true);
    //}
    ui->videoLoader->setInteractionMode(InteractionMode::SelectWorms);
}

void MainWindow::updateBackgroundColor(int index)
{
    ui->videoLoader->setAssumeLightBackground(index);
}

void MainWindow::updateThresholdModeSettings()
{
    auto senderObject = sender();
    if (senderObject == ui->adaptiveRadio) {
        ui->globalRadio->setChecked(false);
        ui->adaptiveGroupBox->setVisible(true);
        ui->globalGroupBox->setVisible(false);
        if (ui->adaptiveTypeCombo->currentIndex() == 0)
        {
            ui->videoLoader->setThresholdAlgorithm(ThresholdAlgorithm::AdaptiveGaussian);
        } else
            ui->videoLoader->setThresholdAlgorithm(ThresholdAlgorithm::AdaptiveMean);
    } else if (senderObject == ui->globalRadio){
        ui->adaptiveRadio->setChecked(false);
        ui->globalGroupBox->setVisible(true);
        ui->adaptiveGroupBox->setVisible(false);
        if(ui->globalThreshAutoCheck->isChecked())
        {
            ui->videoLoader->setThresholdAlgorithm(ThresholdAlgorithm::Otsu);
        } else
            ui->videoLoader->setThresholdAlgorithm(ThresholdAlgorithm::Global);
    }
}

void MainWindow::setGlobalThreshValue(int value)
{
    auto senderObject = sender();
    if (senderObject == ui->globalThreshSlider)
    {
        ui->globalThreshValueSpin->setValue(value);
    } else
        ui->globalThreshSlider->setValue(value);
    ui->videoLoader->setThresholdValue(value);
}

void MainWindow::setGlobalMode(bool checked)
{
    if (checked)
    {
        ui->globalThreshSlider->setDisabled(true);
        ui->globalThreshValueSpin->setDisabled(true);
        ui->videoLoader->setThresholdAlgorithm(ThresholdAlgorithm::Otsu);
    } else {
        ui->globalThreshSlider->setEnabled(true);
        ui->globalThreshValueSpin->setEnabled(true);
        ui->videoLoader->setThresholdAlgorithm(ThresholdAlgorithm::Global);
    }
}

void MainWindow::setAdaptiveMode(int value)
{
    if (value == 0)
    {
        ui->videoLoader->setThresholdAlgorithm(ThresholdAlgorithm::AdaptiveGaussian);
    } else
    {
        ui->videoLoader->setThresholdAlgorithm(ThresholdAlgorithm::AdaptiveMean);
    }
}

void MainWindow::setBlockSize(int value)
{
    ui->videoLoader->setAdaptiveThresholdBlockSize(value);
}
void MainWindow::setTuning(double value)
{
    ui->videoLoader->setAdaptiveThresholdC(value);
}
void MainWindow::setPreBlur(bool checked)
{
    ui->videoLoader->setEnableBlur(checked);
    if(checked)
    {
        ui->blurKernelSpin->setEnabled(true);
    } else {
        ui->blurKernelSpin->setDisabled(true);
    }
}
void MainWindow::setBlurKernel(int value)
{
    ui->videoLoader->setBlurKernelSize(value);
}

void MainWindow::onWormBlobDetected(const QPointF& centroid, const QRectF& bbox) {
    // Get current frame number from VideoLoader or store it when selection mode starts
    int currentFrame = ui->videoLoader->getCurrentFrameNumber(); // Assuming videoLoader is your VideoLoader instance
    m_wormTableModel->addItem(centroid, bbox, currentFrame, ItemType::Worm); // Default to "Worm"
}


void MainWindow::onStartTrackingActionTriggered() {
    // Inside MainWindow::onStartTrackingActionTriggered()

    qDebug() << "Attempting to start tracking action...";

    // 1. Check videoLoader instance
    if (!ui->videoLoader) { // Assuming 'videoLoader' is your VideoLoader instance pointer
        qCritical() << "FATAL: videoLoader is nullptr in onStartTrackingActionTriggered()!";
        QMessageBox::critical(this, "Error", "VideoLoader is not available. Cannot start tracking.");
        return;
    }
    qDebug() << "videoLoader instance seems valid.";

    // 2. Check if a video is actually loaded
    if (!ui->videoLoader->isVideoLoaded()) {
        qWarning() << "WARNING: No video loaded in VideoLoader.";
        QMessageBox::warning(this, "Tracking Setup", "No video is currently loaded. Please load a video first.");
        return;
    }
    qDebug() << "VideoLoader reports video is loaded.";

    // 3. Get the video path and THOROUGHLY check it
    QString videoPath = ui->videoLoader->getCurrentVideoPath(); // You confirmed this getter exists
    qDebug() << "Retrieved videoPath from videoLoader: \"" << videoPath << "\"";
    qDebug() << "videoPath.isNull():" << videoPath.isNull() << "videoPath.isEmpty():" << videoPath.isEmpty();

    // An empty string is usually fine for QString assignment, but a truly invalid one is not.
    // If videoPath is coming from an uninitialized or corrupted source, it's bad.
    if (videoPath.isNull()) { // isNull() is true for a default-constructed QString, which is fine.
        // However, if it's null AND you expected a path, that's a logic error upstream.
        qWarning() << "WARNING: videoPath is null. This might be okay if default, but check if a path was expected.";
    }
    if (videoPath.isEmpty() && ui->videoLoader->isVideoLoaded()) {
        // This is a strange state: video is loaded, but path is empty.
        qCritical() << "CRITICAL: Video is loaded, but videoPath is empty! Check VideoLoader::currentFilePath initialization and loadVideo logic.";
        QMessageBox::critical(this, "Error", "Video path is empty despite video being loaded. Cannot start tracking.");
        return;
    }
    // If after loading a video, videoPath is still empty, something is wrong in VideoLoader::loadVideo
    // or how currentFilePath is managed.

    // 4. Gather other parameters (and log them too for good measure)
    int keyFrame = ui->videoLoader->getCurrentFrameNumber();
    ThresholdSettings settings = ui->videoLoader->getCurrentThresholdSettings();
    int numWorms = m_wormTableModel->getAllItems().count(); // Example
    int totalFrames = ui->videoLoader->getTotalFrames();

    qDebug() << "Keyframe:" << keyFrame << "NumWorms:" << numWorms << "TotalFrames:" << totalFrames;
    // You can even log parts of the 'settings' struct if you suspect it.

    // 5. Ensure the dialog pointer is valid before use
    if (!m_trackingProgressDialog) {
        qDebug() << "Creating new TrackingProgressDialog instance.";
        m_trackingProgressDialog = new TrackingProgressDialog(this);
        // IMPORTANT: Connect signals only ONCE when the dialog is created.
        connect(m_trackingProgressDialog, &TrackingProgressDialog::beginTrackingRequested,
                this, &MainWindow::handleBeginTrackingFromDialog);
        connect(m_trackingProgressDialog, &TrackingProgressDialog::cancelTrackingRequested,
                this, &MainWindow::handleCancelTrackingFromDialog);
        qDebug() << "Preparing to connect TrackingManager signals to TrackingProgressDialog slots.";
        qDebug() << "MainWindow: m_trackingManager pointer is:" << m_trackingManager;
        qDebug() << "MainWindow: m_trackingProgressDialog pointer is:" << m_trackingProgressDialog;

        if (!m_trackingManager) {
            qCritical() << "FATAL: m_trackingManager is nullptr right before connect() at mainwindow.cpp line ~340!";
            QMessageBox::critical(this, "Critical Error", "TrackingManager is not initialized. Cannot proceed.");
            return; // Or handle error appropriately
        }

        if (!m_trackingProgressDialog) {
            qCritical() << "FATAL: m_trackingProgressDialog is nullptr right before connect() at mainwindow.cpp line ~340!";
            // This might happen if the dialog creation logic (if (!m_trackingProgressDialog) { ... }) has an issue
            // or if the dialog was unexpectedly deleted.
            QMessageBox::critical(this, "Critical Error", "TrackingProgressDialog is not initialized. Cannot proceed.");
            return; // Or handle error appropriately
        }

        qDebug() << "Both m_trackingManager and m_trackingProgressDialog appear to be non-null.";
        qDebug() << "Attempting connect at mainwindow.cpp line ~340...";

        if (m_trackingManager) { // Ensure trackingManager also exists
            connect(m_trackingManager, &TrackingManager::trackingStatusUpdate,
                    m_trackingProgressDialog, &TrackingProgressDialog::updateStatusMessage);
            connect(m_trackingManager, &TrackingManager::overallTrackingProgress,
                    m_trackingProgressDialog, &TrackingProgressDialog::updateOverallProgress);
            connect(m_trackingManager, &TrackingManager::trackingFinishedSuccessfully,
                    m_trackingProgressDialog, &TrackingProgressDialog::onTrackingSuccessfullyFinished);
            connect(m_trackingManager, &TrackingManager::trackingFailed,
                    m_trackingProgressDialog, &TrackingProgressDialog::onTrackingFailed);
            connect(m_trackingManager, &TrackingManager::trackingCancelled,
                    m_trackingProgressDialog, &TrackingProgressDialog::onTrackingCancelledByManager);
        } else {
            qCritical() << "FATAL: m_trackingManager is nullptr! Cannot connect progress dialog signals.";
            // Handle this error appropriately
            return;
        }
    } else {
        qDebug() << "Reusing existing TrackingProgressDialog instance.";
    }
    qDebug() << "m_trackingProgressDialog pointer:" << m_trackingProgressDialog;


    qDebug() << "Attempting to call m_trackingProgressDialog->setTrackingParameters(...)";
    // The actual call that leads to the crash
    m_trackingProgressDialog->setTrackingParameters(videoPath, keyFrame, settings, numWorms, totalFrames);
    qDebug() << "Successfully called setTrackingParameters. Showing dialog...";

    m_trackingProgressDialog->exec();
    qDebug() << "TrackingProgressDialog exec() finished.";
}


void MainWindow::handleBeginTrackingFromDialog() {
    // This slot is called when "Begin" is clicked in the dialog
    // Gather final parameters (especially InitialWormInfo from WormTableModel)
    QString videoPath = ui->videoLoader->getCurrentVideoPath();
    int keyFrame = ui->videoLoader->getCurrentFrameNumber(); // Or stored keyframe
    ThresholdSettings settings = ui->videoLoader->getCurrentThresholdSettings();
    int totalFrames = ui->videoLoader->getTotalFrames();

    std::vector<InitialWormInfo> initialWorms;
    const QList<TrackedItem>& items = m_wormTableModel->getAllItems();
    for(const TrackedItem& item : items) {
        // For now, only consider items marked as "Worm" as starting points for tracking
        if(item.type == ItemType::Worm) {
            InitialWormInfo info;
            info.id = item.id; // The model's auto-generated ID
            // The initial ROI for tracking will be the bounding box from selection
            info.initialRoi = item.initialBoundingBox;
            initialWorms.push_back(info);
        }
    }

    if (initialWorms.empty()) {
        QMessageBox::warning(this, "Tracking Error", "No worms selected or marked for tracking.");
        if(m_trackingProgressDialog) m_trackingProgressDialog->onTrackingFailed("No worms to track.");
        return;
    }

    // TODO: Get expected worm count from UI if you have that feature
    // int expectedWormCount = ui->expectedWormsSpinBox->value();

    m_trackingManager->startFullTrackingProcess(videoPath, keyFrame, initialWorms, settings, totalFrames);
}

void MainWindow::handleCancelTrackingFromDialog() {
    // This slot is called when "Cancel" is clicked in the dialog WHILE tracking is active
    if (m_trackingManager) { // And check if tracking is actually running
        m_trackingManager->cancelTracking();
    }
}


//void MainWindow::onDeleteSelectedWormClicked() {
//    QModelIndexList selectedRows = ui->wormTableView->selectionModel()->selectedRows();
//    // Sort rows in descending order to correctly remove multiple rows
//    std::sort(selectedRows.begin(), selectedRows.end(), [](const QModelIndex& a, const QModelIndex& b){
//        return a.row() > b.row();
//    });
//    for (const QModelIndex &index : selectedRows) {
//        m_wormTableModel->removeRows(index.row(), 1);
//    }
//}

MainWindow::~MainWindow()
{
    delete ui;
}
