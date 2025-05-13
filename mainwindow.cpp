#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QStandardPaths>
#include <QFileDialog>
#include <QIcon>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
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
    ui->adaptiveRadio->setChecked(true);
    ui->globalRadio->setChecked(false);
    ui->globalGroupBox->setVisible(false);
    ui->globalThreshAutoCheck->setChecked(false);

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
    connect(ui->globalThreshAutoCheck, &QCheckBox::clicked, this, &MainWindow::setGlobalOtsuMode);
    connect(ui->globalThreshSlider, &QAbstractSlider::valueChanged, this, &MainWindow::setGlobalThreshValue);
    connect(ui->globalThreshValueSpin, &QSpinBox::valueChanged, this, &MainWindow::setGlobalThreshValue);


    // Slots that update the video loader
    connect(ui->videoTreeView, &VideoFileTreeView::videoFileDoubleClicked, ui->videoLoader, &VideoLoader::loadVideo);
    //connect(ui->playPauseButton, &QToolButton::clicked, ui->videoLoader, &VideoLoader::play);
    connect(ui->framePosition, &QSpinBox::valueChanged, this, &MainWindow::seekFrame);
    connect(ui->frameSlider, &QAbstractSlider::valueChanged, this, &MainWindow::frameSliderMoved);
    connect(ui->roiModeButton, &QToolButton::clicked, this, &MainWindow::roiModeToggle);
    connect(ui->panModeButton, &QToolButton::clicked, this, &MainWindow::panModeToggle);
    connect(ui->cropModeButton, &QToolButton::clicked, this, &MainWindow::cropModeToggle);
    connect(ui->threshModeButton, &QToolButton::clicked, this, &MainWindow::threshModeViewToggle);

    // Close out by updating threshold settings in videoLoader

    ui->globalThreshSlider->setValue(50);
    ui->globalThreshAutoCheck->setChecked(true);
    setGlobalOtsuMode(true);

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

void MainWindow::updateThresholdModeSettings()
{
    auto senderObject = sender();
    if (senderObject == ui->adaptiveRadio) {
        ui->globalRadio->setChecked(false);
        ui->adaptiveGroupBox->setVisible(true);
        ui->globalGroupBox->setVisible(false);
        // update videoLoader settings
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

void MainWindow::setGlobalOtsuMode(bool checked)
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


MainWindow::~MainWindow()
{
    delete ui;
}
