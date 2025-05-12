#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QStandardPaths>
#include <QFileDialog>

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
    //ui->frameSlider->setTracking(0);



    // Slots that update main window
    connect(ui->selectDirButton, &QToolButton::clicked, this, &MainWindow::chooseWorkingDirectory);
    connect(ui->videoLoader, &VideoLoader::videoLoaded, this, &MainWindow::initiateFrameDisplay);
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, &MainWindow::updateFrameDisplay);

    // Slots that update the video loader
    connect(ui->videoTreeView, &VideoFileTreeView::videoFileDoubleClicked, ui->videoLoader, &VideoLoader::loadVideo);
    connect(ui->playButton, &QToolButton::clicked, ui->videoLoader, &VideoLoader::play);
    connect(ui->framePosition, &QSpinBox::valueChanged, this, &MainWindow::seekFrame);
    connect(ui->frameSlider, &QAbstractSlider::valueChanged, this, &MainWindow::frameSliderMoved);
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


MainWindow::~MainWindow()
{
    delete ui;
}
