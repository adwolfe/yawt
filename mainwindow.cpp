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

    // Interact with video
    connect(ui->videoTreeView, &VideoFileTreeView::videoFileDoubleClicked, ui->videoLoader, &VideoLoader::loadVideo);
    connect(ui->playButton, &QToolButton::clicked, ui->videoLoader, &VideoLoader::play);
    connect(ui->framePosition, &QLineEdit::textChanged, this, &MainWindow::seekFrame);

    // Interact with main window
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, &MainWindow::updateFrameDisplay);
    connect(ui->selectDirButton, &QToolButton::clicked, this, &MainWindow::chooseWorkingDirectory);

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

void MainWindow::updateFrameDisplay(int currentFrameNumber, const QImage& currentFrame)
{
    ui->framePosition->setText(QString::number(currentFrameNumber));
}

void MainWindow::seekFrame(QString frame)
{
    ui->videoLoader->seekToFrame(frame.toInt());
}



MainWindow::~MainWindow()
{
    delete ui;
}
