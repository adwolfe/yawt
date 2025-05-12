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

    // Set a fixed size matching the QSS (important for border-radius)
    // If your QSS uses min/max width/height, setFixedSize isn't strictly
    // necessary, but it guarantees the size.
    //ui->playPauseButton->setFixedSize(50, 50);

    // --- Connect the toggled signal ---
    // This is where you put your existing play/pause logic
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
    //applyStyleSheet(ui->playPauseButton, ":/resources/qss/playpause.qss"); // Preferred method



    // Slots that update main window
    connect(ui->selectDirButton, &QToolButton::clicked, this, &MainWindow::chooseWorkingDirectory);
    connect(ui->videoLoader, &VideoLoader::videoLoaded, this, &MainWindow::initiateFrameDisplay);
    connect(ui->videoLoader, &VideoLoader::frameChanged, this, &MainWindow::updateFrameDisplay);

    // Slots that update the video loader
    connect(ui->videoTreeView, &VideoFileTreeView::videoFileDoubleClicked, ui->videoLoader, &VideoLoader::loadVideo);
    //connect(ui->playPauseButton, &QToolButton::clicked, ui->videoLoader, &VideoLoader::play);
    connect(ui->framePosition, &QSpinBox::valueChanged, this, &MainWindow::seekFrame);
    connect(ui->frameSlider, &QAbstractSlider::valueChanged, this, &MainWindow::frameSliderMoved);
    connect(ui->roiModeButton, &QToolButton::clicked, this, &MainWindow::roiModeToggle);
    connect(ui->panModeButton, &QToolButton::clicked, this, &MainWindow::panModeToggle);
    connect(ui->cropModeButton, &QToolButton::clicked, this, &MainWindow::cropModeToggle);
    connect(ui->threshModeButton, &QToolButton::clicked, this, &MainWindow::threshModeToggle);

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

void MainWindow::threshModeToggle()
{
    m_threshModeToggle = !m_threshModeToggle;
    ui->videoLoader->toggleThresholdView(m_threshModeToggle);

}

void MainWindow::applyStyleSheet(QWidget *widget, const QString &styleSheetPath) {
    QFile file(styleSheetPath);
    if (file.open(QFile::ReadOnly | QFile::Text)) {
        QString styleSheet = QLatin1String(file.readAll());
        widget->setStyleSheet(styleSheet);
        file.close();
    } else {
        qWarning() << "Could not open stylesheet file:" << styleSheetPath;
    }
}


MainWindow::~MainWindow()
{
    delete ui;
}
