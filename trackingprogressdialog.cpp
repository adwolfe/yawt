// trackingprogressdialog.cpp
#include "trackingprogressdialog.h" // Lowercase include
#include "ui_trackingprogressdialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QDialogButtonBox>
#include <QDebug>
#include <QMetaEnum>
#include <QFileInfo>

TrackingProgressDialog::TrackingProgressDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TrackingProgressDialog),
    m_keyFrame(-1),
    m_numberOfWorms(0),
    m_totalFramesInVideo(0),
    m_isTrackingRunning(false)
{
    ui->setupUi(this);
    setWindowTitle("Tracking Process");
    setModal(true);

    QPushButton *beginButton = ui->buttonBox->button(QDialogButtonBox::Apply);
    if (beginButton) {
        beginButton->setText("Begin Tracking");
        connect(beginButton, &QPushButton::clicked, this, &TrackingProgressDialog::onBeginButtonClicked);
    } else {
        beginButton = findChild<QPushButton*>("beginButton"); // Fallback if custom button
        if(beginButton) connect(beginButton, &QPushButton::clicked, this, &TrackingProgressDialog::onBeginButtonClicked);
        else qWarning() << "TrackingProgressDialog: Begin button not found.";
    }

    QPushButton *cancelButton = ui->buttonBox->button(QDialogButtonBox::Cancel);
    if (cancelButton) {
        connect(cancelButton, &QPushButton::clicked, this, &TrackingProgressDialog::onCancelButtonClicked);
    } else {
        cancelButton = findChild<QPushButton*>("cancelButton"); // Fallback
        if(cancelButton) connect(cancelButton, &QPushButton::clicked, this, &TrackingProgressDialog::onCancelButtonClicked);
        else qWarning() << "TrackingProgressDialog: Cancel button not found.";
    }

    ui->overallProgressBar->setValue(0);
    ui->overallProgressBar->setVisible(false);
    ui->statusLabel->setText("Ready to start tracking.");
    if(beginButton) beginButton->setEnabled(true);
}

TrackingProgressDialog::~TrackingProgressDialog() {
    delete ui;
}

void TrackingProgressDialog::setTrackingParameters(const QString& videoPath,
                                                   int keyFrame,
                                                   const ThresholdSettings& settings,
                                                   int numberOfWorms,
                                                   int totalFramesInVideo) {
    m_videoPath = videoPath;
    m_keyFrame = keyFrame;
    m_thresholdSettings = settings;
    m_numberOfWorms = numberOfWorms;
    m_totalFramesInVideo = totalFramesInVideo;

    QString summary;
    summary += "Video File: " + QFileInfo(videoPath).fileName() + "\n"; // Show only filename
    summary += "Total Frames: " + QString::number(totalFramesInVideo) + "\n";
    summary += "Keyframe for Initial Selection: " + QString::number(keyFrame) + "\n";
    summary += "Number of Worms Selected: " + QString::number(numberOfWorms) + "\n\n";
    summary += "Thresholding Settings:\n" + formatThresholdSettings(settings);

    ui->settingsSummaryTextEdit->setPlainText(summary);
    ui->settingsSummaryTextEdit->setReadOnly(true);

    m_isTrackingRunning = false;
    ui->overallProgressBar->setValue(0);
    ui->overallProgressBar->setVisible(false);
    ui->statusLabel->setText("Ready. Click 'Begin Tracking' to start.");

    QPushButton *beginButton = ui->buttonBox->button(QDialogButtonBox::Apply);
    if (!beginButton) beginButton = findChild<QPushButton*>("beginButton");
    if (beginButton) beginButton->setEnabled(true);

    QPushButton *cancelButton = ui->buttonBox->button(QDialogButtonBox::Cancel);
    if (!cancelButton) cancelButton = findChild<QPushButton*>("cancelButton");
    if (cancelButton) {
        cancelButton->setText("Cancel"); // Ensure it's "Cancel" initially
        cancelButton->setEnabled(true);
        // Ensure correct connection for initial state
        disconnect(cancelButton, &QPushButton::clicked, this, &TrackingProgressDialog::accept);
        disconnect(cancelButton, &QPushButton::clicked, this, &TrackingProgressDialog::reject);
        connect(cancelButton, &QPushButton::clicked, this, &TrackingProgressDialog::onCancelButtonClicked);
    }
}

QString TrackingProgressDialog::formatThresholdSettings(const ThresholdSettings& s) const {
    QString details;

    // Get the QMetaEnum for ThresholdAlgorithm using the meta-object of this class
    const QMetaObject* dialogMetaObject = this->metaObject(); // Or TrackingProgressDialog::staticMetaObject
    int enumIndex = dialogMetaObject->indexOfEnumerator("ThresholdAlgorithm");
    QMetaEnum metaEnum = dialogMetaObject->enumerator(enumIndex);

    QString algorithmStr = metaEnum.valueToKey(static_cast<int>(s.algorithm));
    if (algorithmStr.isEmpty()) { // Fallback if the key is not found
        algorithmStr = "Unknown Algorithm";
    }

    details += QString("  Background: %1\n").arg(s.assumeLightBackground ? "Light (Dark Worms)" : "Dark (Light Worms)");
    details += QString("  Algorithm: %1\n").arg(algorithmStr);

    switch (s.algorithm) {
    case ThresholdAlgorithm::Global:
        details += QString("    Value: %1\n").arg(s.globalThresholdValue);
        break;
    case ThresholdAlgorithm::Otsu:
        // Otsu is a type of global thresholding, value is automatic
        details += "    Value: Automatic (Otsu)\n";
        break;
    case ThresholdAlgorithm::AdaptiveMean:
        // details += "    Method: Adaptive Mean\n"; // Redundant if algorithmStr is "AdaptiveMean"
        details += QString("    Block Size: %1\n").arg(s.adaptiveBlockSize);
        details += QString("    C Value: %1\n").arg(s.adaptiveCValue);
        break;
    case ThresholdAlgorithm::AdaptiveGaussian:
        // details += "    Method: Adaptive Gaussian\n"; // Redundant
        details += QString("    Block Size: %1\n").arg(s.adaptiveBlockSize);
        details += QString("    C Value: %1\n").arg(s.adaptiveCValue);
        break;
    }
    details += QString("  Blur Enabled: %1\n").arg(s.enableBlur ? "Yes" : "No");
    if (s.enableBlur) {
        details += QString("    Blur Kernel Size: %1\n").arg(s.blurKernelSize);
        details += QString("    Blur Sigma X: %1%2\n").arg(s.blurSigmaX).arg(s.blurSigmaX == 0.0 ? " (Auto)" : "");
    }
    return details;
}


void TrackingProgressDialog::onBeginButtonClicked() {
    qDebug() << "Begin Tracking button clicked.";
    m_isTrackingRunning = true;
    ui->overallProgressBar->setVisible(true);
    ui->statusLabel->setText("Initiating tracking...");

    QPushButton *beginButton = ui->buttonBox->button(QDialogButtonBox::Apply);
    if (!beginButton) beginButton = findChild<QPushButton*>("beginButton");
    if (beginButton) beginButton->setEnabled(false);

    QPushButton *cancelButton = ui->buttonBox->button(QDialogButtonBox::Cancel);
    if (!cancelButton) cancelButton = findChild<QPushButton*>("cancelButton");
    if (cancelButton) cancelButton->setText("Cancel Tracking");

    emit beginTrackingRequested();
}

void TrackingProgressDialog::onCancelButtonClicked() {
    qDebug() << "Cancel button clicked. Tracking running:" << m_isTrackingRunning;
    if (m_isTrackingRunning) {
        ui->statusLabel->setText("Cancellation requested...");
        QPushButton *cancelButton = ui->buttonBox->button(QDialogButtonBox::Cancel);
        if (!cancelButton) cancelButton = findChild<QPushButton*>("cancelButton");
        if (cancelButton) cancelButton->setEnabled(false);
        emit cancelTrackingRequested();
    } else {
        reject(); // Close the dialog if tracking hasn't started
    }
}

void TrackingProgressDialog::updateStatusMessage(const QString& message) {
    ui->statusLabel->setText(message);
}

void TrackingProgressDialog::updateOverallProgress(int percentage) {
    if (!ui->overallProgressBar->isVisible()){
        ui->overallProgressBar->setVisible(true);
    }
    ui->overallProgressBar->setValue(percentage);
}

void TrackingProgressDialog::onTrackingSuccessfullyFinished() {
    m_isTrackingRunning = false;
    ui->statusLabel->setText("Tracking finished successfully!");
    ui->overallProgressBar->setValue(100);
    ui->overallProgressBar->setStyleSheet(""); // Reset stylesheet

    QPushButton *beginButton = ui->buttonBox->button(QDialogButtonBox::Apply);
    if (!beginButton) beginButton = findChild<QPushButton*>("beginButton");
    if (beginButton) beginButton->setEnabled(true); // Or change text to "View Results"

    QPushButton *cancelButton = ui->buttonBox->button(QDialogButtonBox::Cancel);
    if (!cancelButton) cancelButton = findChild<QPushButton*>("cancelButton");
    if (cancelButton) {
        cancelButton->setText("Close");
        cancelButton->setEnabled(true);
        disconnect(cancelButton, &QPushButton::clicked, this, &TrackingProgressDialog::onCancelButtonClicked);
        connect(cancelButton, &QPushButton::clicked, this, &TrackingProgressDialog::accept); // Close dialog
    }
}

void TrackingProgressDialog::onTrackingFailed(const QString& reason) {
    m_isTrackingRunning = false;
    ui->statusLabel->setText("Tracking Failed: " + reason);
    ui->overallProgressBar->setValue(ui->overallProgressBar->value()); // Keep current progress
    ui->overallProgressBar->setStyleSheet("QProgressBar::chunk { background-color: red; border-radius: 7px;}");

    QPushButton *beginButton = ui->buttonBox->button(QDialogButtonBox::Apply);
    if (!beginButton) beginButton = findChild<QPushButton*>("beginButton");
    if (beginButton) beginButton->setEnabled(true);

    QPushButton *cancelButton = ui->buttonBox->button(QDialogButtonBox::Cancel);
    if (!cancelButton) cancelButton = findChild<QPushButton*>("cancelButton");
    if (cancelButton) {
        cancelButton->setText("Close");
        cancelButton->setEnabled(true);
        disconnect(cancelButton, &QPushButton::clicked, this, &TrackingProgressDialog::onCancelButtonClicked);
        connect(cancelButton, &QPushButton::clicked, this, &TrackingProgressDialog::reject); // Close dialog
    }
}

void TrackingProgressDialog::onTrackingCancelledByManager() {
    m_isTrackingRunning = false;
    ui->statusLabel->setText("Tracking was cancelled.");
    // ui->overallProgressBar->setValue(0); // Optionally reset progress

    QPushButton *beginButton = ui->buttonBox->button(QDialogButtonBox::Apply);
    if (!beginButton) beginButton = findChild<QPushButton*>("beginButton");
    if (beginButton) beginButton->setEnabled(true);

    QPushButton *cancelButton = ui->buttonBox->button(QDialogButtonBox::Cancel);
    if (!cancelButton) cancelButton = findChild<QPushButton*>("cancelButton");
    if (cancelButton) {
        cancelButton->setText("Close");
        cancelButton->setEnabled(true);
        disconnect(cancelButton, &QPushButton::clicked, this, &TrackingProgressDialog::onCancelButtonClicked);
        connect(cancelButton, &QPushButton::clicked, this, &TrackingProgressDialog::reject); // Close dialog
    }
}

