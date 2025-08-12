#include "retrackingdialog.h"
#include "ui_retrackingdialog.h"
#include <QMessageBox>

RetrackingDialog::RetrackingDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::RetrackingDialog)
    , m_totalFrames(0)
{
    ui->setupUi(this);
    
    // Connect signals
    connect(ui->autoRangeCheck, &QCheckBox::toggled, this, &RetrackingDialog::onAutoRangeToggled);
    connect(ui->rangeSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &RetrackingDialog::onRangeSizeChanged);
    connect(ui->startFrameSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &RetrackingDialog::updateFrameRanges);
    connect(ui->endFrameSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &RetrackingDialog::updateFrameRanges);
    
    // Set initial state
    onAutoRangeToggled(ui->autoRangeCheck->isChecked());
    updateStatusLabel();
}

RetrackingDialog::~RetrackingDialog()
{
    delete ui;
}

void RetrackingDialog::setFixBlobInfo(const TableItems::ClickedItem& fixBlob, int totalFrames)
{
    m_fixBlob = fixBlob;
    m_totalFrames = totalFrames;
    
    // Update blob info display
    ui->blobIdValue->setText(QString::number(fixBlob.id));
    ui->blobFrameValue->setText(QString::number(fixBlob.frameOfSelection));
    ui->blobPositionValue->setText(QString("(%1, %2)")
                                   .arg(QString::number(fixBlob.initialCentroid.x(), 'f', 1))
                                   .arg(QString::number(fixBlob.initialCentroid.y(), 'f', 1)));
    
    // Set frame range limits
    ui->startFrameSpin->setMaximum(totalFrames - 1);
    ui->endFrameSpin->setMaximum(totalFrames - 1);
    
    // Update auto range if enabled
    if (ui->autoRangeCheck->isChecked()) {
        updateAutoRange();
    } else {
        // Set reasonable defaults
        ui->startFrameSpin->setValue(qMax(0, fixBlob.frameOfSelection - 50));
        ui->endFrameSpin->setValue(qMin(totalFrames - 1, fixBlob.frameOfSelection + 50));
    }
    
    updateStatusLabel();
}

::RetrackingParameters RetrackingDialog::getRetrackingParameters() const
{
    ::RetrackingParameters params;
    params.fixBlobId = m_fixBlob.id;
    params.startFrame = ui->startFrameSpin->value();
    params.endFrame = ui->endFrameSpin->value();
    params.autoRange = ui->autoRangeCheck->isChecked();
    params.rangeSize = ui->rangeSizeSpin->value();
    params.replaceExisting = ui->replaceExistingCheck->isChecked();
    params.extendTrack = ui->extendTrackCheck->isChecked();
    
    return params;
}

void RetrackingDialog::onAutoRangeToggled(bool enabled)
{
    // Enable/disable manual frame inputs
    ui->startFrameSpin->setEnabled(!enabled);
    ui->endFrameSpin->setEnabled(!enabled);
    ui->rangeSizeSpin->setEnabled(enabled);
    ui->labelRangeSize->setEnabled(enabled);
    
    if (enabled) {
        updateAutoRange();
    }
    
    updateStatusLabel();
}

void RetrackingDialog::updateFrameRanges()
{
    validateFrameRanges();
    updateStatusLabel();
}

void RetrackingDialog::onRangeSizeChanged()
{
    if (ui->autoRangeCheck->isChecked()) {
        updateAutoRange();
    }
}

void RetrackingDialog::updateAutoRange()
{
    if (m_fixBlob.id < 0 || m_totalFrames <= 0) {
        return;
    }
    
    int rangeSize = ui->rangeSizeSpin->value();
    int centerFrame = m_fixBlob.frameOfSelection;
    
    int startFrame = qMax(0, centerFrame - rangeSize);
    int endFrame = qMin(m_totalFrames - 1, centerFrame + rangeSize);
    
    ui->startFrameSpin->setValue(startFrame);
    ui->endFrameSpin->setValue(endFrame);
    
    updateStatusLabel();
}

void RetrackingDialog::validateFrameRanges()
{
    // Ensure start frame is not greater than end frame
    if (ui->startFrameSpin->value() > ui->endFrameSpin->value()) {
        // Adjust end frame to match start frame
        ui->endFrameSpin->setValue(ui->startFrameSpin->value());
    }
}

void RetrackingDialog::updateStatusLabel()
{
    if (m_fixBlob.id < 0) {
        ui->statusLabel->setText("No Fix blob selected");
        return;
    }
    
    int frameCount = ui->endFrameSpin->value() - ui->startFrameSpin->value() + 1;
    QString status = QString("Ready to retrack %1 frames (%2 to %3)")
                      .arg(frameCount)
                      .arg(ui->startFrameSpin->value())
                      .arg(ui->endFrameSpin->value());
    
    ui->statusLabel->setText(status);
}

void RetrackingDialog::accept()
{
    // Validate parameters before accepting
    if (m_fixBlob.id < 0) {
        QMessageBox::warning(this, "Invalid Parameters", "No Fix blob information available.");
        return;
    }
    
    if (ui->startFrameSpin->value() >= ui->endFrameSpin->value()) {
        QMessageBox::warning(this, "Invalid Frame Range", "Start frame must be less than end frame.");
        return;
    }
    
    if (ui->endFrameSpin->value() - ui->startFrameSpin->value() < 5) {
        QMessageBox::warning(this, "Invalid Frame Range", "Frame range is too small. Minimum 5 frames required.");
        return;
    }
    
    // All validation passed, accept the dialog
    QDialog::accept();
}