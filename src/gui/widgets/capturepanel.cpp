#include "capturepanel.h"
#include "liveview.h"
#include "captureworker.h"
#include "cameraenumerator.h"
#include "../scaledialog.h"
#include "../../data/videometadatastore.h"

#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QDateTime>
#include <QDir>
#include <cmath>
#include <QMetaType>

CapturePanel::CapturePanel(QObject* parent)
    : QObject(parent)
    , m_worker(new CaptureWorker)
    , m_workerThread(new QThread(this))
{
    qRegisterMetaType<cv::Mat>();
    qRegisterMetaType<CameraParamSet>();
}

CapturePanel::~CapturePanel()
{
    QMetaObject::invokeMethod(m_worker, "stopCapture", Qt::BlockingQueuedConnection);
    m_workerThread->quit();
    m_workerThread->wait(3000);
}

// ---------------------------------------------------------------------------
// Setup — called once after setupUi()
// ---------------------------------------------------------------------------

void CapturePanel::setup(const Widgets& widgets)
{
    w = widgets;

    // Populate resolution combo with labelled size data
    w.resolutionCombo->addItem("640 × 480",   QSize(640,  480));
    w.resolutionCombo->addItem("1280 × 720",  QSize(1280, 720));
    w.resolutionCombo->addItem("1920 × 1080", QSize(1920, 1080));

    // Worker thread
    m_worker->moveToThread(m_workerThread);
    m_workerThread->start();

    // Worker → view
    connect(m_worker, &CaptureWorker::frameCaptured,
            w.liveView, &LiveView::updateFrame,      Qt::QueuedConnection);
    connect(m_worker, &CaptureWorker::cameraOpened,
            this,      &CapturePanel::onCameraOpened, Qt::QueuedConnection);
    connect(m_worker, &CaptureWorker::paramsReady,
            this,      &CapturePanel::onParamsReady,  Qt::QueuedConnection);
    connect(m_worker, &CaptureWorker::cameraError,
            this,      &CapturePanel::onCameraError,  Qt::QueuedConnection);
    connect(m_worker, &CaptureWorker::recordingStarted,
            this,      &CapturePanel::onRecordingStarted, Qt::QueuedConnection);
    connect(m_worker, &CaptureWorker::recordingStopped,
            this,      &CapturePanel::onRecordingStopped, Qt::QueuedConnection);
    connect(m_workerThread, &QThread::finished,
            m_worker,  &QObject::deleteLater);

    // ROI: LiveView → panel → worker
    connect(w.liveView, &LiveView::roiDefined, this, &CapturePanel::onRoiDefined);
    connect(w.liveView, &LiveView::roiCleared, this, &CapturePanel::onRoiCleared);

    // Camera controls
    connect(w.scanButton,        &QPushButton::clicked,
            this, &CapturePanel::onScanClicked);
    connect(w.connectButton,     &QPushButton::clicked,
            this, &CapturePanel::onConnectClicked);
    connect(w.disconnectButton,  &QPushButton::clicked,
            this, &CapturePanel::onDisconnectClicked);
    connect(w.recordButton,      &QPushButton::toggled,
            this, &CapturePanel::onRecordToggled);
    connect(w.setScaleButton,    &QPushButton::clicked,
            this, &CapturePanel::onSetScaleClicked);
    connect(w.drawRoiButton,     &QPushButton::toggled,
            this, &CapturePanel::onDrawRoiToggled);
    connect(w.clearRoiButton,    &QPushButton::clicked,
            this, &CapturePanel::onClearRoiClicked);
    connect(w.liveView, &LiveView::scaleMeasured,
            this, &CapturePanel::onScaleMeasured);

    // Parameter spinboxes
    connect(w.exposureSpin,      qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &CapturePanel::onExposureChanged);
    connect(w.exposureAutoCheck, &QCheckBox::toggled,
            this, &CapturePanel::onExposureAutoToggled);
    connect(w.gainSpin,          qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &CapturePanel::onGainChanged);
    connect(w.gainAutoCheck,     &QCheckBox::toggled,
            this, &CapturePanel::onGainAutoToggled);
    connect(w.gammaSpin,         qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &CapturePanel::onGammaChanged);
    connect(w.fpsSpin,           qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &CapturePanel::onFpsChanged);
    connect(w.brightnessSpin,    qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &CapturePanel::onBrightnessChanged);

    setConnectedState(false);
    // Do NOT auto-scan here. On macOS the probe loop briefly opens camera handles,
    // which triggers the system privacy sound even when the device is muted.
    // Scanning is deferred until the user navigates to the Capture tab
    // (see MainWindow::onCaptureTabActivated) or clicks Scan manually.
    w.statusLabel->setText("Click Scan to detect cameras.");
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void CapturePanel::setConnectedState(bool connected)
{
    m_connected = connected;
    w.connectButton->setEnabled(!connected);
    w.disconnectButton->setEnabled(connected);
    w.recordButton->setEnabled(connected);
    w.cameraCombo->setEnabled(!connected);
    w.resolutionCombo->setEnabled(!connected);
    w.scanButton->setEnabled(!connected);
    w.drawRoiButton->setEnabled(connected);

    if (!connected) {
        for (auto* s : { w.exposureSpin, w.gainSpin, w.gammaSpin,
                         w.fpsSpin, w.brightnessSpin })
            s->setEnabled(false);
        w.exposureAutoCheck->setEnabled(false);
        w.gainAutoCheck->setEnabled(false);
    }
}

void CapturePanel::applyParamInfo(QDoubleSpinBox* spin,
                                   QCheckBox*      autoBox,
                                   QLabel*         label,
                                   const ParamInfo& info)
{
    label->setEnabled(info.supported);
    spin->setEnabled(info.supported && !(autoBox && autoBox->isChecked()));
    if (autoBox) {
        autoBox->setVisible(info.hasAuto);
        autoBox->setEnabled(info.supported && info.hasAuto);
        if (info.hasAuto) {
            QSignalBlocker b(autoBox);
            autoBox->setChecked(info.autoEnabled);
            spin->setEnabled(info.supported && !info.autoEnabled);
        }
    }
    if (info.supported) {
        QSignalBlocker b(spin);
        spin->setRange(info.minVal, info.maxVal);
        spin->setSingleStep(info.step > 0 ? info.step : 1.0);
        spin->setValue(info.currentVal);
    }
}

QString CapturePanel::makeOutputPath() const
{
    const QString dir = m_outputDirectory.isEmpty() ? QDir::homePath() : m_outputDirectory;
    return QDir(dir).filePath(
        QString("capture_%1.mp4")
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")));
}

// ---------------------------------------------------------------------------
// Public slots
// ---------------------------------------------------------------------------

void CapturePanel::setOutputDirectory(const QString& dir)
{
    m_outputDirectory = dir;
    // Keep the output field in sync with the project folder so users always
    // know where recordings will land. Overwritten with the actual filename when
    // recording starts, and restored to the folder on stop.
    if (!m_recording)
        w.outputPathEdit->setText(dir);
}

// ---------------------------------------------------------------------------
// Camera slots
// ---------------------------------------------------------------------------

void CapturePanel::scanCameras()
{
    onScanClicked();
}

void CapturePanel::onScanClicked()
{
    w.statusLabel->setText("Scanning…");
    m_cameras = CameraEnumerator::enumerate();
    w.cameraCombo->clear();
    for (const CameraInfo& info : m_cameras)
        w.cameraCombo->addItem(info.displayName);
    if (m_cameras.isEmpty()) {
        w.cameraCombo->addItem("(no cameras found)");
        w.statusLabel->setText("No cameras found.");
    } else {
        w.statusLabel->setText(QString("%1 camera(s) found.").arg(m_cameras.size()));
    }
}

void CapturePanel::onConnectClicked()
{
    if (m_connected) return;
    const int idx = w.cameraCombo->currentIndex();
    if (idx < 0 || idx >= m_cameras.size()) return;

    const CameraInfo& info = m_cameras.at(idx);
    auto source = CameraEnumerator::create(info);
    if (!source) {
        w.statusLabel->setText("Failed to create camera source.");
        return;
    }

    QSize res = w.resolutionCombo->currentData().toSize();
    w.statusLabel->setText(QString("Opening %1…").arg(info.displayName));

    ICameraSource* rawSrc = source.release();
    QMetaObject::invokeMethod(m_worker,
        [rawSrc, ww = res.width(), hh = res.height(), worker = m_worker]() {
            worker->startCapture(rawSrc, ww, hh, 30.0);
        },
        Qt::QueuedConnection);
}

void CapturePanel::onDisconnectClicked()
{
    if (!m_connected) return;
    if (w.recordButton->isChecked())    w.recordButton->setChecked(false);
    if (w.drawRoiButton->isChecked())   w.drawRoiButton->setChecked(false);

    QMetaObject::invokeMethod(m_worker, "stopCapture", Qt::QueuedConnection);
    w.liveView->clear();
    setConnectedState(false);
    w.statusLabel->setText("Disconnected.");
}

void CapturePanel::onRecordToggled(bool checked)
{
    if (!m_connected) { w.recordButton->setChecked(false); return; }
    if (checked) {
        QString path = makeOutputPath();
        w.outputPathEdit->setText(path);
        QMetaObject::invokeMethod(m_worker, "startRecording",
                                  Qt::QueuedConnection, Q_ARG(QString, path));
    } else {
        QMetaObject::invokeMethod(m_worker, "stopRecording", Qt::QueuedConnection);
    }
}

void CapturePanel::onCameraOpened(int width, int height, double fps)
{
    setConnectedState(true);
    w.statusLabel->setText(
        QString("Connected — %1 × %2 @ %3 fps")
            .arg(width).arg(height).arg(fps, 0, 'f', 1));
    w.statusLabel->setStyleSheet("color: green; font-size: 11px;");
}

void CapturePanel::onParamsReady(CameraParamSet params)
{
    applyParamInfo(w.exposureSpin, w.exposureAutoCheck, w.exposureLabel, params.exposure);
    applyParamInfo(w.gainSpin,     w.gainAutoCheck,     w.gainLabel,     params.gain);
    applyParamInfo(w.gammaSpin,    nullptr,             w.gammaLabel,    params.gamma);
    applyParamInfo(w.brightnessSpin, nullptr,           w.brightnessLabel, params.brightness);

    // FPS is always enabled (software fallback guaranteed)
    {
        QSignalBlocker b(w.fpsSpin);
        w.fpsSpin->setRange(params.fps.minVal > 0 ? params.fps.minVal : 1.0,
                            params.fps.maxVal > 0 ? params.fps.maxVal : 120.0);
        w.fpsSpin->setSingleStep(1.0);
        w.fpsSpin->setValue(params.fps.currentVal > 0 ? params.fps.currentVal : 30.0);
    }
    w.fpsSpin->setEnabled(true);
    w.fpsLabel->setEnabled(true);

    if (params.fps.softwareOnly || !params.fps.supported) {
        w.fpsSpin->setToolTip(
            "Hardware FPS control not detected.\n"
            "The grab timer will be limited in software — the saved video\n"
            "plays back at this rate, but the camera may capture faster internally.");
        w.fpsLabel->setText("FPS (sw):");
    } else {
        w.fpsSpin->setToolTip("Acquisition frame rate (hardware-controlled).");
        w.fpsLabel->setText("FPS:");
    }

    if (params.exposure.supported)
        w.exposureLabel->setText(
            params.exposure.maxVal > 1000 ? "Exposure (µs):" : "Exposure:");
    if (params.gain.supported)
        w.gainLabel->setText(params.gain.maxVal <= 48 ? "Gain (dB):" : "Gain:");
}

void CapturePanel::onCameraError(const QString& message)
{
    setConnectedState(false);
    w.liveView->clear();
    w.statusLabel->setText(QString("Error: %1").arg(message));
    w.statusLabel->setStyleSheet("color: red; font-size: 11px;");
}

void CapturePanel::onRecordingStarted(const QString& path)
{
    m_recording = true;
    w.outputPathEdit->setText(path);
    w.statusLabel->setText(QString("● Recording → %1").arg(path));
    w.statusLabel->setStyleSheet("color: red; font-size: 11px;");

    // Stamp this video's metadata JSON with the current calibration so it loads
    // automatically when the file is later opened in the Processing tab.
    if (m_currentUmPerPixel > 0 && !m_outputDirectory.isEmpty()) {
        const QString dataDir  = QDir(m_outputDirectory).filePath("yawt");
        const QString baseName = QFileInfo(path).completeBaseName();
        VideoMetadataStore::saveUmPerPixel(dataDir, baseName, m_currentUmPerPixel);
    }
}

void CapturePanel::onRecordingStopped(int frameCount)
{
    const QString savedPath = w.outputPathEdit->text();
    w.statusLabel->setText(
        QString("Saved %1 frames → %2").arg(frameCount).arg(savedPath));
    w.statusLabel->setStyleSheet("color: green; font-size: 11px;");
    m_recording = false;
    w.recordButton->setChecked(false);
    // Restore folder path so the field remains informative between recordings.
    w.outputPathEdit->setText(m_outputDirectory);
}

// ---------------------------------------------------------------------------
// Parameter slots
// ---------------------------------------------------------------------------

void CapturePanel::onExposureChanged(double v)
{
    if (!m_connected || w.exposureAutoCheck->isChecked()) return;
    QMetaObject::invokeMethod(m_worker, "applyParam", Qt::QueuedConnection,
        Q_ARG(int, (int)CameraParam::ExposureUs), Q_ARG(double, v));
}
void CapturePanel::onExposureAutoToggled(bool checked)
{
    w.exposureSpin->setEnabled(!checked);
    if (!m_connected) return;
    QMetaObject::invokeMethod(m_worker, "applyParamAuto", Qt::QueuedConnection,
        Q_ARG(int, (int)CameraParam::ExposureUs), Q_ARG(bool, checked));
}
void CapturePanel::onGainChanged(double v)
{
    if (!m_connected || w.gainAutoCheck->isChecked()) return;
    QMetaObject::invokeMethod(m_worker, "applyParam", Qt::QueuedConnection,
        Q_ARG(int, (int)CameraParam::GainDb), Q_ARG(double, v));
}
void CapturePanel::onGainAutoToggled(bool checked)
{
    w.gainSpin->setEnabled(!checked);
    if (!m_connected) return;
    QMetaObject::invokeMethod(m_worker, "applyParamAuto", Qt::QueuedConnection,
        Q_ARG(int, (int)CameraParam::GainDb), Q_ARG(bool, checked));
}
void CapturePanel::onGammaChanged(double v)
{
    if (!m_connected) return;
    QMetaObject::invokeMethod(m_worker, "applyParam", Qt::QueuedConnection,
        Q_ARG(int, (int)CameraParam::Gamma), Q_ARG(double, v));
}
void CapturePanel::onFpsChanged(double v)
{
    if (!m_connected) return;
    QMetaObject::invokeMethod(m_worker, "applyParam", Qt::QueuedConnection,
        Q_ARG(int, (int)CameraParam::FPS), Q_ARG(double, v));
}
void CapturePanel::onBrightnessChanged(double v)
{
    if (!m_connected) return;
    QMetaObject::invokeMethod(m_worker, "applyParam", Qt::QueuedConnection,
        Q_ARG(int, (int)CameraParam::Brightness), Q_ARG(double, v));
}

// ---------------------------------------------------------------------------
// ROI slots
// ---------------------------------------------------------------------------

void CapturePanel::onDrawRoiToggled(bool checked)
{
    w.liveView->setRoiMode(checked);
}

void CapturePanel::onClearRoiClicked()
{
    w.liveView->clearRoi();
    QMetaObject::invokeMethod(m_worker, "clearRoi", Qt::QueuedConnection);
    w.clearRoiButton->setEnabled(false);
    w.roiLabel->clear();
}

void CapturePanel::onRoiDefined(QRectF roi)
{
    w.drawRoiButton->setChecked(false);
    w.clearRoiButton->setEnabled(true);
    w.roiLabel->setText(QString("ROI  %1 × %2 px  @ (%3, %4)")
        .arg((int)roi.width()).arg((int)roi.height())
        .arg((int)roi.x()).arg((int)roi.y()));
    QMetaObject::invokeMethod(m_worker, "setRoi", Qt::QueuedConnection,
                              Q_ARG(QRect, roi.toRect()));
}

void CapturePanel::onRoiCleared()
{
    w.clearRoiButton->setEnabled(false);
    w.roiLabel->clear();
    QMetaObject::invokeMethod(m_worker, "clearRoi", Qt::QueuedConnection);
}

// ---------------------------------------------------------------------------
// Scale calibration slots
// ---------------------------------------------------------------------------

void CapturePanel::onSetScaleClicked()
{
    ScaleDialog dlg(w.setScaleButton ? w.setScaleButton->window() : nullptr);
    if (dlg.exec() != QDialog::Accepted) return;

    m_scalePhysicalValue = dlg.physicalValue();
    m_scaleUnit          = dlg.unit();

    w.liveView->setScaleMeasureMode(true);
    w.scaleLabel->setText(
        QString("Draw a line = %1 %2")
            .arg(m_scalePhysicalValue, 0, 'g', 4)
            .arg(m_scaleUnit));
}

void CapturePanel::onScaleMeasured(double pixelLength)
{
    if (pixelLength < 1.0 || m_scaleUnit.isEmpty()) return;

    const double pixelsPerUnit = pixelLength / m_scalePhysicalValue;

    // Convert to canonical pixels/µm for storage and the spinbox signal.
    auto unitToUmFactor = [](const QString& u) -> double {
        if (u == "mm")   return 1000.0;
        if (u == "cm")   return 10000.0;
        if (u == "inch") return 25400.0;
        if (u == "µm")   return 1.0;
        return 0.0;
    };
    const double factor = unitToUmFactor(m_scaleUnit);
    m_currentUmPerPixel = (factor > 0 && pixelsPerUnit > 0) ? factor / pixelsPerUnit : 0.0;

    // Show result in toolbar label (µm/pixel convention, matching the spinbox)
    w.scaleLabel->setText(
        QString("Scale: %1 µm/px")
            .arg(m_currentUmPerPixel, 0, 'f', 2));

    // Save camera-level calibration file to the project yawt folder
    if (!m_outputDirectory.isEmpty()) {
        const QString dataDir = QDir(m_outputDirectory).filePath("yawt");
        VideoMetadataStore::ScaleCalibration cal;
        cal.pixelsPerUnit  = pixelsPerUnit;
        cal.unit           = m_scaleUnit;
        cal.physicalValue  = m_scalePhysicalValue;
        cal.pixelLength    = pixelLength;
        cal.timestamp      = QDateTime::currentDateTime();
        VideoMetadataStore::saveScale(dataDir, "capture_calibration", cal);
        // Also write the canonical µm/pixel for this project-level calibration file.
        if (m_currentUmPerPixel > 0)
            VideoMetadataStore::saveUmPerPixel(dataDir, "capture_calibration", m_currentUmPerPixel);
    }

    emit pixelScaleSet(pixelsPerUnit, m_scaleUnit);
}
