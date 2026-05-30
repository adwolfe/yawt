#include "capturepanel.h"
#include "liveview.h"
#include "captureworker.h"
#include "cameraenumerator.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QGroupBox>
#include <QDateTime>
#include <QDir>
#include <QMetaType>

CapturePanel::CapturePanel(QWidget* parent)
    : QWidget(parent)
    , m_worker(new CaptureWorker)
    , m_workerThread(new QThread(this))
{
    qRegisterMetaType<cv::Mat>();
    qRegisterMetaType<CameraParamSet>();

    buildUi();

    m_worker->moveToThread(m_workerThread);
    m_workerThread->start();

    connect(m_worker, &CaptureWorker::frameCaptured,
            m_liveView, &LiveView::updateFrame,     Qt::QueuedConnection);
    connect(m_worker, &CaptureWorker::cameraOpened,
            this,     &CapturePanel::onCameraOpened, Qt::QueuedConnection);
    connect(m_worker, &CaptureWorker::paramsReady,
            this,     &CapturePanel::onParamsReady,  Qt::QueuedConnection);
    connect(m_worker, &CaptureWorker::cameraError,
            this,     &CapturePanel::onCameraError,  Qt::QueuedConnection);
    connect(m_worker, &CaptureWorker::recordingStarted,
            this,     &CapturePanel::onRecordingStarted, Qt::QueuedConnection);
    connect(m_worker, &CaptureWorker::recordingStopped,
            this,     &CapturePanel::onRecordingStopped, Qt::QueuedConnection);
    connect(m_workerThread, &QThread::finished,
            m_worker, &QObject::deleteLater);

    // ROI from LiveView → worker
    connect(m_liveView, &LiveView::roiDefined,  this, &CapturePanel::onRoiDefined);
    connect(m_liveView, &LiveView::roiCleared,  this, &CapturePanel::onRoiCleared);

    setConnectedState(false);
    onScanClicked();
}

CapturePanel::~CapturePanel()
{
    QMetaObject::invokeMethod(m_worker, "stopCapture", Qt::BlockingQueuedConnection);
    m_workerThread->quit();
    m_workerThread->wait(3000);
}

// ---------------------------------------------------------------------------
// UI construction
// ---------------------------------------------------------------------------

void CapturePanel::buildUi()
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(4, 4, 4, 4);
    root->setSpacing(4);

    // ---- ROI toolbar (above LiveView) ----
    auto* roiBar = new QHBoxLayout;
    m_drawRoiButton = new QPushButton("Draw ROI", this);
    m_drawRoiButton->setCheckable(true);
    m_drawRoiButton->setToolTip("Click and drag in the view to define a capture ROI");
    m_clearRoiButton = new QPushButton("Clear ROI", this);
    m_clearRoiButton->setEnabled(false);
    m_roiLabel = new QLabel(this);
    m_roiLabel->setStyleSheet("color: #c8a800; font-size: 11px;");
    roiBar->addWidget(m_drawRoiButton);
    roiBar->addWidget(m_clearRoiButton);
    roiBar->addWidget(m_roiLabel, 1);
    roiBar->addStretch();
    root->addLayout(roiBar);

    // ---- LiveView ----
    m_liveView = new LiveView(this);
    root->addWidget(m_liveView, 1);

    // ---- Camera controls ----
    auto* camGroup = new QGroupBox("Camera", this);
    auto* camLayout = new QVBoxLayout(camGroup);
    camLayout->setSpacing(4);

    auto* row1 = new QHBoxLayout;
    row1->addWidget(new QLabel("Camera:", this));
    m_cameraCombo = new QComboBox(this);
    m_cameraCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    row1->addWidget(m_cameraCombo, 1);
    m_scanButton = new QPushButton("Scan", this);
    row1->addWidget(m_scanButton);
    camLayout->addLayout(row1);

    auto* row2 = new QHBoxLayout;
    row2->addWidget(new QLabel("Resolution:", this));
    m_resolutionCombo = new QComboBox(this);
    m_resolutionCombo->addItem("640 × 480",   QSize(640,  480));
    m_resolutionCombo->addItem("1280 × 720",  QSize(1280, 720));
    m_resolutionCombo->addItem("1920 × 1080", QSize(1920, 1080));
    row2->addWidget(m_resolutionCombo);
    row2->addStretch();
    m_connectButton    = new QPushButton("Connect",    this);
    m_disconnectButton = new QPushButton("Disconnect", this);
    row2->addWidget(m_connectButton);
    row2->addWidget(m_disconnectButton);
    camLayout->addLayout(row2);

    auto* row3 = new QHBoxLayout;
    m_recordButton = new QPushButton("Record", this);
    m_recordButton->setCheckable(true);
    row3->addWidget(m_recordButton);
    row3->addWidget(new QLabel("Output:", this));
    m_outputPathEdit = new QLineEdit(this);
    m_outputPathEdit->setReadOnly(true);
    m_outputPathEdit->setPlaceholderText("(project folder not set)");
    row3->addWidget(m_outputPathEdit, 1);
    camLayout->addLayout(row3);

    m_statusLabel = new QLabel(this);
    m_statusLabel->setStyleSheet("color: gray; font-size: 11px;");
    camLayout->addWidget(m_statusLabel);
    root->addWidget(camGroup);

    // ---- Camera parameters ----
    m_paramsGroup = new QGroupBox("Camera Settings", this);
    m_paramsGroup->setCheckable(false);
    auto* form = new QFormLayout(m_paramsGroup);
    form->setSpacing(4);
    form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

    auto makeParamRow = [&](const QString& label,
                             QDoubleSpinBox*& spin,
                             QCheckBox*& autoBox,
                             QLabel*& lbl) {
        spin    = new QDoubleSpinBox(this);
        spin->setDecimals(1);
        spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        autoBox = new QCheckBox("Auto", this);
        autoBox->setVisible(false);
        lbl = new QLabel(label, this);

        auto* row = new QHBoxLayout;
        row->addWidget(spin, 1);
        row->addWidget(autoBox);
        form->addRow(lbl, row);
    };

    auto makeParamRowNoAuto = [&](const QString& label,
                                   QDoubleSpinBox*& spin,
                                   QLabel*& lbl) {
        spin    = new QDoubleSpinBox(this);
        spin->setDecimals(2);
        spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        lbl = new QLabel(label, this);
        form->addRow(lbl, spin);
    };

    makeParamRow("Exposure:", m_exposureSpin, m_exposureAutoCheck, m_exposureLabel);
    makeParamRow("Gain:",     m_gainSpin,     m_gainAutoCheck,     m_gainLabel);
    makeParamRowNoAuto("Gamma:",      m_gammaSpin,      m_gammaLabel);
    makeParamRowNoAuto("FPS:",        m_fpsSpin,        m_fpsLabel);
    makeParamRowNoAuto("Brightness:", m_brightnessSpin, m_brightnessLabel);

    root->addWidget(m_paramsGroup);

    // ---- Wire buttons ----
    connect(m_scanButton,        &QPushButton::clicked,         this, &CapturePanel::onScanClicked);
    connect(m_connectButton,     &QPushButton::clicked,         this, &CapturePanel::onConnectClicked);
    connect(m_disconnectButton,  &QPushButton::clicked,         this, &CapturePanel::onDisconnectClicked);
    connect(m_recordButton,      &QPushButton::toggled,         this, &CapturePanel::onRecordToggled);
    connect(m_drawRoiButton,     &QPushButton::toggled,         this, &CapturePanel::onDrawRoiToggled);
    connect(m_clearRoiButton,    &QPushButton::clicked,         this, &CapturePanel::onClearRoiClicked);

    // ---- Wire parameter spinboxes ----
    connect(m_exposureSpin,      qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &CapturePanel::onExposureChanged);
    connect(m_exposureAutoCheck, &QCheckBox::toggled,
            this, &CapturePanel::onExposureAutoToggled);
    connect(m_gainSpin,          qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &CapturePanel::onGainChanged);
    connect(m_gainAutoCheck,     &QCheckBox::toggled,
            this, &CapturePanel::onGainAutoToggled);
    connect(m_gammaSpin,         qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &CapturePanel::onGammaChanged);
    connect(m_fpsSpin,           qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &CapturePanel::onFpsChanged);
    connect(m_brightnessSpin,    qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, &CapturePanel::onBrightnessChanged);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void CapturePanel::applyParamInfo(QDoubleSpinBox* spin,
                                   QCheckBox*      autoBox,
                                   QLabel*         label,
                                   const ParamInfo& info)
{
    const bool ok = info.supported;
    spin->setEnabled(ok && !(autoBox && autoBox->isChecked()));
    label->setEnabled(ok);
    if (autoBox) {
        autoBox->setVisible(info.hasAuto);
        autoBox->setEnabled(ok && info.hasAuto);
        if (info.hasAuto) {
            QSignalBlocker b(autoBox);
            autoBox->setChecked(info.autoEnabled);
            spin->setEnabled(ok && !info.autoEnabled);
        }
    }
    if (ok) {
        QSignalBlocker b(spin);
        spin->setRange(info.minVal, info.maxVal);
        spin->setSingleStep(info.step > 0 ? info.step : 1.0);
        spin->setValue(info.currentVal);
    }
}

void CapturePanel::setConnectedState(bool connected)
{
    m_connected = connected;
    m_connectButton->setEnabled(!connected);
    m_disconnectButton->setEnabled(connected);
    m_recordButton->setEnabled(connected);
    m_cameraCombo->setEnabled(!connected);
    m_resolutionCombo->setEnabled(!connected);
    m_scanButton->setEnabled(!connected);
    m_drawRoiButton->setEnabled(connected);

    // Disable all param controls until we know what's supported
    if (!connected) {
        for (auto* w : { m_exposureSpin, m_gainSpin, m_gammaSpin,
                         m_fpsSpin, m_brightnessSpin })
            w->setEnabled(false);
        m_exposureAutoCheck->setEnabled(false);
        m_gainAutoCheck->setEnabled(false);
    }
}

QString CapturePanel::makeOutputPath() const
{
    const QString dir = m_outputDirectory.isEmpty() ? QDir::homePath() : m_outputDirectory;
    const QString ts  = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    return QDir(dir).filePath(QString("capture_%1.mp4").arg(ts));
}

// ---------------------------------------------------------------------------
// Camera slots
// ---------------------------------------------------------------------------

void CapturePanel::setOutputDirectory(const QString& dir)
{
    m_outputDirectory = dir;
}

void CapturePanel::onScanClicked()
{
    m_statusLabel->setText("Scanning…");
    m_cameras = CameraEnumerator::enumerate();
    m_cameraCombo->clear();
    for (const CameraInfo& info : m_cameras)
        m_cameraCombo->addItem(info.displayName);
    if (m_cameras.isEmpty()) {
        m_cameraCombo->addItem("(no cameras found)");
        m_statusLabel->setText("No cameras found.");
    } else {
        m_statusLabel->setText(QString("%1 camera(s) found.").arg(m_cameras.size()));
    }
    m_statusLabel->setStyleSheet("color: gray; font-size: 11px;");
}

void CapturePanel::onConnectClicked()
{
    if (m_connected) return;
    const int idx = m_cameraCombo->currentIndex();
    if (idx < 0 || idx >= m_cameras.size()) return;

    const CameraInfo& info = m_cameras.at(idx);
    auto source = CameraEnumerator::create(info);
    if (!source) {
        m_statusLabel->setText("Failed to create camera source.");
        m_statusLabel->setStyleSheet("color: red; font-size: 11px;");
        return;
    }

    QSize res = m_resolutionCombo->currentData().toSize();
    m_statusLabel->setText(QString("Opening %1…").arg(info.displayName));

    ICameraSource* rawSrc = source.release();
    QMetaObject::invokeMethod(m_worker,
        [rawSrc, w = res.width(), h = res.height(), worker = m_worker]() {
            worker->startCapture(rawSrc, w, h, 30.0);
        },
        Qt::QueuedConnection);
}

void CapturePanel::onDisconnectClicked()
{
    if (!m_connected) return;
    if (m_recordButton->isChecked()) m_recordButton->setChecked(false);
    if (m_drawRoiButton->isChecked()) m_drawRoiButton->setChecked(false);

    QMetaObject::invokeMethod(m_worker, "stopCapture", Qt::QueuedConnection);
    m_liveView->clear();
    setConnectedState(false);
    m_statusLabel->setText("Disconnected.");
    m_statusLabel->setStyleSheet("color: gray; font-size: 11px;");
}

void CapturePanel::onRecordToggled(bool checked)
{
    if (!m_connected) { m_recordButton->setChecked(false); return; }
    if (checked) {
        QString path = makeOutputPath();
        m_outputPathEdit->setText(path);
        QMetaObject::invokeMethod(m_worker, "startRecording",
                                  Qt::QueuedConnection, Q_ARG(QString, path));
    } else {
        QMetaObject::invokeMethod(m_worker, "stopRecording", Qt::QueuedConnection);
    }
}

void CapturePanel::onCameraOpened(int width, int height, double fps)
{
    setConnectedState(true);
    m_statusLabel->setText(
        QString("Connected — %1 × %2 @ %3 fps")
            .arg(width).arg(height).arg(fps, 0, 'f', 1));
    m_statusLabel->setStyleSheet("color: green; font-size: 11px;");
}

void CapturePanel::onParamsReady(CameraParamSet params)
{
    applyParamInfo(m_exposureSpin, m_exposureAutoCheck, m_exposureLabel, params.exposure);
    applyParamInfo(m_gainSpin,     m_gainAutoCheck,     m_gainLabel,     params.gain);
    applyParamInfo(m_gammaSpin,    nullptr,             m_gammaLabel,    params.gamma);
    applyParamInfo(m_brightnessSpin, nullptr,           m_brightnessLabel, params.brightness);

    // FPS is always enabled: if the hardware honours it, great; otherwise the worker
    // applies software rate-limiting by adjusting the grab timer interval.
    {
        QSignalBlocker b(m_fpsSpin);
        m_fpsSpin->setRange(params.fps.minVal > 0 ? params.fps.minVal : 1.0,
                            params.fps.maxVal > 0 ? params.fps.maxVal : 120.0);
        m_fpsSpin->setSingleStep(1.0);
        m_fpsSpin->setValue(params.fps.currentVal > 0 ? params.fps.currentVal : 30.0);
    }
    m_fpsSpin->setEnabled(true);
    m_fpsLabel->setEnabled(true);
    if (params.fps.softwareOnly || !params.fps.supported) {
        m_fpsSpin->setToolTip(
            "Hardware FPS control not detected for this camera.\n"
            "The grab rate will be limited in software — the saved video\n"
            "will play back at this rate, but the camera may still capture\n"
            "faster internally.");
        m_fpsLabel->setText("FPS (sw):");
    } else {
        m_fpsSpin->setToolTip("Acquisition frame rate (hardware-controlled).");
        m_fpsLabel->setText("FPS:");
    }

    // Label units based on what we know about each backend's conventions
    if (params.exposure.supported)
        m_exposureLabel->setText(
            params.exposure.maxVal > 1000 ? "Exposure (µs):" : "Exposure:");
    if (params.gain.supported)
        m_gainLabel->setText(
            params.gain.maxVal <= 48 ? "Gain (dB):" : "Gain:");
}

void CapturePanel::onCameraError(const QString& message)
{
    setConnectedState(false);
    m_liveView->clear();
    m_statusLabel->setText(QString("Error: %1").arg(message));
    m_statusLabel->setStyleSheet("color: red; font-size: 11px;");
}

void CapturePanel::onRecordingStarted(const QString& path)
{
    m_outputPathEdit->setText(path);
    m_statusLabel->setText(QString("● Recording → %1").arg(path));
    m_statusLabel->setStyleSheet("color: red; font-size: 11px;");
}

void CapturePanel::onRecordingStopped(int frameCount)
{
    m_statusLabel->setText(
        QString("Saved %1 frames → %2").arg(frameCount).arg(m_outputPathEdit->text()));
    m_statusLabel->setStyleSheet("color: green; font-size: 11px;");
    m_recordButton->setChecked(false);
}

// ---------------------------------------------------------------------------
// Parameter slots
// ---------------------------------------------------------------------------

void CapturePanel::onExposureChanged(double v)
{
    if (!m_connected || m_exposureAutoCheck->isChecked()) return;
    QMetaObject::invokeMethod(m_worker, "applyParam", Qt::QueuedConnection,
        Q_ARG(int, (int)CameraParam::ExposureUs), Q_ARG(double, v));
}
void CapturePanel::onExposureAutoToggled(bool checked)
{
    m_exposureSpin->setEnabled(!checked);
    if (!m_connected) return;
    QMetaObject::invokeMethod(m_worker, "applyParamAuto", Qt::QueuedConnection,
        Q_ARG(int, (int)CameraParam::ExposureUs), Q_ARG(bool, checked));
}
void CapturePanel::onGainChanged(double v)
{
    if (!m_connected || m_gainAutoCheck->isChecked()) return;
    QMetaObject::invokeMethod(m_worker, "applyParam", Qt::QueuedConnection,
        Q_ARG(int, (int)CameraParam::GainDb), Q_ARG(double, v));
}
void CapturePanel::onGainAutoToggled(bool checked)
{
    m_gainSpin->setEnabled(!checked);
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
    m_liveView->setRoiMode(checked);
}

void CapturePanel::onClearRoiClicked()
{
    m_liveView->clearRoi();
    QMetaObject::invokeMethod(m_worker, "clearRoi", Qt::QueuedConnection);
    m_clearRoiButton->setEnabled(false);
    m_roiLabel->clear();
}

void CapturePanel::onRoiDefined(QRectF roi)
{
    // Exit draw mode automatically after drawing
    m_drawRoiButton->setChecked(false);
    m_clearRoiButton->setEnabled(true);
    m_roiLabel->setText(QString("ROI: %1 × %2 px  @ (%3, %4)")
        .arg((int)roi.width()).arg((int)roi.height())
        .arg((int)roi.x()).arg((int)roi.y()));

    // Send integer rect to worker
    QRect iRoi(roi.toRect());
    QMetaObject::invokeMethod(m_worker, "setRoi", Qt::QueuedConnection,
                              Q_ARG(QRect, iRoi));
}

void CapturePanel::onRoiCleared()
{
    m_clearRoiButton->setEnabled(false);
    m_roiLabel->clear();
    QMetaObject::invokeMethod(m_worker, "clearRoi", Qt::QueuedConnection);
}
