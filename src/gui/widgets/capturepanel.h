#ifndef CAPTUREPANEL_H
#define CAPTUREPANEL_H

#include "icamerasource.h"
#include <QObject>
#include <QThread>
#include <QString>
#include <QList>

class LiveView;
class CaptureWorker;
class QComboBox;
class QPushButton;
class QLabel;
class QLineEdit;
class QDoubleSpinBox;
class QCheckBox;

/**
 * CapturePanel — controller for the Capture tab.
 *
 * Not a widget itself; wires together the widgets defined in mainwindow.ui
 * with CaptureWorker (which lives on its own thread).
 *
 * Usage:
 *   m_capturePanel = new CapturePanel(this);
 *   m_capturePanel->setup({ ui->captureVideoView, ui->captureCameraCombo, ... });
 */
class CapturePanel : public QObject
{
    Q_OBJECT
public:
    /** All UI widget pointers sourced from mainwindow.ui. */
    struct Widgets {
        // View
        LiveView*       liveView          = nullptr;
        // Toolbar
        QPushButton*    setScaleButton    = nullptr;
        QLabel*         scaleLabel        = nullptr;
        QPushButton*    drawRoiButton     = nullptr;
        QPushButton*    clearRoiButton    = nullptr;
        QLabel*         roiLabel          = nullptr;
        // Camera controls
        QComboBox*      cameraCombo       = nullptr;
        QPushButton*    scanButton        = nullptr;
        QComboBox*      resolutionCombo   = nullptr;
        QPushButton*    connectButton     = nullptr;
        QPushButton*    disconnectButton  = nullptr;
        QPushButton*    recordButton      = nullptr;
        QLineEdit*      outputPathEdit    = nullptr;
        QLabel*         statusLabel       = nullptr;
        // Camera settings
        QDoubleSpinBox* exposureSpin      = nullptr;
        QCheckBox*      exposureAutoCheck = nullptr;
        QLabel*         exposureLabel     = nullptr;
        QDoubleSpinBox* gainSpin          = nullptr;
        QCheckBox*      gainAutoCheck     = nullptr;
        QLabel*         gainLabel         = nullptr;
        QDoubleSpinBox* gammaSpin         = nullptr;
        QLabel*         gammaLabel        = nullptr;
        QDoubleSpinBox* fpsSpin           = nullptr;
        QLabel*         fpsLabel          = nullptr;
        QDoubleSpinBox* brightnessSpin    = nullptr;
        QLabel*         brightnessLabel   = nullptr;
    };

    explicit CapturePanel(QObject* parent = nullptr);
    ~CapturePanel() override;

    /** Wire the controller to the UI widgets and start the worker thread.
     *  Call once, after setupUi(). */
    void setup(const Widgets& w);

public slots:
    void setOutputDirectory(const QString& dir);
    /** Probe for available cameras. Safe to call any time; triggers the macOS
     *  camera privacy sound so should not be called at application startup. */
    void scanCameras();

signals:
    /** Emitted after a successful scale measurement.
     *  @p pixelsPerUnit  pixels per @p unit (e.g. pixels per mm).
     *  Connect to MainWindow to update pixelSizeSpinBoxD. */
    void pixelScaleSet(double pixelsPerUnit, QString unit);

private slots:
    void onScanClicked();
    void onConnectClicked();
    void onDisconnectClicked();
    void onRecordToggled(bool checked);
    void onCameraOpened(int width, int height, double fps);
    void onParamsReady(CameraParamSet params);
    void onCameraError(const QString& message);
    void onRecordingStarted(const QString& path);
    void onRecordingStopped(int frameCount);

    void onExposureChanged(double value);
    void onExposureAutoToggled(bool checked);
    void onGainChanged(double value);
    void onGainAutoToggled(bool checked);
    void onGammaChanged(double value);
    void onFpsChanged(double value);
    void onBrightnessChanged(double value);

    void onDrawRoiToggled(bool checked);
    void onClearRoiClicked();
    void onRoiDefined(QRectF roi);
    void onRoiCleared();

    void onSetScaleClicked();
    void onScaleMeasured(double pixelLength);

private:
    void setConnectedState(bool connected);
    void applyParamInfo(QDoubleSpinBox* spin, QCheckBox* autoBox,
                        QLabel* label, const ParamInfo& info);
    QString makeOutputPath() const;

    // Worker
    CaptureWorker* m_worker       = nullptr;
    QThread*       m_workerThread = nullptr;

    // Cached widget pointers (non-owning)
    Widgets w;

    QList<CameraInfo> m_cameras;
    QString           m_outputDirectory;
    bool              m_connected  = false;
    bool              m_recording  = false;

    // Scale calibration state
    double  m_scalePhysicalValue = 1.0;
    QString m_scaleUnit;
    double  m_currentUmPerPixel = 0.0;  // 0 = no calibration set yet
};

#endif // CAPTUREPANEL_H
