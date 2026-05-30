#ifndef CAPTUREPANEL_H
#define CAPTUREPANEL_H

#include "icamerasource.h"
#include <QWidget>
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
class QGroupBox;

class CapturePanel : public QWidget
{
    Q_OBJECT
public:
    explicit CapturePanel(QWidget* parent = nullptr);
    ~CapturePanel() override;

public slots:
    void setOutputDirectory(const QString& dir);

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

    // Parameter changes → worker
    void onExposureChanged(double value);
    void onExposureAutoToggled(bool checked);
    void onGainChanged(double value);
    void onGainAutoToggled(bool checked);
    void onGammaChanged(double value);
    void onFpsChanged(double value);
    void onBrightnessChanged(double value);

    // ROI
    void onDrawRoiToggled(bool checked);
    void onClearRoiClicked();
    void onRoiDefined(QRectF roi);
    void onRoiCleared();

private:
    void buildUi();
    void setConnectedState(bool connected);
    void applyParamInfo(QDoubleSpinBox* spin, QCheckBox* autoBox,
                        QLabel* label, const ParamInfo& info);
    QString makeOutputPath() const;

    // Widgets
    LiveView*        m_liveView          = nullptr;
    CaptureWorker*   m_worker            = nullptr;
    QThread*         m_workerThread      = nullptr;

    QComboBox*       m_cameraCombo       = nullptr;
    QComboBox*       m_resolutionCombo   = nullptr;
    QPushButton*     m_scanButton        = nullptr;
    QPushButton*     m_connectButton     = nullptr;
    QPushButton*     m_disconnectButton  = nullptr;
    QPushButton*     m_recordButton      = nullptr;
    QLineEdit*       m_outputPathEdit    = nullptr;
    QLabel*          m_statusLabel       = nullptr;

    // Camera parameter controls
    QGroupBox*       m_paramsGroup       = nullptr;
    QDoubleSpinBox*  m_exposureSpin      = nullptr;
    QCheckBox*       m_exposureAutoCheck = nullptr;
    QLabel*          m_exposureLabel     = nullptr;
    QDoubleSpinBox*  m_gainSpin          = nullptr;
    QCheckBox*       m_gainAutoCheck     = nullptr;
    QLabel*          m_gainLabel         = nullptr;
    QDoubleSpinBox*  m_gammaSpin         = nullptr;
    QLabel*          m_gammaLabel        = nullptr;
    QDoubleSpinBox*  m_fpsSpin           = nullptr;
    QLabel*          m_fpsLabel          = nullptr;
    QDoubleSpinBox*  m_brightnessSpin    = nullptr;
    QLabel*          m_brightnessLabel   = nullptr;

    // ROI
    QPushButton*     m_drawRoiButton     = nullptr;
    QPushButton*     m_clearRoiButton    = nullptr;
    QLabel*          m_roiLabel          = nullptr;

    QList<CameraInfo> m_cameras;
    QString           m_outputDirectory;
    bool              m_connected = false;
};

#endif // CAPTUREPANEL_H
