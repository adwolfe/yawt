#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "wormtablemodel.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void chooseWorkingDirectory();
    void updateFrameDisplay(int currentFrameNumber, const QImage& currentFrame);
    void seekFrame(int frame);
    void initiateFrameDisplay(const QString& filePath, int totalFrames, double fps, QSize frameSize);
    void frameSliderMoved(int value);
    void panModeToggle();
    void roiModeToggle();
    void cropModeToggle();
    void threshModeViewToggle();
    void selectionModeToggle();
    void updateThresholdModeSettings();
    void setGlobalMode(bool checked);
    void setAdaptiveMode(int value);
    void setGlobalThreshValue(int value);
    void setBlockSize(int value);
    void setTuning(double value);
    void setPreBlur(bool checked);
    void setBlurKernel(int value);
    void updateBackgroundColor(int index);
    void onWormBlobDetected(const QPointF& centroid, const QRectF& bbox);


private:
    Ui::MainWindow *ui;
    bool m_threshModeToggle = false;
    WormTableModel *m_wormTableModel;
};
#endif // MAINWINDOW_H
