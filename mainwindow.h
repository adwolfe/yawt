#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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
    void threshModeToggle();



private:
    Ui::MainWindow *ui;
    bool m_threshModeToggle = false;
};
#endif // MAINWINDOW_H
