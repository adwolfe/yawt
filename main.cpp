#include "mainwindow.h"
#include <QStyleFactory>
#include <QApplication>
#include <QMetaType>
#include <opencv2/core.hpp>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Register cv::Mat for queued signal/slot delivery across threads.
    // Using the string name ensures the meta-type is registered for queued use.
    qRegisterMetaType<cv::Mat>("cv::Mat");

    QApplication::setStyle(QStyleFactory::create("Fusion"));


    QFont appFont;

#ifdef Q_OS_MAC
    appFont = QFont("Helvetica Neue", 13);  // macOS default system font
#elif defined(Q_OS_WIN)
    appFont = QFont("Segoe UI", 9);  // Windows system font
#else
    appFont = QFont("Sans Serif", 9);  // Linux or fallback
#endif

    QApplication::setFont(appFont);

    MainWindow w;
    w.setWindowTitle("Yet Another Worm Tracker");
    w.show();
    return a.exec();
}
