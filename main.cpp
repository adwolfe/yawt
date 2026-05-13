#include "mainwindow.h"
#include <QStyleFactory>
#include <QApplication>
#include <QMetaType>
#include <opencv2/core.hpp>
#include <QCommandLineParser>
#include "src/utils/debugutils.h"
#define LOGGING_CATEGORIES_DEFINE
#include "src/utils/loggingcategories.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Install unified log message pattern and apply debug logging from command-line arguments.
    Yawt::Logging::installDefaultMessagePattern();

    QCommandLineParser parser;
    parser.setApplicationDescription("YAWT - Yet Another Worm Tracker");
    parser.addHelpOption();
    QCommandLineOption debugOpt("debug", "Enable YAWT debug logging.");
    parser.addOption(debugOpt);
    parser.process(a);

    const bool debugEnabled = parser.isSet(debugOpt);
    Yawt::Logging::applyDebugMode(debugEnabled);
    DebugUtils::setTrackingDebugEnabled(debugEnabled);

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
