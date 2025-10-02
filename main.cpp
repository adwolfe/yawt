#include "mainwindow.h"
#include <QStyleFactory>
#include <QApplication>
#include <QMetaType>
#include <opencv2/core.hpp>
#include <QCommandLineParser>
#define LOGGING_CATEGORIES_DEFINE
#include "src/utils/loggingcategories.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Install unified log message pattern and apply verbosity from command-line arguments.
    Yawt::Logging::installDefaultMessagePattern();

    QCommandLineParser parser;
    parser.setApplicationDescription("YAWT - Yet Another Worm Tracker");
    parser.addHelpOption();
    QCommandLineOption verbosityOpt(QStringList() << "v" << "verbosity", "Verbosity level (1-3). 1=basic, 2=normal, 3=firehose.", "level");
    QCommandLineOption firehoseOpt("firehose", "Alias for --verbosity=3 (very verbose)");
    parser.addOption(verbosityOpt);
    parser.addOption(firehoseOpt);
    parser.process(a);

    if (parser.isSet(firehoseOpt)) {
        Yawt::Logging::applyFirehose();
    } else {
        bool ok = false;
        int lvl = parser.value(verbosityOpt).toInt(&ok);
        if (!ok) lvl = 1;
        Yawt::Logging::applyVerbosity(lvl);
    }

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
