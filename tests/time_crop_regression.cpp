#include "src/gui/mainwindow.h"
#include "src/gui/widgets/videoloader.h"
#include "src/gui/widgets/framecropslider.h"
#include "src/core/trackingmanager.h"
#include <QApplication>
#include <QElapsedTimer>
#include <QLoggingCategory>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTabWidget>
#include <QGridLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QLineEdit>
#include <QKeyEvent>
#include <QTemporaryDir>
#include <QThread>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <functional>
#include <stdexcept>

static void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
static bool waitFor(const std::function<bool()>& ready) {
    QElapsedTimer timer;
    timer.start();
    while (!ready() && timer.elapsed() < 10000) {
        QApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(1);
    }
    return ready();
}
static void mouse(FrameCropSlider& slider, QEvent::Type type, QPointF pos) {
    QMouseEvent event(type, pos, slider.mapToGlobal(pos.toPoint()),
                      type == QEvent::MouseMove ? Qt::NoButton : Qt::LeftButton,
                      type == QEvent::MouseButtonRelease ? Qt::NoButton : Qt::LeftButton,
                      Qt::NoModifier);
    QApplication::sendEvent(&slider, &event);
}
class TestSlider : public FrameCropSlider {
public:
    QPoint handleCenter() const {
        QStyleOptionSlider option;
        initStyleOption(&option);
        return style()->subControlRect(QStyle::CC_Slider, &option, QStyle::SC_SliderHandle, this).center();
    }
};
static void testSlider() {
    TestSlider slider;
    slider.setOrientation(Qt::Horizontal);
    slider.resize(600, 30);
    slider.setRange(0, 99);
    slider.setFrameCrop(20, 70);
    slider.show();
    slider.setValue(45);
    mouse(slider, QEvent::MouseButtonPress, slider.handleCenter());
    mouse(slider, QEvent::MouseMove, QPointF(-100, 15));
    mouse(slider, QEvent::MouseButtonRelease, QPointF(-100, 15));
    check(slider.value() == 20 && slider.sliderPosition() == 20, "drag stops at left bumper");
    // Move from the left bumper to beyond the other end of the video.
    mouse(slider, QEvent::MouseButtonPress, slider.handleCenter());
    mouse(slider, QEvent::MouseMove, QPointF(900, 15));
    mouse(slider, QEvent::MouseButtonRelease, QPointF(900, 15));
    check(slider.value() == 70 && slider.sliderPosition() == 70, "drag stops at right bumper");
    slider.triggerAction(QAbstractSlider::SliderToMinimum);
    check(slider.value() == 20, "Home stays in crop");
    slider.triggerAction(QAbstractSlider::SliderPageStepSub);
    check(slider.value() == 20, "page back stays in crop");
    slider.triggerAction(QAbstractSlider::SliderToMaximum);
    check(slider.value() == 70, "End stays in crop");
    slider.triggerAction(QAbstractSlider::SliderSingleStepAdd);
    check(slider.value() == 70, "step forward stays in crop");
    QWheelEvent wheel(slider.handleCenter(), slider.mapToGlobal(slider.handleCenter()),
                      QPoint(), QPoint(0, 120), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
    QApplication::sendEvent(&slider, &wheel);
    check(slider.value() == 70, "wheel stays in crop");
    check(slider.minimum() == 0 && slider.maximum() == 99, "full video scale retained");
    slider.grab().save("/tmp/yawt-time-crop-slider.png");
    slider.setFrameCrop(42, 42);
    slider.triggerAction(QAbstractSlider::SliderToMaximum);
    check(slider.value() == 42, "one-frame crop");
}
static QString makeVideo(const QString& directory) {
    const QString path = directory + "/crop.avi";
    cv::VideoWriter writer(path.toStdString(), cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(100, 80));
    check(writer.isOpened(), "create video fixture");
    for (int i = 0; i < 20; ++i) {
        cv::Mat frame(80, 100, CV_8UC3, cv::Scalar::all(255));
        cv::ellipse(frame, cv::Point(40 + i / 4, 40), cv::Size(10, 4), 0, 0, 360, cv::Scalar::all(0), -1);
        writer.write(frame);
    }
    return path;
}
static void testNavigation(const QString& path) {
    MainWindow window;
    auto* video = window.findChild<VideoLoader*>();
    auto* first = window.findChild<QSpinBox*>("startFrameSpinBox");
    auto* last = window.findChild<QSpinBox*>("stopFrameSpinBox");
    auto* slider = window.findChild<QSlider*>("frameSlider");
    check(first && last && slider && video, "crop widgets present");
    check(!first->isEnabled() && !last->isEnabled(), "crop disabled before video loads");
    video->loadVideo(path);
    check(waitFor([&] { return video->getCurrentFrameNumber() == 0; }), "video opens");
    check(first->value() == 0 && last->value() == 19, "default full range");
    auto* layout = qobject_cast<QGridLayout*>(first->parentWidget()->layout());
    auto* begin = window.findChild<QPushButton*>("trackingDialogButton");
    int row, column, rows, columns, firstRow, lastRow;
    layout->getItemPosition(layout->indexOf(first), &firstRow, &column, &rows, &columns);
    layout->getItemPosition(layout->indexOf(last), &lastRow, &column, &rows, &columns);
    layout->getItemPosition(layout->indexOf(begin), &row, &column, &rows, &columns);
    check(firstRow + 1 == lastRow && lastRow + 1 == row, "crop controls directly above Begin Tracking");
    first->setValue(5);
    last->setValue(12);
    check(waitFor([&] { return video->getCurrentFrameNumber() == 5; }), "editing crop seeks inside it");
    check(first->maximum() == 12 && last->minimum() == 5, "crossed ranges prevented");
    check(slider->maximum() == 19, "scrub scale unchanged");
    for (auto* tabs : window.findChildren<QTabWidget*>()) {
        for (int i = 0; i < tabs->count(); ++i)
            if (tabs->widget(i)->isAncestorOf(first)) tabs->setCurrentIndex(i);
    }
    window.resize(1206, 850);
    window.show();
    QApplication::processEvents();
    window.grab().save("/tmp/yawt-time-crop-window.png");
    auto* stopEditor = last->findChild<QLineEdit*>();
    check(stopEditor, "stop frame editor present");
    last->setFocus();
    last->selectAll();
    for (int i = 0; i < 30; ++i) {
        QKeyEvent digit(QEvent::KeyPress, Qt::Key_9, Qt::NoModifier, "9");
        QApplication::sendEvent(last, &digit);
    }
    check(stopEditor->text() == QString(30, QLatin1Char('9')), "oversized input is not rejected while typing");
    check(last->value() == 12, "crop unchanged until input committed");
    QKeyEvent enter(QEvent::KeyPress, Qt::Key_Return, Qt::NoModifier);
    QApplication::sendEvent(last, &enter);
    check(last->value() == 19 && stopEditor->text() == "19", "oversized input clamps on Enter");
    video->seekToFrame(19);
    check(waitFor([&] { return video->getCurrentFrameNumber() == 19; }), "oversized entry expands navigation to video end");
    last->setValue(12);
    last->setFocus();
    last->selectAll();
    stopEditor->insert("999999999999999999999999999999");
    first->setFocus();
    QApplication::processEvents();
    check(last->value() == 19 && stopEditor->text() == "19", "oversized pasted input clamps on focus loss");
    last->setValue(12);
    video->seekToFrame(19);
    check(waitFor([&] { return video->getCurrentFrameNumber() == 12; }), "external seeks clamped");
    bool playing = false;
    QObject::connect(video, &VideoLoader::playbackStateChanged, &window, [&](bool active, double) { playing = active; });
    video->play();
    check(waitFor([&] { return !playing; }), "playback stops at crop end");
    check(video->getCurrentFrameNumber() == 12, "last included frame displayed");
    first->setValue(12);
    video->play();
    check(waitFor([&] { return !playing; }), "single-frame playback stops");
    video->loadVideo(path);
    check(waitFor([&] { return video->getCurrentFrameNumber() == 0; }), "video reload opens");
    check(first->value() == 0 && last->value() == 19, "new video resets crop");
}
static void testTracking(const QString& path, const QString& directory, int first, int last, int key) {
    TrackingDataStorage storage;
    TrackingManager manager(&storage);
    manager.setCenterlineEnabled(false);
    bool finished = false;
    QString error;
    Tracking::AllWormTracks tracks;
    QObject::connect(&manager, &TrackingManager::allTracksUpdated, [&](const Tracking::AllWormTracks& result) { tracks = result; });
    QObject::connect(&manager, &TrackingManager::trackingFinishedSuccessfully, [&](const QString&) { finished = true; });
    QObject::connect(&manager, &TrackingManager::trackingFailed, [&](const QString& message) { error = message; });
    Tracking::InitialWormInfo worm;
    worm.id = 1;
    worm.initialSearchWindow = QRectF(20, 20, 50, 40);
    manager.startFullTrackingProcess(path, directory, key, {worm}, {}, 20, first, last);
    check(waitFor([&] { return finished || !error.isEmpty(); }), "tracking completes");
    if (!error.isEmpty()) throw std::runtime_error(error.toStdString());
    check(tracks.count(1) && tracks.at(1).size() == size_t(last - first + 1), "only selected frames tracked");
    for (int i = first; i <= last; ++i)
        check(tracks.at(1).at(i - first).frameNumber == i, "absolute frame numbers and inclusive endpoints preserved");
    manager.cleanupThreadsAndObjects();
}
int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QLoggingCategory::setFilterRules("*.debug=false\n*.info=false");
    try {
        QTemporaryDir directory;
        check(directory.isValid(), "temporary directory");
        testSlider();
        const auto path = makeVideo(directory.path());
        testNavigation(path);
        testTracking(path, directory.path(), 5, 12, 8);
        testTracking(path, directory.path(), 8, 8, 8);
        testTracking(path, directory.path(), 5, 12, 5);
        testTracking(path, directory.path(), 5, 12, 12);
        testTracking(path, directory.path(), 0, 19, 0);
        TrackingManager invalid;
        QString invalidError;
        QObject::connect(&invalid, &TrackingManager::trackingFailed, [&](const QString& error) { invalidError = error; });
        invalid.startFullTrackingProcess(path, directory.path(), 4, {}, {}, 20, 5, 12);
        check(!invalidError.isEmpty(), "selection frame outside crop rejected");
        std::puts("Time crop regressions passed");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
