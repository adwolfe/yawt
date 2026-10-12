#include "src/data/referenceannotations.h"
#include "src/data/trackingdatastorage.h"
#include "src/gui/mainwindow.h"
#include "src/gui/widgets/referenceannotationpanel.h"
#include "src/gui/widgets/videoloader.h"
#include "src/utils/debugutils.h"

#include <QApplication>
#include <QComboBox>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QLabel>
#include <QLineEdit>
#include <QLoggingCategory>
#include <QMouseEvent>
#include <QAbstractButton>
#include <QSpinBox>
#include <QScrollArea>
#include <QScrollBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QThread>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <cmath>
#include <functional>
#include <stdexcept>

using namespace ReferenceAnnotations;
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
static QString makeVideo(const QString& directory) {
    const QString path = directory + "/reference.avi";
    cv::VideoWriter writer(path.toStdString(), cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(100, 80));
    check(writer.isOpened(), "create reference video fixture");
    for (int i = 0; i < 20; ++i) {
        cv::Mat frame(80, 100, CV_8UC3, cv::Scalar(170, 190, 210));
        cv::ellipse(frame, cv::Point(40 + i / 4, 40), cv::Size(10, 4), 0, 0, 360, cv::Scalar::all(0), -1);
        writer.write(frame);
    }
    return path;
}
static Video videoInfo(const QString& path) { return {path, QSize(100, 80), 20, 30, QFileInfo(path).size()}; }
static Document read(const QString& path) {
    Document result;
    QString error;
    if (!load(defaultPath(path), videoInfo(path), &result, &error)) throw std::runtime_error(error.toStdString());
    return result;
}

static void testCodec(const QString& path, const QString& directory) {
    Document d;
    d.video = videoInfo(path);
    d.source = {"original.avi", QPoint(123, 456), 789};
    auto& a = d.frames[5][2];
    a.position = QPointF(40.5, 30.25);
    a.endA = QPointF(20, 30);
    a.centerline = {{20, 30}, {40, 35}, {60, 30}};
    a.orientation = "unknown";
    a.visibility = "partial";
    a.event = "self_contact";
    a.review = "uncertain";
    a.notes = "Hidden end; do not infer its position. μm";
    d.frames[17][2].visibility = "hidden";
    d.frames[17][2].event = "lost";
    d.frames[17][2].review = "verified";
    const auto file = directory + "/codec.json";
    QString error;
    check(save(file, d, &error), "save references");
    Document reopened;
    check(load(file, d.video, &reopened, &error) && reopened == d, "round trip sparse frames, source mapping, unknown endpoints, and review labels");
    QFile raw(file);
    check(raw.open(QIODevice::ReadOnly), "read codec fixture");
    const auto validBytes = raw.readAll();
    const auto json = QJsonDocument::fromJson(validBytes);
    check(!json.object()["annotations"].toArray()[0].toObject().contains("end_b"), "unknown endpoint omitted, not stored as zero or predicted");
    raw.close();

    auto reject = [&](QJsonObject root) {
        QFile modified(file);
        check(modified.open(QIODevice::WriteOnly | QIODevice::Truncate), "write invalid fixture");
        modified.write(QJsonDocument(root).toJson());
        modified.close();
        Document output = d;
        check(!load(file, d.video, &output, &error) && output == d, "invalid references rejected without partially changing output");
    };
    auto root = json.object();
    root["schema_version"] = 99;
    reject(root);
    root = json.object();
    auto records = root["annotations"].toArray();
    auto record = records[0].toObject();
    record["position"] = QJsonArray{200, 10};
    records[0] = record;
    root["annotations"] = records;
    reject(root);
    root = json.object();
    records = root["annotations"].toArray();
    records.append(records[0]);
    root["annotations"] = records;
    reject(root);
    root = json.object();
    auto wrongVideo = root["video"].toObject();
    wrongVideo["byte_size"] = 1;
    root["video"] = wrongVideo;
    reject(root);
    check(!save(directory + "/missing/subdirectory/reference.json", d, &error), "save failures reported");
}

static QPointF widgetPoint(VideoLoader* video, QPointF p) {
    QSizeF size = video->getVideoFrameSize();
    size.scale(video->size(), Qt::KeepAspectRatio);
    size *= video->getZoomFactor();
    return QPointF((video->width() - size.width()) / 2.0, (video->height() - size.height()) / 2.0) +
        QPointF(p.x() * size.width() / 100.0, p.y() * size.height() / 80.0);
}
static void mouse(VideoLoader* video, QEvent::Type type, QPointF p, Qt::MouseButton button = Qt::LeftButton) {
    const auto wp = widgetPoint(video, p);
    QMouseEvent event(type, wp, video->mapToGlobal(wp.toPoint()),
        type == QEvent::MouseMove ? Qt::NoButton : button,
        type == QEvent::MouseButtonRelease ? Qt::NoButton : button, Qt::NoModifier);
    QApplication::sendEvent(video, &event);
}
static void click(VideoLoader* video, QPointF p, Qt::MouseButton button = Qt::LeftButton) {
    mouse(video, QEvent::MouseButtonPress, p, button);
    mouse(video, QEvent::MouseButtonRelease, p, button);
}
template<class T> static T* widget(QWidget& window, const char* name) {
    auto* result = window.findChild<T*>(name);
    check(result != nullptr, name);
    return result;
}
static void showReference(MainWindow& window) {
    auto* mainTabs = widget<QTabWidget>(window, "mainTabWidget");
    mainTabs->setCurrentWidget(widget<QWidget>(window, "processingTab"));
    widget<QTabWidget>(window, "tabWidget")->setCurrentWidget(widget<ReferenceAnnotationPanel>(window, "referenceAnnotationPanel"));
    window.resize(1240, 1000);
    window.show();
    QApplication::processEvents();
}

static void testUI(const QString& path) {
    DebugUtils::setDebugCaptureEnabled(false);
    {
        MainWindow normal;
        check(!normal.findChild<ReferenceAnnotationPanel*>(), "reference mode absent without --debug");
        auto* video = normal.findChild<VideoLoader*>();
        video->setInteractionMode(VideoLoader::InteractionMode::ReferenceAnnotation);
        check(video->getCurrentInteractionMode() != VideoLoader::InteractionMode::ReferenceAnnotation, "annotation interaction is debug gated");
    }
    DebugUtils::setDebugCaptureEnabled(true);
    {
        MainWindow window;
        auto* video = window.findChild<VideoLoader*>();
        auto* panel = widget<ReferenceAnnotationPanel>(window, "referenceAnnotationPanel");
        check(!panel->isEnabled(), "reference tools disabled before video loads");
        check(video->loadVideo(path), "load video accepted");
        check(waitFor([&] { return video->getCurrentFrameNumber() == 0; }), "video opens");
        video->setViewModeOption(VideoLoader::ViewModeOption::Threshold, true);
        showReference(window);
        auto* tool = widget<QComboBox>(window, "referenceTool");
        auto* worm = widget<QSpinBox>(window, "referenceWormId");
        auto* review = widget<QComboBox>(window, "referenceReview");
        auto* orientation = widget<QComboBox>(window, "referenceOrientation");
        auto* table = widget<QTableWidget>(window, "referenceFrames");
        check(panel->isEnabled() && table->rowCount() == 0 && !QFileInfo::exists(defaultPath(path)), "no references manufactured by opening a video");
        const QColor rawPixel = video->grab().toImage().pixelColor(widgetPoint(video, {85, 65}).toPoint());
        check(rawPixel.red() > rawPixel.blue() + 20, "reference view displays raw video even with threshold enabled");

        tool->setCurrentIndex(1); // approximate position
        click(video, {40, 40});
        auto d = read(path);
        check(d.frames.size() == 1 && d.frames[0][1].position == QPointF(40, 40), "pixel coordinates saved for the clicked frame and worm");
        check(d.frames[0][1].review == "unreviewed", "manual geometry is not automatically verified");
        tool->setCurrentIndex(2);
        click(video, {30, 40});
        tool->setCurrentIndex(3);
        click(video, {50, 40});
        orientation->setCurrentIndex(1);
        review->setCurrentIndex(1);
        check(read(path).frames[0][1].review == "verified", "explicit review saved");

        video->setZoomFactor(2);
        video->centerOnVideoPoint({50, 40});
        click(video, {52, 42});
        d = read(path);
        check(d.frames[0][1].endB == QPointF(52, 42) && d.frames[0][1].review == "unreviewed", "zoomed edits use video pixels and invalidate previous verification");
        widget<QAbstractButton>(window, "referenceUndo")->click();
        d = read(path);
        check(d.frames[0][1].endB == QPointF(50, 40) && d.frames[0][1].review == "verified", "undo restores geometry and verification on disk");
        widget<QAbstractButton>(window, "referenceRedo")->click();
        check(read(path).frames[0][1].endB == QPointF(52, 42), "redo is autosaved");
        tool->setCurrentIndex(4);
        click(video, {35, 40});
        click(video, {40, 38});
        click(video, {50, 40});
        mouse(video, QEvent::MouseButtonPress, {40, 38});
        mouse(video, QEvent::MouseMove, {42, 37});
        mouse(video, QEvent::MouseButtonRelease, {42, 37});
        d = read(path);
        check(d.frames[0][1].centerline.size() == 3 && d.frames[0][1].centerline[1] == QPointF(42, 37), "centerline vertices can be dragged without appending another point");
        click(video, {42, 37}, Qt::RightButton);
        check(read(path).frames[0][1].centerline.size() == 2, "right click removes the selected vertex");
        widget<QAbstractButton>(window, "referenceUndo")->click();
        widget<QAbstractButton>(window, "referenceSwapEnds")->click();
        d = read(path);
        check(d.frames[0][1].endA == QPointF(52, 42) && d.frames[0][1].orientation == "b_is_head" &&
              d.frames[0][1].centerline.front() == QPointF(50, 40), "swap reverses geometry while preserving anatomical head assignment");

        video->seekToFrame(8);
        check(waitFor([&] { return video->getCurrentFrameNumber() == 8; }), "seek to sparse reference frame");
        check(review->currentIndex() == 0, "frame navigation does not carry review labels to unannotated frames");
        worm->setValue(2);
        tool->setCurrentIndex(1);
        click(video, {60, 40});
        widget<QComboBox>(window, "referenceEvent")->setCurrentIndex(2);
        widget<QComboBox>(window, "referenceVisibility")->setCurrentIndex(1);
        review->setCurrentIndex(2);
        auto* notes = widget<QLineEdit>(window, "referenceNotes");
        notes->setFocus();
        notes->insert("Identity uncertain after merge");
        d = read(path);
        check(d.frames[8][2].notes == "Identity uncertain after merge" && d.frames[8][2].review == "unreviewed", "notes saved to correct worm/frame and invalidate verification");
        review->setCurrentIndex(2);
        video->seekToFrame(0);
        check(waitFor([&] { return video->getCurrentFrameNumber() == 0; }), "seek after note editing");
        check(read(path).frames[8][2].notes == "Identity uncertain after merge" && notes->text().isEmpty(), "notes do not leak to a different frame");
        widget<QAbstractButton>(window, "referenceNextMarked")->click();
        check(waitFor([&] { return video->getCurrentFrameNumber() == 8; }), "marked-frame navigation follows selected worm");
        widget<QAbstractButton>(window, "referenceDeleteFrame")->click();
        check(!read(path).frames.contains(8), "delete removes only selected worm/frame reference");
        widget<QAbstractButton>(window, "referenceUndo")->click();
        check(read(path).frames[8][2].review == "uncertain", "delete undo restores complete annotation");
        check(!window.findChild<TrackingDataStorage*>()->hasAnyData(), "reference editing never writes tracking storage");

        video->setZoomFactor(1);
        worm->setValue(1);
        video->seekToFrame(0);
        check(waitFor([&] { return video->getCurrentFrameNumber() == 0; }), "show centerline reference for visual inspection");
        window.grab().save("/private/tmp/yawt-reference-window.png");
        video->grab().save("/private/tmp/yawt-reference-video.png");
        window.resize(1206, 735);
        QApplication::processEvents();
        auto* scroll = panel->findChild<QScrollArea*>();
        check(scroll && scroll->horizontalScrollBar()->maximum() == 0, "reference controls fit the narrow side panel without horizontal scrolling");
        check(widget<QAbstractButton>(window, "referenceSaveAs")->isVisible(), "save controls remain visible in a small window");
        window.grab().save("/private/tmp/yawt-reference-small-window.png");
        widget<QTabWidget>(window, "tabWidget")->setCurrentIndex(0);
        check(video->getCurrentInteractionMode() != VideoLoader::InteractionMode::ReferenceAnnotation, "leaving reference tab restores normal interaction");
        const QColor thresholdPixel = video->grab().toImage().pixelColor(widgetPoint(video, {85, 65}).toPoint());
        check(thresholdPixel.red() == thresholdPixel.blue(), "leaving reference tab restores threshold view");
        const auto preserved = read(path);
        video->seekToFrame(2, true);
        check(waitFor([&] { return video->getCurrentFrameNumber() == 2; }), "silent seek while Reference is hidden");
        check(video->loadVideo(path), "reload annotated video accepted without clearing-reference prompt");
        check(waitFor([&] { return video->getCurrentFrameNumber() == 0; }), "reloaded video opens");
        check(table->rowCount() == 2 && read(path) == preserved, "reopening automatically loads sparse reference sidecar");
        showReference(window);
        video->seekToFrame(8, true);
        check(waitFor([&] { return video->getCurrentFrameNumber() == 8; }), "silent seek while annotating");
        check(orientation->currentIndex() == 0, "reference labels follow the actual displayed frame even for silent seeks");
        video->setInteractionMode(VideoLoader::InteractionMode::Crop);
        check(widget<QTabWidget>(window, "tabWidget")->currentWidget() != panel &&
              video->getCurrentInteractionMode() == VideoLoader::InteractionMode::Crop, "normal crop tool leaves reference mode without losing the selected tool");
    }
    {
        QFile corrupt(defaultPath(path));
        check(corrupt.open(QIODevice::WriteOnly | QIODevice::Truncate), "create unreadable sidecar");
        corrupt.write("preserve this unreadable reference");
        corrupt.close();
        MainWindow window;
        auto* video = window.findChild<VideoLoader*>();
        video->loadVideo(path);
        check(waitFor([&] { return video->getCurrentFrameNumber() == 0; }), "video with bad sidecar still opens");
        check(widget<QLabel>(window, "referenceSaveStatus")->text().contains("preserved"), "bad sidecar produces visible recovery message");
        check(corrupt.open(QIODevice::ReadOnly) && corrupt.readAll() == "preserve this unreadable reference", "unreadable sidecar never silently overwritten");
    }
}

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QApplication::setStyle("Fusion");
    QLoggingCategory::setFilterRules("*.debug=false\n*.info=false");
    try {
        QTemporaryDir directory;
        check(directory.isValid(), "temporary directory");
        const auto path = makeVideo(directory.path());
        testCodec(path, directory.path());
        testUI(path);
        std::puts("Reference annotation regressions passed");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
