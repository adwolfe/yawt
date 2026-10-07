#include "src/gui/analysissessionmodel.h"
#include "src/gui/mainwindow.h"
#include "src/gui/widgets/videoloader.h"
#include "src/data/wormsjsoncodec.h"
#include "src/data/videometadatastore.h"
#include <QtConcurrent>
#include "src/data/trackingdatastorage.h"
#include "src/utils/yawtjsonio.h"

#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLoggingCategory>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <QSlider>
#include <QSpinBox>
#include <opencv2/videoio.hpp>
#include <cstdio>
#include <functional>
#include <stdexcept>
#ifdef Q_OS_UNIX
#include <sys/stat.h>
#include <condition_variable>
#include <mutex>
#include <thread>
#endif

static void check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

static bool waitFor(const std::function<bool()>& ready, int timeout = 10000)
{
    QElapsedTimer timer;
    timer.start();
    while (!ready() && timer.elapsed() < timeout) {
        QApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(1);
    }
    return ready();
}

static void writeFile(const QString& path, const QByteArray& bytes)
{
    QFile file(path);
    check(file.open(QIODevice::WriteOnly | QIODevice::Truncate), "open test file");
    check(file.write(bytes) == bytes.size(), "write test file");
}

static WormsJson::Document document(int id)
{
    WormsJson::Document doc;
    doc.keyFrame = 7;
    TableItems::AnnotationItem item;
    item.id = id;
    item.type = TableItems::ItemType::Worm;
    doc.items.append(item);
    Tracking::TrackPoint point;
    point.frameNumber = 7;
    point.position = cv::Point2f(10, 20);
    doc.tracks[id].push_back(point);
    return doc;
}

static QModelIndex firstWorm(AnalysisSessionModel& model)
{
    const auto group = model.index(0, 0);
    const auto video = model.index(0, 0, group);
    return model.index(0, 0, video);
}

// Fixed saved-file fixtures protect the JSON schema while serializers are shared.
static void testGeometrySerialization()
{
    const QJsonObject fixture = QJsonDocument::fromJson(R"json({
        "frame":7,"quality":0,"position":{"x":10.25,"y":-2.5},
        "roi":{"x":1,"y":2,"width":30,"height":40},
        "area":20,"aspectRatio":2,"bodyLength":5,
        "tips":{"head":{"x":1.25,"y":2.5},"tail":{"x":4.25,"y":6.5}},
        "blob":{
            "isValid":true,"area":20,"convexHullArea":25,"touchesROIboundary":true,
            "centroid":{"x":10.25,"y":-2.5},
            "boundingBox":{"x":1,"y":2,"width":30,"height":40},
            "contourPoints":[[-1,2],[3,4],[5,6]],
            "holeContourPoints":[[[1,2],[2,3]],[]]
        },
        "centerline":{
            "points":[[1.25,2.5],[4.25,6.5]],"hasCutPoint":true,
            "cutPoint":{"x":2.25,"y":3.5},"tipCandidates":[],
            "headTipIdx":-1,"tailTipIdx":-1,"topology":0
        }
    })json").object();
    check(!fixture.isEmpty(), "parse geometry fixture");
    Tracking::DetectedBlob blob;
    bool hasBlob = false;
    const auto point = WormsJson::trackPointFromJson(fixture, &blob, &hasBlob);
    check(hasBlob && blob.holeContourPoints.size() == 2, "load ring geometry");
    check(blob.centerline.cutPoint == cv::Point2f(2.25f, 3.5f), "load fractional cut point");
    check(WormsJson::trackPointToJson(point, &blob) == fixture, "preserve saved geometry schema");

    // The former combined layout remains readable as well as the split layout.
    QJsonObject combined = fixture.value("blob").toObject();
    combined["centerlinePoints"] = fixture["centerline"].toObject()["points"];
    combined["hasCenterlineCutPoint"] = true;
    combined["centerlineCutPoint"] = fixture["centerline"].toObject()["cutPoint"];
    combined["tipCandidates"] = QJsonArray();
    combined["assignedHeadTipIdx"] = -1;
    combined["assignedTailTipIdx"] = -1;
    combined["topologyState"] = 0;
    check(Tracking::detectedBlobToJson(Tracking::detectedBlobFromJson(combined)) == combined,
          "preserve combined blob schema");

    // Old centerline-only files can include incomplete coordinate entries.
    const auto legacy = QJsonDocument::fromJson(R"json({
        "frame":7,"position":{"x":10.25},"roi":{"width":30},
        "centerlinePoints":[[1.25,2.5],[],[99],null,[4.25,6.5]]
    })json").object();
    const auto oldPoint = WormsJson::trackPointFromJson(legacy, &blob, &hasBlob);
    check(hasBlob && blob.centerline.points.size() == 2, "skip incomplete legacy coordinates");
    check(oldPoint.position == cv::Point2f(10.25f, 0.f), "preserve missing-coordinate defaults");
    check(oldPoint.searchWindow == QRectF(0, 0, 30, 0), "preserve missing rectangle defaults");
    check(oldPoint.bodyLength == 5.f, "retain legacy centerline length derivation");
}

static void testIndex(const QString& root)
{
    const QString path = QDir(root).filePath("worms.json");
    auto doc = document(42);
    check(WormsJson::write(path, doc), "write compressed run");
    check(QFile::exists(path + ".index.json"), "writer creates a small ID index");
    check(WormsJson::readWormIds(path) == QList<int>{42}, "read indexed IDs");
    // Legacy files have no index and still load, then gain an index.
    QFile::remove(path + ".index.json");
    check(WormsJson::readWormIds(path) == QList<int>{42}, "read legacy run");
    check(QFile::exists(path + ".index.json"), "legacy run gains an index");
    // Replace the source without updating its index, as an external writer could.
    doc = document(314);
    auto json = WormsJson::toJson(doc);
    json["padding"] = QString(200, 'x');
    writeFile(path, QJsonDocument(json).toJson());
    check(WormsJson::readWormIds(path) == QList<int>{314}, "stale index invalidated");
    writeFile(path + ".index.json", "invalid json");
    check(WormsJson::readWormIds(path) == QList<int>{314}, "malformed index falls back");
    WormsJson::Document read;
    check(WormsJson::read(path, read), "read full document after indexing");
    TrackingDataStorage storage;
    storage.applyWormsDocument(std::move(read));
    check(storage.getAllTracks().count(314) == 1, "prepared import applies tracks");
}

static void testMetadata(const QString& root)
{
    VideoMetadataStore::ScaleCalibration cal;
    cal.pixelsPerUnit = 50;
    cal.unit = "µm";
    VideoMetadataStore::saveScaleAsync(root, "metadata", cal);
    VideoMetadataStore::saveUmPerPixelAsync(root, "metadata", 4);
    auto result = QtConcurrent::run([root]() {
        double scale = 0, fps = 0;
        VideoMetadataStore::loadAnalysisMetadata(root, "metadata", scale, fps);
        return scale;
    });
    check(waitFor([&] { return result.isFinished(); }), "queued metadata writes finish");
    check(result.result() == 4, "reader observes ordered metadata saves");
    VideoMetadataStore::ScaleCalibration restored;
    check(VideoMetadataStore::loadScale(root, "metadata", restored), "calibration survives merged saves");
    check(restored.pixelsPerUnit == 50, "calibration fields preserved");
}

static void testAnalysis(const QString& root)
{
    const QString data = QDir(root).filePath("analysis/yawt");
    const QString run = QDir(data).filePath("video/PROC_2026-10-06-120000");
    check(QDir().mkpath(run), "create run directory");
    check(WormsJson::write(QDir(run).filePath("worms.json"), document(23)), "write analysis fixture");
    writeFile(QDir(data).filePath("video_metadata.json"), "{\"umPerPixel\":2,\"fps\":25}");
    int finished = 0;
    {
        AnalysisSessionModel model;
        QObject::connect(&model, &AnalysisSessionModel::directoryScanFinished, [&] { ++finished; });
        model.scanDataDirectory(data);
        check(finished == 0, "directory scan returns before completion");
        check(waitFor([&] { return finished == 1; }), "analysis scan finishes");
        check(firstWorm(model).isValid(), "discovered worm is visible");
        auto groups = model.getGroupedData();
        check(groups.isEmpty(), "first plot request does not synchronously read tracks");
        check(waitFor([&] { return !model.getGroupedData().isEmpty(); }), "tracks arrive asynchronously");
        groups = model.getGroupedData();
        check(groups.first().worms.first().umPerPixel == 2, "metadata applied");
        check(groups.first().worms.first().fps == 25, "fps metadata applied");
        check(groups.first().worms.first().points.size() == 1, "track snapshot available");

        // An unchecked run must never start loading its full tracking document.
        check(model.setData(firstWorm(model), Qt::Unchecked, Qt::CheckStateRole), "uncheck worm");
        model.scanDataDirectory(data);
        check(waitFor([&] { return finished == 2; }), "refresh finishes");
        check(firstWorm(model).data(Qt::CheckStateRole).toInt() == Qt::Unchecked, "refresh retains selection");
        check(model.setData(firstWorm(model), Qt::Checked, Qt::CheckStateRole), "recheck worm");
        check(!model.getGroupedData().isEmpty(), "refresh preserves unchanged track cache");
        // Edits made after starting a refresh must win over the disk snapshot.
        model.scanDataDirectory(data);
        check(model.setData(firstWorm(model), Qt::Unchecked, Qt::CheckStateRole), "edit during refresh");
        check(waitFor([&] { return finished == 3; }), "edited refresh finishes");
        check(firstWorm(model).data(Qt::CheckStateRole).toInt() == Qt::Unchecked, "refresh retains in-flight edit");
    }
    // Destructor flushes the debounced state snapshot through the serial writer.
    AnalysisSessionModel restored;
    bool restoredDone = false;
    QObject::connect(&restored, &AnalysisSessionModel::directoryScanFinished, [&] { restoredDone = true; });
    restored.scanDataDirectory(data);
    check(waitFor([&] { return restoredDone; }), "restore scan finishes");
    check(firstWorm(restored).data(Qt::CheckStateRole).toInt() == Qt::Unchecked, "saved checks restored");

    const QString empty = QDir(root).filePath("empty/yawt");
    check(QDir().mkpath(empty), "create empty project");
    // Switch away and back before either scan completes; saved state must
    // survive even though the previous folder's visible rows were cleared.
    restoredDone = false;
    restored.scanDataDirectory(empty);
    restored.scanDataDirectory(data);
    check(waitFor([&] { return restoredDone; }), "returning-folder scan finishes");
    check(firstWorm(restored).data(Qt::CheckStateRole).toInt() == Qt::Unchecked,
          "rapid folder switching preserves saved checks");
    restoredDone = false;
    restored.scanDataDirectory(data);
    restored.scanDataDirectory(empty);
    check(waitFor([&] { return restoredDone; }), "latest folder scan finishes");
    check(restored.rowCount(restored.index(0, 0)) == 0, "stale folder result ignored");
}

#ifdef Q_OS_UNIX
static void testSlowDisk(const QString& root)
{
    const QString data = QDir(root).filePath("slow/yawt");
    const QString run = QDir(data).filePath("video/PROC_2026-10-06-120000");
    check(QDir().mkpath(run), "create slow fixture");
    const QString path = QDir(run).filePath("worms.json");
    check(::mkfifo(QFile::encodeName(path).constData(), 0600) == 0, "create delayed read pipe");
    const QByteArray bytes = QJsonDocument(WormsJson::toJson(document(91))).toJson();
    std::mutex mutex;
    std::condition_variable condition;
    bool release = false;
    std::thread writer([&] {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [&] { return release; });
        lock.unlock();
        writeFile(path, bytes);
    });
    AnalysisSessionModel model;
    bool finished = false;
    bool heartbeat = false;
    QObject::connect(&model, &AnalysisSessionModel::directoryScanFinished, [&] { finished = true; });
    QTimer::singleShot(100, [&] {
        heartbeat = true;
        std::lock_guard<std::mutex> lock(mutex);
        release = true;
        condition.notify_one();
    });
    // A synchronous read would deadlock here: the GUI timer releases the writer.
    model.scanDataDirectory(data);
    const bool completed = waitFor([&] { return finished; });
    writer.join();
    check(completed && heartbeat, "GUI timer remains responsive during blocked disk read");
    check(firstWorm(model).isValid(), "slow run eventually appears");
}
#endif

static void makeVideo(const QString& path, int intensity)
{
    cv::VideoWriter writer(path.toStdString(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                           25, cv::Size(64, 64));
    check(writer.isOpened(), "open fixture video writer");
    for (int i = 0; i < 20; ++i)
        writer.write(cv::Mat(64, 64, CV_8UC3, cv::Scalar::all(intensity)));
}

static void testVideo(const QString& root)
{
    const QString first = QDir(root).filePath("first.avi");
    const QString second = QDir(root).filePath("second.avi");
    makeVideo(first, 25);
    makeVideo(second, 220);
    VideoLoader video;
    int loaded = 0, changed = 0, failed = 0;
    QObject::connect(&video, &VideoLoader::videoLoaded, [&] { ++loaded; });
    QObject::connect(&video, &VideoLoader::frameChanged, [&] { ++changed; });
    QObject::connect(&video, &VideoLoader::videoLoadFailed, [&] { ++failed; });
    check(video.loadVideo(first), "accept initial video");
    check(!video.isVideoLoaded() && loaded == 0, "opening is asynchronous");
    check(waitFor([&] { return video.getCurrentFrameNumber() == 0; }), "first frame arrives");
    check(video.getTotalFrames() == 20 && loaded == 1, "video metadata applied");
    // Superseding a pending open must not publish its metadata or its frames.
    check(video.loadVideo(first), "accept superseded open");
    check(video.loadVideo(second), "accept replacement open");
    check(waitFor([&] { return video.getCurrentFrameNumber() == 0; }), "replacement frame arrives");
    check(loaded == 2, "only latest open published");
    check(video.getCurrentQImageFrame().pixelColor(0, 0).red() > 200, "replacement frame belongs to latest video");
    video.seekToFrame(19);
    video.seekToFrame(12);
    check(waitFor([&] { return video.getCurrentFrameNumber() == 12; }), "latest seek wins");
    const int before = changed;
    video.seekToFrame(17, true);
    check(waitFor([&] { return video.getCurrentFrameNumber() == 17; }), "suppressed seek arrives");
    check(changed == before, "async seek preserves suppressEmit");
    check(video.loadVideo(QDir(root).filePath("missing.avi")), "accept failed-open request");
    check(waitFor([&] { return failed == 1; }), "open failure reported asynchronously");
    check(!video.isVideoLoaded(), "failed open leaves video unloaded");
}

static void testPlaybackNavigation(const QString& root)
{
    const QString path = QDir(root).filePath("navigation.avi");
    makeVideo(path, 100);
    MainWindow window;
    auto* video = window.findChild<VideoLoader*>();
    auto* slider = window.findChild<QSlider*>("frameSlider");
    auto* position = window.findChild<QSpinBox*>("framePosition");
    check(video && slider && position, "find navigation controls");
    video->setPlaybackSpeed(20.0);
    check(video->getPlaybackSpeed() == 20.0, "20x playback is supported without clamping to 10x");
    video->setPlaybackSpeed(1.0);
    video->loadVideo(path);
    check(waitFor([&] { return video->getCurrentFrameNumber() == 0; }), "navigation fixture opens");
    int changed = 0;
    bool playing = false;
    QObject::connect(video, &VideoLoader::playbackStateChanged, [&](bool value, double) { playing = value; });
    QObject::connect(video, &VideoLoader::frameChanged, [&] { ++changed; });
    video->seekToFrame(10);
    check(waitFor([&] { return video->getCurrentFrameNumber() == 10; }), "navigation seek completes");
    check(changed == 1, "control synchronization does not present the frame again");
    check(slider->value() == 10 && position->value() == 10, "controls follow presented frame");

    // Force a playback tick before queued decode results can reach the GUI.
    video->play();
    video->seekToFrame(18);
    check(QMetaObject::invokeMethod(video, "processNextFrame", Qt::DirectConnection), "invoke playback tick");
    video->pause();
    check(waitFor([&] { return video->getCurrentFrameNumber() == 18; }), "playback tick preserves explicit seek");

    const int before = changed;
    slider->setSliderDown(true);
    slider->setValue(5);
    slider->setValue(9);
    slider->setValue(14);
    check(changed == before, "drag events are coalesced before decoding");
    slider->setSliderDown(false);
    check(waitFor([&] { return video->getCurrentFrameNumber() == 14; }), "release seeks exact final frame");
    check(changed == before + 1, "only final position presented for a short drag");

    video->play();
    slider->setSliderDown(true);
    check(!playing, "drag pauses playback");
    slider->setValue(7);
    slider->setSliderDown(false);
    check(playing, "release resumes previously playing video");
    video->pause();
    check(waitFor([&] { return video->getCurrentFrameNumber() == 7; }), "resumed drag keeps release target");
}

static void testRunImport(const QString& root)
{
    const QString project = QDir(root).filePath("import");
    const QString run = QDir(project).filePath("yawt/video/PROC_2026-10-06-120000");
    check(QDir().mkpath(run), "create import fixture");
    const QString path = QDir(project).filePath("video.avi");
    const QString other = QDir(project).filePath("other.avi");
    makeVideo(path, 100);
    makeVideo(other, 200);
    check(WormsJson::write(QDir(run).filePath("worms.json"), document(81)), "write imported run");
    writeFile(QDir(run).filePath("thresholding.json"), "{\"algorithm\":0,\"globalThresholdValue\":77}");
    writeFile(QDir(run).filePath("video_tracks.csv"), "frame,x,y\n7,10,20\n");
    TableItems::AnnotationItem start;
    start.id = 82;
    start.type = TableItems::ItemType::StartPoint;
    start.initialCentroid = QPointF(5, 5);
    check(WormsJson::writeRoiPoints(QDir(run).filePath("roi_points.json"), path, 7, {start}),
          "write imported ROI points");
    MainWindow window;
    auto* video = window.findChild<VideoLoader*>();
    auto* storage = window.findChild<TrackingDataStorage*>();
    check(video && storage, "main window supplies video and tracking storage");
    window.loadRunFromDirectoryPath(run);
    check(storage->getAllTracks().empty(), "import request returns before applying data");
    video->loadVideo(other);
    check(waitFor([&] { return video->getCurrentFrameNumber() == 0; }), "replacement video opens during import");
    check(storage->getAllTracks().empty(), "superseded import cannot overwrite replacement video");
    window.loadRunFromDirectoryPath(run);
    check(waitFor([&] { return storage->getAllTracks().count(81) == 1
                              && video->getCurrentFrameNumber() == 0; }), "prepared import applied after video opens");
    check(video->getCurrentVideoPath() == path, "import displays associated video");
    check(video->getCurrentThresholdSettings().globalThresholdValue == 77, "import applies threshold snapshot");
    check(storage->getAllItems().size() == 2, "import applies worms and ROI points");
    check(!video->getCurrentThresholdSettings().enableBackgroundSubtraction,
          "older threshold snapshots default subtraction off");
    writeFile(QDir(run).filePath("thresholding.json"),
              "{\"algorithm\":0,\"globalThresholdValue\":77,\"enableBackgroundSubtraction\":true}");
    MainWindow restored;
    auto* restoredVideo = restored.findChild<VideoLoader*>();
    restored.loadRunFromDirectoryPath(run);
    check(waitFor([&] { return restoredVideo->getCurrentFrameNumber() == 0; }),
          "import with subtraction opens video");
    const auto restoredSettings = restoredVideo->getCurrentThresholdSettings();
    check(restoredSettings.enableBackgroundSubtraction && !restoredSettings.medianBackground.empty(),
          "import restores subtraction and rebuilds transient background model");
}

// Opt-in throughput experiment: fresh application/cache for each run, real Qt
// event-loop timing, and alternating order to reduce warm filesystem bias.
static void benchmarkPlaybackSpeed(const QString& root, bool light = false)
{
    for (const auto size : {cv::Size(640, 480), cv::Size(1920, 1080)}) {
        const QString path = QDir(root).filePath(QString("speed-%1.avi").arg(size.width));
        cv::VideoWriter writer(path.toStdString(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, size);
        check(writer.isOpened(), "open benchmark writer");
        cv::Mat frame(size, CV_8UC3);
        cv::RNG random(12345);
        random.fill(frame, cv::RNG::UNIFORM, 0, 256);
        if (light) {
            // Smooth spatial gradient provides an inexpensive decode control.
            for (int y = 0; y < size.height; ++y)
                for (int x = 0; x < size.width; ++x)
                    frame.at<cv::Vec3b>(y, x) = cv::Vec3b(x * 255 / size.width, y * 255 / size.height, 100);
        }
        for (int i = 0; i < 240; ++i) writer.write(frame);
        writer.release();
        for (const double speed : {10.0, 20.0, 20.0, 10.0}) {
            MainWindow window;
            window.resize(1000, 700);
            window.show();
            auto* video = window.findChild<VideoLoader*>();
            video->loadVideo(path);
            check(waitFor([&] { return video->getCurrentFrameNumber() == 0; }), "benchmark opens");
            video->setPlaybackSpeed(speed);
            check(video->getPlaybackSpeed() == speed, "benchmark speed is not clamped");
            QEventLoop loop;
            QTimer deadline;
            deadline.setSingleShot(true);
            QObject::connect(&deadline, &QTimer::timeout, &loop, &QEventLoop::quit);
            QObject::connect(video, &VideoLoader::frameChanged, &loop, [&](int number, const QImage&) {
                if (number == 239) loop.quit();
            });
            QElapsedTimer elapsed;
            elapsed.start();
            deadline.start(15000);
            video->play();
            loop.exec();
            const double seconds = elapsed.nsecsElapsed() / 1e9;
            video->pause();
            check(video->getCurrentFrameNumber() == 239, "benchmark finishes before timeout");
            std::printf("BENCH %dx%d requested=%.0fx seconds=%.3f fps=%.1f achieved=%.2fx\n",
                        size.width, size.height, speed, seconds, 239 / seconds, 239 / seconds / 25);
            std::fflush(stdout);
        }
    }
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QLoggingCategory::setFilterRules("*.debug=false");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    QTemporaryDir temp;
    try {
        check(temp.isValid(), "create temporary fixtures");
        if (app.arguments().contains("--playback-only")) {
            testVideo(temp.path());
            testPlaybackNavigation(temp.path());
            std::puts("Playback regressions passed.");
            return 0;
        }
        if (app.arguments().contains("--benchmark-speed")) {
            benchmarkPlaybackSpeed(temp.path(), app.arguments().contains("--light"));
            return 0;
        }
        testGeometrySerialization();
        if (app.arguments().contains("--geometry-only")) {
            std::puts("Geometry serialization regressions passed.");
            return 0;
        }
        testIndex(temp.path());
        testMetadata(temp.path());
        testAnalysis(temp.path());
#ifdef Q_OS_UNIX
        testSlowDisk(temp.path());
#endif
        testVideo(temp.path());
        testPlaybackNavigation(temp.path());
        testRunImport(temp.path());
        std::puts("Disk access regressions passed.");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
}
