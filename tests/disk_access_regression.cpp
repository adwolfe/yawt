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
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QLoggingCategory::setFilterRules("*.debug=false");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    QTemporaryDir temp;
    try {
        check(temp.isValid(), "create temporary fixtures");
        testIndex(temp.path());
        testMetadata(temp.path());
        testAnalysis(temp.path());
#ifdef Q_OS_UNIX
        testSlowDisk(temp.path());
#endif
        testVideo(temp.path());
        testRunImport(temp.path());
        std::puts("Disk access regressions passed.");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
}
