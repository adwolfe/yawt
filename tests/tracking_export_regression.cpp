#include "src/core/trackingmanager.h"
#include "src/core/trackingexportworker.h"
#include "src/gui/trackingprogressdialog.h"
#include "src/utils/yawtjsonio.h"
#include <QApplication>
#include <QDialogButtonBox>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QLabel>
#include <QLoggingCategory>
#include <QProgressBar>
#include <QPushButton>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <QtEndian>
#include <algorithm>
#include <cmath>
#include <atomic>
#include <cstdio>
#include <functional>
#include <stdexcept>
#ifdef Q_OS_UNIX
#include <csignal>
#include <sys/resource.h>
#endif

static void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
static bool waitFor(const std::function<bool()>& ready) {
    QElapsedTimer timer; timer.start();
    while (!ready() && timer.elapsed() < 30000) {
        QApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(1);
    }
    return ready();
}
static QByteArray readBytes(const QString& path) {
    QFile file(path); check(file.open(QIODevice::ReadOnly), "read output"); return file.readAll();
}
static QByteArray zipEntry(const QString& path, const QByteArray& name) {
    const QByteArray bytes = readBytes(path);
    qsizetype offset = 0;
    while (offset + 30 <= bytes.size()) {
        const auto* header = reinterpret_cast<const uchar*>(bytes.constData() + offset);
        if (qFromLittleEndian<quint32>(header) != 0x04034b50) break;
        check(qFromLittleEndian<quint16>(header + 8) == 0, "stored workbook entry");
        const auto size = qFromLittleEndian<quint32>(header + 18);
        const auto nameSize = qFromLittleEndian<quint16>(header + 26);
        const auto extraSize = qFromLittleEndian<quint16>(header + 28);
        const qsizetype data = offset + 30 + nameSize + extraSize;
        check(data + size <= bytes.size(), "complete workbook entry");
        if (bytes.mid(offset + 30, nameSize) == name) return bytes.mid(data, size);
        offset = data + size;
    }
    throw std::runtime_error("missing workbook sheet");
}
static Tracking::DetectedBlob blob(int frame) {
    Tracking::DetectedBlob b;
    b.isValid = true; b.area = 12; b.convexHullArea = 12;
    b.centroid = QPointF(3 * frame, 4 * frame);
    b.boundingBox = QRectF(0, 0, 8, 2);
    b.contourPoints = {{0,0}, {8,0}, {8,2}, {0,2}};
    b.centerline.points = {{0,0}, {3,4}};
    b.centerline.topology = Tracking::TopologyState::Clean;
    return b;
}
static TrackingExportSnapshot fixture(const QString& directory) {
    TrackingExportSnapshot s;
    s.directory = directory; s.workbookPath = QDir(directory).filePath("fixture_tracks.xlsx");
    s.worms.videoPath = "fixture.avi"; s.worms.keyFrame = 0;
    s.fps = 10; s.umPerPixel = 2; s.frameSize = cv::Size(100,80);
    TableItems::AnnotationItem item; item.id = 7; item.type = TableItems::ItemType::Worm;
    s.items.append(item); s.worms.items.append(item);
    item.id = 8; item.type = TableItems::ItemType::StartPoint; item.initialCentroid = QPointF(1,2); s.items.append(item);
    const Tracking::TrackPointQuality qualities[] = {
        Tracking::TrackPointQuality::Single, Tracking::TrackPointQuality::Merged,
        Tracking::TrackPointQuality::Merged, Tracking::TrackPointQuality::Lost,
        Tracking::TrackPointQuality::Split};
    for (int frame = 4; frame >= 0; --frame) {
        Tracking::TrackPoint point{}; point.frameNumber = frame;
        point.position = cv::Point2f(3*frame,4*frame); point.searchWindow = QRectF(0,0,8,2); point.quality = qualities[frame];
        s.worms.tracks[7].push_back(point); s.worms.blobsByFrame[frame][7] = blob(frame);
    }
    s.headTailSwaps[7] = {1}; s.centerlineReviews[7] = {2};
    s.motionSwaps[7] = {1}; s.geometrySwaps[7] = {2};
    SharedBlob merged; merged.uniqueId = 2; merged.frameNumber = 1;
    merged.participatingWormTrackerIDs = {7}; merged.contourPoints = {{1,2},{3,4}};
    s.mergeRecords[1].append(merged); s.nextSharedBlobId = 3;
    s.wormToSharedBlobId[7] = 2; s.splitResolutions[4][7] = blob(4);
    return s;
}
static void testWorkerOutputs(const QString& directory) {
    auto s = fixture(directory);
    WormsJson::Document expected = s.worms;
    std::reverse(expected.tracks[7].begin(), expected.tracks[7].end());
    expected.mergeState = s.mergeStateToJson();
    // Shared blob maps detach cleanly when live data is changed after capture.
    auto live = s.worms.blobsByFrame;
    live[0][7].centerline.points.clear();
    check(s.findBlob(0,7)->centerline.points.size() == 2, "snapshot independent of later blob edits");
    TrackingExportWorker worker(std::move(s));
    QString error; bool finished = false;
    QObject::connect(&worker, &TrackingExportWorker::finished, [&](const QString& e) { finished = true; error = e; });
    worker.run();
    check(finished && error.isEmpty(), "worker writes complete output set");
    const auto saved = YawtJsonIO::readJsonDocument(QDir(directory).filePath("worms.json")).object();
    check(saved == WormsJson::toJson(expected), "tracking geometry and merge schema preserved");
    const auto summary = YawtJsonIO::readJsonDocument(QDir(directory).filePath("worm_summary.json")).object();
    const auto stats = summary["worms"].toObject()["7"].toObject();
    check(stats["frames_single"].toInt() == 1 && stats["frames_merged"].toInt() == 2
          && stats["frames_lost"].toInt() == 1 && stats["frames_split"].toInt() == 1, "quality counts");
    check(stats["total_distance_px"].toDouble() == 10 && stats["net_displacement_px"].toDouble() == 20, "lost frames break distance continuity");
    check(stats["mean_body_length_px"].toDouble() == 5 && stats["mean_area_px2"].toDouble() == 12
          && stats["mean_aspect_ratio"].toDouble() == 4, "morphology means exclude merged and lost frames");
    check(std::abs(stats["mean_speed_um_per_s"].toDouble() - 200.0/3) < 1e-8, "speed and spatial calibration preserved");
    const auto report = readBytes(QDir(directory).filePath("fixture_ProcessingSummary.txt"));
    check(report.contains("Merge runs:           1") && report.contains("Longest run:          2"), "merge event summary");
    const auto references = WormsJson::readRoiPoints(QDir(directory).filePath("roi_points.json"));
    check(references.size() == 1 && references.front().type == TableItems::ItemType::StartPoint, "reference annotations saved");
    const auto workbook = QDir(directory).filePath("fixture_tracks.xlsx");
    check(zipEntry(workbook,"xl/workbook.xml").contains("Centerlines"), "workbook retains four sheets");
    check(zipEntry(workbook,"xl/worksheets/sheet4.xml").contains("Point10Y"), "resampled centerline columns");
    check(zipEntry(workbook,"xl/worksheets/sheet2.xml").contains("Start Point"), "workbook reference coordinates");
    check(zipEntry(QDir(directory).filePath("fixture_headtail_swaps.xlsx"),"xl/worksheets/sheet1.xml").contains("REVIEW"), "review report saved");
}
static void populate(TrackingDataStorage& storage, int worms, int frames) {
    for (int w = 0; w < worms; ++w) {
        int id = storage.addItem(QPointF(10,10),QRectF(0,0,80,20),0,TableItems::ItemType::Worm);
        Tracking::Track track;
        for (int f = 0; f < frames; ++f) {
            auto b = blob(f);
            const double x = 100+w*100+f*.2, y = 100+20*std::sin(f*.02+w);
            b.centroid = QPointF(x,y); b.boundingBox = QRectF(x-40,y-10,80,20);
            b.contourPoints.clear(); b.centerline.points.clear();
            for (int j=0;j<100;++j) {
                const double a = j*6.28318530718/100;
                b.contourPoints.emplace_back(int(x+40*std::cos(a)),int(y+10*std::sin(a)));
            }
            for (int j=0;j<25;++j) b.centerline.points.emplace_back(x-40+j*80.0/24,y+2*std::sin(j*.3+f*.01));
            storage.setDetectedBlobForFrame(f,id,b);
            Tracking::TrackPoint p{}; p.frameNumber=f; p.position=cv::Point2f(x,y); p.searchWindow=b.boundingBox; p.quality=Tracking::TrackPointQuality::Single;
            track.push_back(p);
        }
        storage.setTrackForWorm(id,track);
    }
}
static void testBackgroundSave(const QString& directory) {
    TrackingDataStorage storage; populate(storage,8,2000);
    TrackingManager manager(&storage);
    manager.setLoadedRunContext("async.avi",directory,0); manager.setCenterlineEnabled(false);
    bool completed=false, edited=false, announced=false; QString error;
    int ticks=0, maxGap=0; QElapsedTimer gap, total; gap.start(); total.start();
    QTimer heartbeat;
    QObject::connect(&heartbeat,&QTimer::timeout,[&] { ++ticks; maxGap=std::max(maxGap,int(gap.restart())); });
    heartbeat.start(10);
    QObject::connect(&manager,&TrackingManager::savingProgress,[&](int percent,const QString&) {
        announced=true;
        check(!completed, "save progress precedes completion");
        if (percent==0) check(!QFile::exists(QDir(directory).filePath("async_tracks.xlsx")), "no preliminary workbook");
        if (percent==45 && !edited) { edited=true; storage.clearAllData(); }
        if (percent==100) check(QFile::exists(QDir(directory).filePath("worm_summary.json")), "completion requires saved reports");
    });
    QObject::connect(&manager,&TrackingManager::centerlineFinished,[&] { completed=true; });
    QObject::connect(&manager,&TrackingManager::trackingFailed,[&](const QString& e) {error=e;});
    manager.startCenterlineComputation();
    check(announced && !completed, "save starts asynchronously even with centerlines disabled");
    // A concurrent rerun cannot replace the snapshot or start another save.
    manager.startCenterlineComputation();
    check(waitFor([&] { return completed || !error.isEmpty(); }), "asynchronous finalization completes");
    heartbeat.stop();
    const double seconds = total.nsecsElapsed()/1e9;
    if (!error.isEmpty()) throw std::runtime_error(error.toStdString());
    check(edited && ticks>10 && maxGap<750, "UI event loop remains responsive during export");
    WormsJson::Document restored;
    check(WormsJson::read(QDir(directory).filePath("worms.json"),restored), "load background output");
    check(restored.tracks.size()==8 && restored.blobsByFrame.size()==2000, "background save survives live storage clearing");
    std::printf("Background export: %.3f s, %d UI timer ticks, longest gap %d ms\n",seconds,ticks,maxGap);
}

static void testCenterlineAndRepeatedSave(const QString& directory) {
    TrackingDataStorage storage; populate(storage,2,6);
    TrackingManager manager(&storage); manager.setLoadedRunContext("centerline.avi",directory,0);
    manager.setSmoothCenterline(false);
    Centerline::CenterlineSnakeParams params; params.iterations=5; params.nPoints=10;
    manager.setCenterlineSnakeParams(params);
    int completions=0, centerlineUpdates=0; QString error;
    QObject::connect(&manager,&TrackingManager::centerlineProgress,[&](int) {++centerlineUpdates;});
    QObject::connect(&manager,&TrackingManager::centerlineFinished,[&] {++completions;});
    QObject::connect(&manager,&TrackingManager::trackingFailed,[&](const QString& e) {error=e;});
    for(int run=1;run<=2;++run) {
        manager.startCenterlineComputation();
        check(waitFor([&] {return completions==run || !error.isEmpty();}),"centerline workers stop and results save");
        if(!error.isEmpty()) throw std::runtime_error(error.toStdString());
        const auto rows=zipEntry(QDir(directory).filePath("centerline_tracks.xlsx"),"xl/worksheets/sheet1.xml");
        check(rows.count("<row ")==13,"centerline rerun exports stored tracks after transient tracks were cleared");
    }
    check(centerlineUpdates>0,"background centerline phase was exercised");
    const auto root=YawtJsonIO::readJsonDocument(QDir(directory).filePath("worms.json")).object();
    for(const auto& [id,points]:storage.getAllTracks()) {
        const auto saved=root["tracks"].toObject()[QString::number(id)].toArray();
        for(size_t i=0;i<points.size();++i)
            check(saved[int(i)].toObject()==WormsJson::trackPointToJson(points[i],storage.findDetectedBlob(points[i].frameNumber,id)),"final centerline geometry and derived track data saved");
    }
}

static void testRetiredDataRelease(const QString& directory) {
    const auto* guiThread=QThread::currentThread();
    std::atomic_bool releasedOffGui{false};
    std::shared_ptr<void> retired(new int(1),[&](void* value) {
        delete static_cast<int*>(value);
        releasedOffGui.store(QThread::currentThread()!=guiThread);
    });
    QThread thread;
    auto* worker=new TrackingExportWorker(fixture(directory),std::move(retired));
    worker->moveToThread(&thread);
    bool finished=false; QString error;
    QObject::connect(&thread,&QThread::started,worker,&TrackingExportWorker::run);
    QObject::connect(worker,&TrackingExportWorker::finished,&thread,[&](const QString& e) {finished=true; error=e;});
    QObject::connect(worker,&TrackingExportWorker::finished,&thread,&QThread::quit,Qt::DirectConnection);
    QObject::connect(&thread,&QThread::finished,worker,&QObject::deleteLater);
    thread.start();
    check(waitFor([&] {return finished && thread.isFinished();}),"saving worker shuts down");
    thread.wait();
    check(error.isEmpty() && releasedOffGui.load(),"temporary data is released on the worker before completion");
}
static void testFailureAndDialog(const QString& directory) {
    TrackingManager unconfigured;
    bool rejected=false;
    QObject::connect(&unconfigured,&TrackingManager::trackingFailed,[&](const QString&) {rejected=true;});
    unconfigured.startCenterlineComputation();
    check(rejected,"an empty run never writes a workbook into the current directory");
    auto snapshot = fixture(directory);
    snapshot.workbookPath = directory; // Cannot replace a directory with a workbook.
    QString error;
    TrackingExportWorker worker(std::move(snapshot));
    QObject::connect(&worker,&TrackingExportWorker::finished,[&](const QString& e) {error=e;});
    worker.run(); check(!error.isEmpty(), "workbook failure is reported");
    TrackingDataStorage storage;
    TrackingManager manager(&storage);
    manager.setLoadedRunContext("failed.avi",directory+"/missing",0); manager.setCenterlineEnabled(false);
    bool completed=false, failed=false;
    QObject::connect(&manager,&TrackingManager::centerlineFinished,[&] {completed=true;});
    QObject::connect(&manager,&TrackingManager::trackingFailed,[&](const QString&) {failed=true;});
    manager.startCenterlineComputation();
    check(waitFor([&] {return failed;}), "manager reports save failure");
    check(!completed, "save failure never reports successful completion");
    TrackingProgressDialog dialog;
    auto* buttons=dialog.findChild<QDialogButtonBox*>(); auto* progress=dialog.findChild<QProgressBar*>("overallProgressBar");
    dialog.onTrackingSuccessfullyFinished(); dialog.onSavingProgress(45,"Saving results: Excel workbook...");
    dialog.show(); dialog.reject();
    check(dialog.isVisible() && !buttons->button(QDialogButtonBox::Cancel)->isEnabled(), "saving cannot be dismissed as completed");
    check(!dialog.close() && dialog.isVisible(),"window close waits for saving");
    dialog.onTrackingFailed(error);
    check(buttons->button(QDialogButtonBox::Cancel)->isEnabled(), "failed save allows closing");
    dialog.reject(); check(!dialog.isVisible(), "failed save dialog closes");
    dialog.onSavingProgress(95,"Releasing temporary data..."); dialog.onCenterlineFinished();
    check(progress->value()==100 && buttons->button(QDialogButtonBox::Cancel)->isEnabled(), "only successful completion enables Close at 100 percent");
}
static void testAtomicJson(const QString& directory) {
#ifdef Q_OS_UNIX
    const QString path=QDir(directory).filePath("atomic.json");
    const QByteArray previous("previous completed result");
    {QFile file(path); check(file.open(QIODevice::WriteOnly),"write previous result"); file.write(previous);}
    QJsonArray entries; for(int i=0;i<4096;++i) entries.append(QString::number(i*7919)+"-"+QString::number(i*104729));
    struct rlimit oldLimit; check(getrlimit(RLIMIT_FSIZE,&oldLimit)==0,"read file size limit");
    auto limit=oldLimit; limit.rlim_cur=512;
    auto oldSignal=std::signal(SIGXFSZ,SIG_IGN);
    check(setrlimit(RLIMIT_FSIZE,&limit)==0,"set short-write limit");
    QString error; const bool success=YawtJsonIO::writeCompressedJsonDocument(path,QJsonDocument(entries),&error);
    const int restored=setrlimit(RLIMIT_FSIZE,&oldLimit); std::signal(SIGXFSZ,oldSignal);
    check(restored==0 && !success && !error.isEmpty(), "short write surfaces an error");
    check(readBytes(path)==previous,"failed atomic write preserves previous results");
#endif
}
int main(int argc,char** argv) {
    QApplication app(argc,argv); QLoggingCategory::setFilterRules("*.debug=false\n*.info=false");
    try {
        QTemporaryDir root; check(root.isValid(),"temporary directory");
        for(const QString& name:{"schema","async","centerline","cleanup","failure","atomic"}) check(QDir().mkpath(root.filePath(name)),"create output directory");
        testWorkerOutputs(root.filePath("schema"));
        testBackgroundSave(root.filePath("async"));
        testCenterlineAndRepeatedSave(root.filePath("centerline"));
        testRetiredDataRelease(root.filePath("cleanup"));
        testFailureAndDialog(root.filePath("failure"));
        testAtomicJson(root.filePath("atomic"));
        std::puts("Tracking export regressions passed");
    } catch(const std::exception& e) {std::fprintf(stderr,"FAIL: %s\n",e.what()); return 1;}
}
