// trackingmanager.cpp
#include "trackingmanager.h"
#include "debugutils.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QPair>
#include <QRectF>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QThread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm> // For std::reverse, std::min, std::max
#include <numeric>   // For std::accumulate
#include <limits>    // For std::numeric_limits
#include <cmath>     // For std::ceil

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>

// Helper: Save frame-atomic merge/split state to JSON.
// Implemented as a free function in this translation unit so we don't need to modify headers.
// Parameters:
//  - directoryPath: folder to write the JSON file into (must be non-empty).
//  - nextPhysicalBlobId: value of m_nextPhysicalBlobId to persist.
//  - frameMergeRecords: map of frame->list of FrameSpecificPhysicalBlob
//  - splitResolutionMap: map of frame->(map of wormId->DetectedBlob)
//  - wormToPhysicalBlobIdMap: map wormId->physicalBlobId
//  - thresholdSettings: threshold settings to persist for compatibility checks
static void saveFrameAtomicStateToJson(const QString& directoryPath,
                                      int nextPhysicalBlobId,
                                      const QMap<int, QList<FrameSpecificPhysicalBlob>>& frameMergeRecords,
                                      const QMap<int, QMap<int, Tracking::DetectedBlob>>& splitResolutionMap,
                                      const QMap<int, int>& wormToPhysicalBlobIdMap,
                                      const Thresholding::ThresholdSettings& thresholdSettings)
{
    if (directoryPath.isEmpty()) {
        qWarning() << "saveFrameAtomicStateToJson: No directory provided; skipping save.";
        return;
    }

    QJsonObject root;
    root["version"] = 1;
    root["nextPhysicalBlobId"] = nextPhysicalBlobId;

    // Serialize threshold settings (minimal representation)
    QJsonObject threshObj;
    threshObj["algorithm"] = static_cast<int>(thresholdSettings.algorithm);
    threshObj["globalThresholdValue"] = thresholdSettings.globalThresholdValue;
    threshObj["adaptiveBlockSize"] = thresholdSettings.adaptiveBlockSize;
    threshObj["adaptiveCValue"] = thresholdSettings.adaptiveCValue;
    threshObj["assumeLightBackground"] = thresholdSettings.assumeLightBackground;
    threshObj["enableBlur"] = thresholdSettings.enableBlur;
    threshObj["blurKernelSize"] = thresholdSettings.blurKernelSize;
    threshObj["blurSigmaX"] = thresholdSettings.blurSigmaX;
    root["thresholdSettings"] = threshObj;

    // wormToPhysicalBlobIdMap
    QJsonObject wormToPhysObj;
    for (auto it = wormToPhysicalBlobIdMap.constBegin(); it != wormToPhysicalBlobIdMap.constEnd(); ++it) {
        wormToPhysObj[QString::number(it.key())] = it.value();
    }
    root["wormToPhysicalBlobIdMap"] = wormToPhysObj;

    // frameMergeRecords
    QJsonObject frameMergeObj;
    for (auto fit = frameMergeRecords.constBegin(); fit != frameMergeRecords.constEnd(); ++fit) {
        int frameNum = fit.key();
        const QList<FrameSpecificPhysicalBlob>& blobList = fit.value();
        QJsonArray blobsArr;
        for (const FrameSpecificPhysicalBlob& pb : blobList) {
            QJsonObject pbObj;
            pbObj["uniqueId"] = pb.uniqueId;
            pbObj["frameNumber"] = pb.frameNumber;
            pbObj["currentArea"] = pb.currentArea;

            // Centroid (cv::Point2f)
            QJsonObject cObj;
            cObj["x"] = static_cast<double>(pb.currentCentroid.x);
            cObj["y"] = static_cast<double>(pb.currentCentroid.y);
            pbObj["currentCentroid"] = cObj;

            // Bounding box (QRectF)
            QJsonObject bObj;
            bObj["x"] = pb.currentBoundingBox.x();
            bObj["y"] = pb.currentBoundingBox.y();
            bObj["width"] = pb.currentBoundingBox.width();
            bObj["height"] = pb.currentBoundingBox.height();
            pbObj["currentBoundingBox"] = bObj;

            // Serialize contour points for this physical blob (vector<cv::Point>)
            QJsonArray contourArr;
            for (const cv::Point& pt : pb.contourPoints) {
                QJsonArray ptArr;
                ptArr.append(pt.x);
                ptArr.append(pt.y);
                contourArr.append(ptArr);
            }
            pbObj["contourPoints"] = contourArr;

            // participatingWormTrackerIDs (QSet<int>)
            QJsonArray partArr;
            for (int wid : pb.participatingWormTrackerIDs) partArr.append(wid);
            pbObj["participatingWormTrackerIDs"] = partArr;

            pbObj["selectedByWormTrackerId"] = pb.selectedByWormTrackerId;

            blobsArr.append(pbObj);
        }
        frameMergeObj[QString::number(frameNum)] = blobsArr;
    }
    root["frameMergeRecords"] = frameMergeObj;

    // splitResolutionMap
    QJsonObject splitMapObj;
    for (auto sfit = splitResolutionMap.constBegin(); sfit != splitResolutionMap.constEnd(); ++sfit) {
        int frameNum = sfit.key();
        const QMap<int, Tracking::DetectedBlob>& wormMap = sfit.value();
        QJsonObject wormMapObj;
        for (auto wit = wormMap.constBegin(); wit != wormMap.constEnd(); ++wit) {
            int wormId = wit.key();
            const Tracking::DetectedBlob& db = wit.value();
            QJsonObject dbObj;
            dbObj["isValid"] = db.isValid;
            dbObj["area"] = db.area;
            dbObj["convexHullArea"] = db.convexHullArea;
            dbObj["touchesROIboundary"] = db.touchesROIboundary;

            QJsonObject cent;
            cent["x"] = db.centroid.x();
            cent["y"] = db.centroid.y();
            dbObj["centroid"] = cent;

            QJsonObject bbox;
            bbox["x"] = db.boundingBox.x();
            bbox["y"] = db.boundingBox.y();
            bbox["width"] = db.boundingBox.width();
            bbox["height"] = db.boundingBox.height();
            dbObj["boundingBox"] = bbox;

            // Serialize contour points for this detected blob (if available)
            QJsonArray dbContourArr;
            for (const cv::Point& pt : db.contourPoints) {
                QJsonArray ptArr;
                ptArr.append(pt.x);
                ptArr.append(pt.y);
                dbContourArr.append(ptArr);
            }
            dbObj["contourPoints"] = dbContourArr;

            wormMapObj[QString::number(wormId)] = dbObj;
        }
        splitMapObj[QString::number(frameNum)] = wormMapObj;
    }
    root["splitResolutionMap"] = splitMapObj;

    // Write to file
    QString outPath = QDir(directoryPath).absoluteFilePath("frame_atomic_state.json");
    QFile f(outPath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        qWarning() << "saveFrameAtomicStateToJson: Failed to open" << outPath << "for writing.";
        return;
    }
    QJsonDocument doc(root);
    QByteArray data = doc.toJson(QJsonDocument::Indented);
    qint64 written = f.write(data);
    f.close();
    if (written <= 0) {
        qWarning() << "saveFrameAtomicStateToJson: Failed to write data to" << outPath;
    } else {
        qDebug() << "saveFrameAtomicStateToJson: Wrote frame-atomic state to" << outPath << "(" << written << "bytes )";
    }
} // end saveFrameAtomicStateToJson


// Helper: Load frame-atomic merge/split state from JSON into provided output references.
// Returns true on successful load.
static bool loadFrameAtomicStateFromJson(const QString& directoryPath,
                                         int &outNextPhysicalBlobId,
                                         QMap<int, QList<FrameSpecificPhysicalBlob>>& outFrameMergeRecords,
                                         QMap<int, QMap<int, Tracking::DetectedBlob>>& outSplitResolutionMap,
                                         QMap<int, int>& outWormToPhysicalBlobIdMap,
                                         Thresholding::ThresholdSettings& outThresholdSettings)
{
    if (directoryPath.isEmpty()) {
        qWarning() << "loadFrameAtomicStateFromJson: No directory provided; skipping load.";
        return false;
    }
    QString inPath = QDir(directoryPath).absoluteFilePath("frame_atomic_state.json");
    QFile f(inPath);
    if (!f.exists()) {
        qDebug() << "loadFrameAtomicStateFromJson: No saved state file at" << inPath;
        return false;
    }
    if (!f.open(QIODevice::ReadOnly)) {
        qWarning() << "loadFrameAtomicStateFromJson: Failed to open" << inPath << "for reading.";
        return false;
    }
    QByteArray data = f.readAll();
    f.close();
    QJsonParseError perr;
    QJsonDocument doc = QJsonDocument::fromJson(data, &perr);
    if (perr.error != QJsonParseError::NoError) {
        qWarning() << "loadFrameAtomicStateFromJson: JSON parse error in" << inPath << ":" << perr.errorString();
        return false;
    }
    if (!doc.isObject()) {
        qWarning() << "loadFrameAtomicStateFromJson: Unexpected JSON root (not an object) in" << inPath;
        return false;
    }
    QJsonObject root = doc.object();
    int version = root.value("version").toInt(0);
    if (version != 1) {
        qWarning() << "loadFrameAtomicStateFromJson: Unsupported version" << version << "in" << inPath;
        return false;
    }
    outNextPhysicalBlobId = root.value("nextPhysicalBlobId").toInt(outNextPhysicalBlobId);

    // Threshold settings (optional fallback)
    if (root.contains("thresholdSettings") && root["thresholdSettings"].isObject()) {
        QJsonObject t = root["thresholdSettings"].toObject();
        outThresholdSettings.algorithm = static_cast<Thresholding::ThresholdAlgorithm>(t.value("algorithm").toInt(static_cast<int>(outThresholdSettings.algorithm)));
        outThresholdSettings.globalThresholdValue = t.value("globalThresholdValue").toInt(outThresholdSettings.globalThresholdValue);
        outThresholdSettings.adaptiveBlockSize = t.value("adaptiveBlockSize").toInt(outThresholdSettings.adaptiveBlockSize);
        outThresholdSettings.adaptiveCValue = t.value("adaptiveCValue").toDouble(outThresholdSettings.adaptiveCValue);
        outThresholdSettings.assumeLightBackground = t.value("assumeLightBackground").toBool(outThresholdSettings.assumeLightBackground);
        outThresholdSettings.enableBlur = t.value("enableBlur").toBool(outThresholdSettings.enableBlur);
        outThresholdSettings.blurKernelSize = t.value("blurKernelSize").toInt(outThresholdSettings.blurKernelSize);
        outThresholdSettings.blurSigmaX = t.value("blurSigmaX").toDouble(outThresholdSettings.blurSigmaX);
    }

    // wormToPhysicalBlobIdMap
    outWormToPhysicalBlobIdMap.clear();
    if (root.contains("wormToPhysicalBlobIdMap") && root["wormToPhysicalBlobIdMap"].isObject()) {
        QJsonObject wmap = root["wormToPhysicalBlobIdMap"].toObject();
        for (auto it = wmap.constBegin(); it != wmap.constEnd(); ++it) {
            bool ok = false;
            int key = it.key().toInt(&ok);
            if (!ok) continue;
            outWormToPhysicalBlobIdMap.insert(key, it.value().toInt());
        }
    }

    // frameMergeRecords
    outFrameMergeRecords.clear();
    if (root.contains("frameMergeRecords") && root["frameMergeRecords"].isObject()) {
        QJsonObject fm = root["frameMergeRecords"].toObject();
        for (auto fit = fm.constBegin(); fit != fm.constEnd(); ++fit) {
            bool ok = false;
            int frameNum = fit.key().toInt(&ok);
            if (!ok) continue;
            QJsonArray arr = fit.value().toArray();
            QList<FrameSpecificPhysicalBlob> list;
            list.reserve(arr.size());
            for (const QJsonValue &v : arr) {
                if (!v.isObject()) continue;
                QJsonObject pbObj = v.toObject();
                FrameSpecificPhysicalBlob pb;
                pb.uniqueId = pbObj.value("uniqueId").toInt(pb.uniqueId);
                pb.frameNumber = pbObj.value("frameNumber").toInt(pb.frameNumber);
                pb.currentArea = pbObj.value("currentArea").toDouble(pb.currentArea);
                if (pbObj.contains("currentCentroid") && pbObj["currentCentroid"].isObject()) {
                    QJsonObject c = pbObj["currentCentroid"].toObject();
                    pb.currentCentroid = cv::Point2f(static_cast<float>(c.value("x").toDouble()), static_cast<float>(c.value("y").toDouble()));
                }
                if (pbObj.contains("currentBoundingBox") && pbObj["currentBoundingBox"].isObject()) {
                    QJsonObject b = pbObj["currentBoundingBox"].toObject();
                    pb.currentBoundingBox = QRectF(b.value("x").toDouble(), b.value("y").toDouble(), b.value("width").toDouble(), b.value("height").toDouble());
                }

                // Deserialize contourPoints (array of [x,y] pairs) if present
                pb.contourPoints.clear();
                if (pbObj.contains("contourPoints") && pbObj["contourPoints"].isArray()) {
                    QJsonArray contourArr = pbObj["contourPoints"].toArray();
                    for (const QJsonValue &cv : contourArr) {
                        if (cv.isArray()) {
                            QJsonArray ptArr = cv.toArray();
                            if (ptArr.size() >= 2) {
                                int px = ptArr.at(0).toInt();
                                int py = ptArr.at(1).toInt();
                                pb.contourPoints.push_back(cv::Point(px, py));
                            }
                        }
                    }
                }

                pb.participatingWormTrackerIDs.clear();
                if (pbObj.contains("participatingWormTrackerIDs") && pbObj["participatingWormTrackerIDs"].isArray()) {
                    QJsonArray part = pbObj["participatingWormTrackerIDs"].toArray();
                    for (const QJsonValue &pid : part) pb.participatingWormTrackerIDs.insert(pid.toInt());
                }
                pb.selectedByWormTrackerId = pbObj.value("selectedByWormTrackerId").toInt(pb.selectedByWormTrackerId);
                list.append(pb);
            }
            outFrameMergeRecords.insert(frameNum, list);
        }
    }

    // splitResolutionMap
    outSplitResolutionMap.clear();
    if (root.contains("splitResolutionMap") && root["splitResolutionMap"].isObject()) {
        QJsonObject sm = root["splitResolutionMap"].toObject();
        for (auto sfit = sm.constBegin(); sfit != sm.constEnd(); ++sfit) {
            bool okf = false;
            int frameNum = sfit.key().toInt(&okf);
            if (!okf) continue;
            QJsonObject wormMapObj = sfit.value().toObject();
            QMap<int, Tracking::DetectedBlob> inner;
            for (auto wit = wormMapObj.constBegin(); wit != wormMapObj.constEnd(); ++wit) {
                bool okw = false;
                int wormId = wit.key().toInt(&okw);
                if (!okw) continue;
                if (!wit.value().isObject()) continue;
                QJsonObject dbObj = wit.value().toObject();
                Tracking::DetectedBlob db;
                db.isValid = dbObj.value("isValid").toBool(db.isValid);
                db.area = dbObj.value("area").toDouble(db.area);
                db.convexHullArea = dbObj.value("convexHullArea").toDouble(db.convexHullArea);
                db.touchesROIboundary = dbObj.value("touchesROIboundary").toBool(db.touchesROIboundary);
                if (dbObj.contains("centroid") && dbObj["centroid"].isObject()) {
                    QJsonObject cent = dbObj["centroid"].toObject();
                    db.centroid = QPointF(cent.value("x").toDouble(), cent.value("y").toDouble());
                }
                if (dbObj.contains("boundingBox") && dbObj["boundingBox"].isObject()) {
                    QJsonObject bbox = dbObj["boundingBox"].toObject();
                    db.boundingBox = QRectF(bbox.value("x").toDouble(), bbox.value("y").toDouble(),
                                            bbox.value("width").toDouble(), bbox.value("height").toDouble());
                }
                // Deserialize contourPoints for DetectedBlob if present
                db.contourPoints.clear();
                if (dbObj.contains("contourPoints") && dbObj["contourPoints"].isArray()) {
                    QJsonArray dbContourArr = dbObj["contourPoints"].toArray();
                    for (const QJsonValue &ptv : dbContourArr) {
                        if (ptv.isArray()) {
                            QJsonArray ptArr = ptv.toArray();
                            if (ptArr.size() >= 2) {
                                int px = ptArr.at(0).toInt();
                                int py = ptArr.at(1).toInt();
                                db.contourPoints.push_back(cv::Point(px, py));
                            }
                        }
                    }
                }
                inner.insert(wormId, db);
            }
            outSplitResolutionMap.insert(frameNum, inner);
        }
    }

    qDebug() << "loadFrameAtomicStateFromJson: Loaded frame-atomic state from" << inPath;
    return true;
}

// No-storage constructor implementation
TrackingManager::TrackingManager(QObject* parent)
    : QObject(parent),
      m_keyFrameNum(-1),
      m_totalFramesInVideoHint(0),
      m_isTrackingRunning(false),
      m_cancelRequested(false),
      m_isVideoSaving(false),
      m_videoProcessorsFinishedCount(0),
      m_totalVideoChunksToProcess(0),
      m_videoFps(0.0),
      m_expectedTrackersToFinish(0),
      m_finishedTrackersCount(0),
      m_videoProcessingOverallProgress(0),
      m_nextPhysicalBlobId(1), // Start IDs from 1
      m_storage(nullptr)
{
    registerMetaTypes();
}

TrackingManager::TrackingManager(TrackingDataStorage* storage, QObject* parent)
    : QObject(parent),
    m_keyFrameNum(-1),
    m_totalFramesInVideoHint(0),
    m_isTrackingRunning(false),
    m_cancelRequested(false),
    m_isVideoSaving(false),
    m_videoProcessorsFinishedCount(0),
    m_totalVideoChunksToProcess(0),
    m_videoFps(0.0),
    m_expectedTrackersToFinish(0),
    m_finishedTrackersCount(0),
    m_videoProcessingOverallProgress(0),
    m_nextPhysicalBlobId(1), // Start IDs from 1
    m_storage(storage)
{
    registerMetaTypes();
}

void TrackingManager::registerMetaTypes()
{

    qRegisterMetaType<QMap<int, std::vector<Tracking::WormTrackPoint>>>("QMap<int, std::vector<Tracking::WormTrackPoint>>");
    qRegisterMetaType<std::vector<cv::Mat>>("std::vector<cv::Mat>");
    qRegisterMetaType<cv::Size>("cv::Size");
    qRegisterMetaType<Tracking::AllWormTracks>("Tracking::AllWormTracks");
    qRegisterMetaType<QList<Tracking::DetectedBlob>>("QList<Tracking::DetectedBlob>");
    qRegisterMetaType<Tracking::DetectedBlob>("Tracking::DetectedBlob");
    qRegisterMetaType<Tracking::TrackerState>("Tracking::TrackerState");
    TRACKING_DEBUG() << "TrackingManager (" << this << ") created with frame-atomic logic. Timer eliminated for direct resolution.";
}

// Destructor
TrackingManager::~TrackingManager() {
    TRACKING_DEBUG() << "TrackingManager (" << this << ") DESTRUCTOR - START cleaning up...";

    // Ensure tracking is cancelled and resources are released
    if (m_isTrackingRunning) {
        TRACKING_DEBUG() << "TrackingManager Destructor: Tracking was running, requesting cancel.";
        cancelTracking();
        QThread::msleep(300);
    }

    // Force a final cleanup to ensure all resources are released
    cleanupThreadsAndObjects();
    TRACKING_DEBUG() << "TrackingManager (" << this << ") DESTRUCTOR - FINISHED cleaning up.";
}

// Main entry point for tracking
void TrackingManager::startFullTrackingProcess(
    const QString& videoPath, const QString& dataDirectory, int keyFrameNum,
    const std::vector<Tracking::InitialWormInfo>& initialWorms,
    const Thresholding::ThresholdSettings& settings, int totalFramesInVideoHint) {
    TRACKING_DEBUG() << "TrackingManager (" << this << "): startFullTrackingProcess called.";
    if (m_isTrackingRunning) {
        qWarning() << "TrackingManager: Attempted to start tracking while already running.";
        emit trackingFailed("Another tracking process is already running.");
        return;
    }

    cleanupThreadsAndObjects(); // Clean up from any previous run

    m_videoPath = videoPath;
    m_dataDirectory = dataDirectory;
    m_keyFrameNum = keyFrameNum;
    m_initialWormInfos = initialWorms;
    m_thresholdSettings = settings;
    m_totalFramesInVideoHint = totalFramesInVideoHint;

    // Create video-specific directory and save JSON files
    m_videoSpecificDirectory = createVideoSpecificDirectory(dataDirectory, videoPath);
    if (!m_videoSpecificDirectory.isEmpty()) {
        saveThresholdSettings(m_videoSpecificDirectory, settings);
        saveInputBlobs(m_videoSpecificDirectory, initialWorms);

        // Check if threshold settings differ from existing ones
        QString thresholdFilePath = QDir(m_videoSpecificDirectory).absoluteFilePath("thresh_settings.json");
        if (QFile::exists(thresholdFilePath)) {
            bool settingsMatch = compareThresholdSettings(thresholdFilePath, settings);
            if (!settingsMatch) {
                TRACKING_DEBUG() << "Current threshold settings differ from stored settings in" << thresholdFilePath;
                emit trackingStatusUpdate("Threshold settings differ from previous run");
            } else {
                // Threshold/settings match — attempt to load previously saved frame-atomic state
                // This will populate m_nextPhysicalBlobId, m_frameMergeRecords, m_splitResolutionMap and m_wormToPhysicalBlobIdMap
                bool loaded = loadFrameAtomicStateFromJson(m_videoSpecificDirectory,
                                                          m_nextPhysicalBlobId,
                                                          m_frameMergeRecords,
                                                          m_splitResolutionMap,
                                                          m_wormToPhysicalBlobIdMap,
                                                          m_thresholdSettings);
                if (loaded) {
                    TRACKING_DEBUG() << "TrackingManager: Loaded saved frame-atomic state from"
                                     << QDir(m_videoSpecificDirectory).absoluteFilePath("frame_atomic_state.json");
                    emit trackingStatusUpdate("Loaded previous merge/split state (retracking enabled)");
                } else {
                    TRACKING_DEBUG() << "TrackingManager: No valid saved frame-atomic state found (or load failed).";
                }
            }
        }
    }
    m_isTrackingRunning = true;
    m_cancelRequested = false;
    m_videoProcessingOverallProgress = 0;
    m_finishedTrackersCount = 0;
    m_expectedTrackersToFinish = 0;
    m_nextPhysicalBlobId = 1; // Reset for new tracking session

    // Clear data structures
    m_finalTracks.clear();
    m_individualTrackerProgress.clear();
    m_frameMergeRecords.clear();
    m_splitResolutionMap.clear();
    m_wormToPhysicalBlobIdMap.clear();

    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();
    for (const auto& info : m_initialWormInfos) {
        if (!m_wormObjectsMap.contains(info.id)) {
            m_wormObjectsMap[info.id] = new WormObject(info.id, info.initialRoi);
        }
        m_wormToPhysicalBlobIdMap[info.id] = -1; // Initially not part of any physical blob
    }

    m_assembledForwardFrameChunks.clear();
    m_assembledBackwardFrameChunks.clear();
    m_videoChunkProgressMap.clear();
    m_videoProcessorsFinishedCount = 0;
    m_totalVideoChunksToProcess = 0;
    m_finalProcessedForwardFrames.clear();
    m_finalProcessedReversedFrames.clear();

    emit trackingStatusUpdate("Initializing video processing...");
    emit overallTrackingProgress(0);

    // --- Determine actual total frames, FPS, and frame size ONCE ---
    // (This logic remains the same as your version with parallel video processing)
    cv::VideoCapture preliminaryCap;
    int actualTotalFrames = 0;
    try {
        if (!preliminaryCap.open(m_videoPath.toStdString())) {
            emit trackingFailed("Failed to open video file for preliminary checks: " + m_videoPath);
            m_isTrackingRunning = false; return;
        }
        actualTotalFrames = static_cast<int>(preliminaryCap.get(cv::CAP_PROP_FRAME_COUNT));
        m_videoFps = preliminaryCap.get(cv::CAP_PROP_FPS);
        m_videoFrameSize = cv::Size(
            static_cast<int>(preliminaryCap.get(cv::CAP_PROP_FRAME_WIDTH)),
            static_cast<int>(preliminaryCap.get(cv::CAP_PROP_FRAME_HEIGHT))
            );
        preliminaryCap.release();
    } catch (const cv::Exception& ex) {
        preliminaryCap.release();
        emit trackingFailed("OpenCV exception during preliminary video checks: " + QString(ex.what()));
        m_isTrackingRunning = false; return;
    }
    if (actualTotalFrames <= 0 && m_totalFramesInVideoHint > 0) actualTotalFrames = m_totalFramesInVideoHint;
    else if (actualTotalFrames <= 0) { emit trackingFailed("Could not determine total frames."); m_isTrackingRunning = false; return; }
    if (m_videoFps <= 0) m_videoFps = 25.0;
    if (m_videoFrameSize.width <= 0 || m_videoFrameSize.height <= 0) { emit trackingFailed("Could not determine frame size."); m_isTrackingRunning = false; return; }
    if (m_keyFrameNum < 0 || m_keyFrameNum >= actualTotalFrames) { emit trackingFailed("Keyframe out of bounds."); m_isTrackingRunning = false; return; }

    // --- Parallel Video Processing Logic ---
    // (This logic remains the same as your version with parallel video processing)
    int numThreads = QThread::idealThreadCount();
    numThreads = qMax(1, qMin(numThreads, 8));
    emit trackingStatusUpdate(QString("Processing video in chunks across %1 threads...").arg(numThreads));
    QList<QPair<int, int>> forwardFrameRanges, backwardFrameRanges;
    int forwardSegmentStart = m_keyFrameNum, forwardSegmentEnd = actualTotalFrames;
    int totalForwardSegmentFrames = forwardSegmentEnd - forwardSegmentStart;
    if (totalForwardSegmentFrames > 0) {
        int framesPerThread = std::max(1, static_cast<int>(std::ceil(static_cast<double>(totalForwardSegmentFrames) / numThreads)));
        for (int cs = forwardSegmentStart; cs < forwardSegmentEnd; cs += framesPerThread) {
            forwardFrameRanges.append({cs, std::min(cs + framesPerThread, forwardSegmentEnd)});
        }
    }
    int backwardSegmentStart = 0, backwardSegmentEnd = m_keyFrameNum;
    int totalBackwardSegmentFrames = backwardSegmentEnd - backwardSegmentStart;
    if (totalBackwardSegmentFrames > 0) {
        int framesPerThread = std::max(1, static_cast<int>(std::ceil(static_cast<double>(totalBackwardSegmentFrames) / numThreads)));
        for (int cs = backwardSegmentStart; cs < backwardSegmentEnd; cs += framesPerThread) {
            backwardFrameRanges.append({cs, std::min(cs + framesPerThread, backwardSegmentEnd)});
        }
    }
    m_totalVideoChunksToProcess = forwardFrameRanges.size() + backwardFrameRanges.size();
    if (m_totalVideoChunksToProcess == 0) {
        handleInitialProcessingComplete({}, {}, m_videoFps, m_videoFrameSize); return;
    }
    auto launch_chunk_processor = [this](int sF, int eF, bool isFwd) { /* ... same as your version ... */
                                                                       int chunkId = sF; m_videoChunkProgressMap[chunkId] = 0;
                                                                       VideoProcessor* proc = new VideoProcessor(); QThread* thr = new QThread(); proc->moveToThread(thr);
                                                                       m_videoProcessorThreads.append(thr);
                                                                       connect(thr, &QThread::started, proc, [=](){ proc->processFrameRange(m_videoPath, m_thresholdSettings, sF, eF, chunkId, isFwd); });
                                                                       connect(proc, &VideoProcessor::rangeProcessingComplete, this, &TrackingManager::handleRangeProcessingComplete);
                                                                       connect(proc, &VideoProcessor::processingError, this, &TrackingManager::handleVideoChunkProcessingError);
                                                                       connect(proc, &VideoProcessor::rangeProcessingProgress, this, &TrackingManager::handleRangeProcessingProgress);
                                                                       connect(thr, &QThread::finished, proc, &QObject::deleteLater);
                                                                       connect(thr, &QThread::finished, this, [this, tPtr = QPointer<QThread>(thr)](){ if(tPtr){/*qDebug() << "VidProcThr fin"*/;}});
                                                                       thr->start();
    };
    for (const auto& r : forwardFrameRanges) launch_chunk_processor(r.first, r.second, true);
    for (const auto& r : backwardFrameRanges) launch_chunk_processor(r.first, r.second, false);

    // Timer-based pause resolution has been eliminated in favor of direct resolution
    TRACKING_DEBUG() << "TM: Using direct split resolution instead of timer-based approach";
}

void TrackingManager::cancelTracking() {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested && !m_isTrackingRunning) return;
    if (m_cancelRequested) return;
    m_cancelRequested = true;
    bool wasRunning = m_isTrackingRunning;

    // Clear the split resolution map
    m_splitResolutionMap.clear();

    locker.unlock();
    emit trackingStatusUpdate("Cancellation requested...");
    for (QPointer<QThread> thread : m_videoProcessorThreads) { if (thread && thread->isRunning()) thread->requestInterruption(); }
    QList<WormTracker*> trackersToStop;
    locker.relock(); trackersToStop = m_wormTrackersList; locker.unlock();
    for (WormTracker* tracker : trackersToStop) { if (tracker) QMetaObject::invokeMethod(tracker, "stopTracking", Qt::QueuedConnection); }
    locker.relock();
    if ((wasRunning && m_expectedTrackersToFinish == 0 && m_wormTrackersList.isEmpty()) || !wasRunning) {
        m_isTrackingRunning = false; locker.unlock(); emit trackingCancelled();
    }
}

void TrackingManager::cleanupThreadsAndObjects() {
    TRACKING_DEBUG() << "TrackingManager: Starting aggressive memory cleanup...";

    // Calculate memory usage before cleanup for reporting
    size_t memoryBefore = getProcessedVideoMemoryUsage();
    size_t tracksMemoryBefore = m_finalTracks.size() * sizeof(Tracking::WormTrackPoint) * 100; // Rough estimate

    // (Largely same as your version, ensuring QPointer safety and clearing new maps)
    QList<QPointer<QThread>> videoThreadsToClean = m_videoProcessorThreads;
    m_videoProcessorThreads.clear();
    for (QPointer<QThread> thread : videoThreadsToClean) { /* ... quit, wait, delete ... */
        if (thread) { if (thread->isRunning()) { thread->requestInterruption(); thread->quit(); if (!thread->wait(1500)) { thread->terminate(); thread->wait();}} delete thread;}
    }
    QList<QPointer<QThread>> trackerThreadsToClean = m_trackerThreads;
    m_trackerThreads.clear();
    for (QPointer<QThread> thread : trackerThreadsToClean) { /* ... quit, wait, delete ... */
        if (thread) { if (thread->isRunning()) { thread->requestInterruption(); thread->quit(); if (!thread->wait(1000)) { thread->terminate(); thread->wait();}} delete thread;}
    }

    // Clean up video saver thread
    if (m_videoSaverThread) {
        if (m_videoSaverThread->isRunning()) {
            m_videoSaverThread->requestInterruption();
            m_videoSaverThread->quit();
            if (!m_videoSaverThread->wait(1000)) {
                m_videoSaverThread->terminate();
                m_videoSaverThread->wait();
            }
        }
        delete m_videoSaverThread;
        m_videoSaverThread = nullptr;
    }

    // Aggressively clear worm trackers and objects
    m_wormTrackersList.clear();
    m_wormIdToForwardTrackerInstanceMap.clear();
    m_wormIdToBackwardTrackerInstanceMap.clear();

    // Clear WormObject map with explicit deletion and memory hints
    qDeleteAll(m_wormObjectsMap);
    m_wormObjectsMap.clear();
    QMap<int, WormObject*>().swap(m_wormObjectsMap); // Force deallocation

    // Aggressively clear processed video memory
    clearProcessedVideoMemory();

    // Clear and shrink all chunk processing maps
    m_assembledForwardFrameChunks.clear();
    m_assembledBackwardFrameChunks.clear();
    QMap<int, std::vector<cv::Mat>>().swap(m_assembledForwardFrameChunks);
    QMap<int, std::vector<cv::Mat>>().swap(m_assembledBackwardFrameChunks);

    m_videoChunkProgressMap.clear();
    QMap<int, int>().swap(m_videoChunkProgressMap);

    // Aggressively clear final tracks with memory hints
    m_finalTracks.clear();
    Tracking::AllWormTracks().swap(m_finalTracks);

    m_individualTrackerProgress.clear();
    QMap<WormTracker*, int>().swap(m_individualTrackerProgress);

    // Clear all data structures related to split resolution with forced deallocation
    m_frameMergeRecords.clear();
    QMap<int, QList<FrameSpecificPhysicalBlob>>().swap(m_frameMergeRecords);

    m_splitResolutionMap.clear();
    QMap<int, QMap<int, Tracking::DetectedBlob>>().swap(m_splitResolutionMap);

    m_wormToPhysicalBlobIdMap.clear();
    QMap<int, int>().swap(m_wormToPhysicalBlobIdMap);

    // Reset state flags
    m_isTrackingRunning = false;
    m_cancelRequested = false;

    // Reset frame counters and IDs
    m_nextPhysicalBlobId = 1;

    // Calculate and report memory freed
    double memoryMB = (memoryBefore + tracksMemoryBefore) / (1024.0 * 1024.0);
    TRACKING_DEBUG() << "TrackingManager: Aggressive cleanup complete - freed approximately"
             << QString::number(memoryMB, 'f', 1) << "MB of tracking data";
    TRACKING_DEBUG() << "TrackingManager: All data structures cleared and memory aggressively freed";
}

// --- Video Processing Callbacks ---
void TrackingManager::handleRangeProcessingProgress(int chunkId, int percentage) {
    if (m_cancelRequested || !m_isTrackingRunning) return;
    QMutexLocker locker(&m_dataMutex);
    if (m_videoChunkProgressMap.contains(chunkId)) m_videoChunkProgressMap[chunkId] = percentage; else return;
    double totalProgressSum = 0; for (int p : m_videoChunkProgressMap.values()) totalProgressSum += p;
    m_videoProcessingOverallProgress = (m_totalVideoChunksToProcess > 0) ? static_cast<int>(totalProgressSum / m_totalVideoChunksToProcess) : 100;
    locker.unlock(); updateOverallProgress();
}
void TrackingManager::handleRangeProcessingComplete(int chunkId, const std::vector<cv::Mat>& processedFrames, bool wasForwardChunk) {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) return;
    if (wasForwardChunk) m_assembledForwardFrameChunks[chunkId] = processedFrames; else m_assembledBackwardFrameChunks[chunkId] = processedFrames;
    m_videoChunkProgressMap[chunkId] = 100; m_videoProcessorsFinishedCount++;
    if (m_videoProcessorsFinishedCount >= m_totalVideoChunksToProcess) {
        locker.unlock(); assembleProcessedFrames();
    } else {
        double totalProgressSum = 0; for (int p : m_videoChunkProgressMap.values()) totalProgressSum += p;
        m_videoProcessingOverallProgress = (m_totalVideoChunksToProcess > 0) ? static_cast<int>(totalProgressSum / m_totalVideoChunksToProcess) : 100;
        locker.unlock(); updateOverallProgress();
    }
}
void TrackingManager::assembleProcessedFrames() {
    QMutexLocker locker(&m_dataMutex); if (m_cancelRequested) { m_isTrackingRunning = false; locker.unlock(); emit trackingCancelled(); return; }
    m_finalProcessedForwardFrames.clear(); m_finalProcessedReversedFrames.clear();
    // Assemble forward chunks in correct order (sorted by chunk key = start frame)
    QList<int> forwardKeys = m_assembledForwardFrameChunks.keys();
    std::sort(forwardKeys.begin(), forwardKeys.end());
    for (int key : forwardKeys) {
        const auto& chunk = m_assembledForwardFrameChunks[key];
        m_finalProcessedForwardFrames.insert(m_finalProcessedForwardFrames.end(), chunk.begin(), chunk.end());
    }

    // Assemble backward chunks in correct order (sorted by chunk key = start frame)
    std::vector<cv::Mat> tempBackwardFrames;
    QList<int> backwardKeys = m_assembledBackwardFrameChunks.keys();
    std::sort(backwardKeys.begin(), backwardKeys.end());
    for (int key : backwardKeys) {
        const auto& chunk = m_assembledBackwardFrameChunks[key];
        tempBackwardFrames.insert(tempBackwardFrames.end(), chunk.begin(), chunk.end());
    }
    m_finalProcessedReversedFrames = tempBackwardFrames; std::reverse(m_finalProcessedReversedFrames.begin(), m_finalProcessedReversedFrames.end());
    m_assembledForwardFrameChunks.clear(); m_assembledBackwardFrameChunks.clear(); m_videoChunkProgressMap.clear();
    locker.unlock();
    handleInitialProcessingComplete(m_finalProcessedForwardFrames, m_finalProcessedReversedFrames, m_videoFps, m_videoFrameSize);
}
void TrackingManager::handleInitialProcessingComplete(const std::vector<cv::Mat>&, const std::vector<cv::Mat>&, double, cv::Size) {
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested) { m_isTrackingRunning = false; locker.unlock(); emit trackingCancelled(); return; }
    if (!m_isTrackingRunning) return;
    m_videoProcessingOverallProgress = 100;
    locker.unlock();
    emit trackingStatusUpdate("Video processing finished. Preparing trackers...");
    updateOverallProgress();
    launchWormTrackers();
}
void TrackingManager::handleVideoChunkProcessingError(int chunkId, const QString& errorMessage) {
    QMutexLocker locker(&m_dataMutex); if (m_cancelRequested || !m_isTrackingRunning) return;
    m_cancelRequested = true; m_isTrackingRunning = false;
    locker.unlock();
    for (QPointer<QThread> thread : m_videoProcessorThreads) { if (thread && thread->isRunning()) thread->requestInterruption(); }
    emit trackingFailed("Video processing failed (chunk " + QString::number(chunkId) + "): " + errorMessage);
}

// --- Helper function for signed worm IDs ---
int TrackingManager::getSignedWormId(int conceptualWormId, WormTracker::TrackingDirection direction) {
    return (direction == WormTracker::TrackingDirection::Forward) ? conceptualWormId : -conceptualWormId;
}

int TrackingManager::getUnsignedWormId(int signedWormId) {
    return qAbs(signedWormId);
}

WormTracker::TrackingDirection TrackingManager::getDirectionFromSignedId(int signedWormId) {
    return (signedWormId >= 0) ? WormTracker::TrackingDirection::Forward : WormTracker::TrackingDirection::Backward;
}

// --- Core Frame Update Logic ---
void TrackingManager::handleFrameUpdate(int reportingConceptualWormId,
                                           int originalFrameNumber,
                                           const Tracking::DetectedBlob& primaryBlob, // Anchor blob for track history
                                           const Tracking::DetectedBlob& fullBlob,    // Full blob for merge/state processing
                                           QRectF searchRoiUsed,
                                           Tracking::TrackerState currentState,
                                           const QList<Tracking::DetectedBlob>& splitCandidates)
{
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning) return;

    WormTracker* reportingTrackerInstance = qobject_cast<WormTracker*>(sender());

    // Get direction and calculate signed worm ID for pause handling
    WormTracker::TrackingDirection direction = reportingTrackerInstance ? reportingTrackerInstance->getDirection() : WormTracker::TrackingDirection::Forward;
    int signedWormId = getSignedWormId(reportingConceptualWormId, direction);

    QString dmsg = QString("TM: WT %1 FN%2 | ").arg(signedWormId).arg(originalFrameNumber);
    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** PROCESSING FRAME UPDATE *** State:").arg(signedWormId).arg(originalFrameNumber) << static_cast<int>(currentState) << "FullBlobValid:" << fullBlob.isValid << "SplitCandidates:" << splitCandidates.size();
    // Note: WormObject update uses primaryBlob
    WormObject* wormObject = m_wormObjectsMap.value(reportingConceptualWormId, nullptr);
    if (wormObject) {
        Tracking::WormTrackPoint point;
        point.frameNumberOriginal = originalFrameNumber;
        point.roi = searchRoiUsed;

        if (primaryBlob.isValid) {
            point.position = cv::Point2f(static_cast<float>(primaryBlob.centroid.x()), static_cast<float>(primaryBlob.centroid.y()));
            point.quality = (currentState == Tracking::TrackerState::TrackingSingle) ? Tracking::TrackPointQuality::Confident : Tracking::TrackPointQuality::Ambiguous;
        } else {
            point.position = cv::Point2f(0.0f, 0.0f);  // Placeholder (won't be used for display)
            point.quality = Tracking::TrackPointQuality::Lost;
        }

        wormObject->updateTrackPoint(point);
    }

    //QString dmsg = QString("TM: WT %1 FN%2 | ").arg(reportingConceptualWormId).arg(originalFrameNumber);
    // qDebug().noquote() << dmsg << "State" << static_cast<int>(currentState) << "FullBlobValid:" << fullBlob.isValid;

    if (currentState == Tracking::TrackerState::TrackingSingle || currentState == Tracking::TrackerState::TrackingLost) {
        m_wormToPhysicalBlobIdMap[signedWormId] = -1; // No longer part of a specific physical blob
    } else if (currentState == Tracking::TrackerState::TrackingMerged) {
        if (fullBlob.isValid) {
            processFrameSpecificMerge(signedWormId, originalFrameNumber, fullBlob, reportingTrackerInstance);
        } else {
            // qDebug().noquote() << dmsg << "State Merged but fullBlob invalid. Treating as lost for merge logic.";
            m_wormToPhysicalBlobIdMap[signedWormId] = -1;
        }
    } else if (currentState == Tracking::TrackerState::PausedForSplit) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** ENTERING SPLIT PROCESSING ***").arg(signedWormId).arg(originalFrameNumber);
        if (!splitCandidates.isEmpty() && reportingTrackerInstance) {
            // Find the chosen candidate (should be the primaryBlob if valid, otherwise first candidate)
            Tracking::DetectedBlob chosenCandidate = primaryBlob.isValid ? primaryBlob :
                                                    (!splitCandidates.isEmpty() ? splitCandidates.first() : Tracking::DetectedBlob());
            TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Chosen candidate Area:").arg(signedWormId).arg(originalFrameNumber) << chosenCandidate.area << "Pos:" << chosenCandidate.centroid.x() << "," << chosenCandidate.centroid.y();
            processFrameSpecificSplit(signedWormId, originalFrameNumber, splitCandidates, chosenCandidate, reportingTrackerInstance);
        } else {
             TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|State PausedForSplit but no candidates/instance. Forcing lost.").arg(signedWormId).arg(originalFrameNumber);
            m_wormToPhysicalBlobIdMap[signedWormId] = -1;
            if(reportingTrackerInstance) QMetaObject::invokeMethod(reportingTrackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        }
    }

    // All split resolution is now immediate without any paused state
}


void TrackingManager::processFrameSpecificMerge(int signedWormId, int frameNumber,
                                                const Tracking::DetectedBlob& reportedFullBlob,
                                                WormTracker* reportingTrackerInstance)
{
    int conceptualWormId = getUnsignedWormId(signedWormId);
    QString dmsg = QString("TM: WT %1 FN%2 | procFrameSpecMerge | ").arg(signedWormId).arg(frameNumber);
    // qDebug().noquote() << dmsg << "Blob @ " << reportedFullBlob.centroid.x() << "," << reportedFullBlob.centroid.y() << " Area: " << reportedFullBlob.area;

    QList<FrameSpecificPhysicalBlob>& blobsOnThisFrame = m_frameMergeRecords[frameNumber];
    FrameSpecificPhysicalBlob* matchedPhysicalBlob = nullptr;

    for (FrameSpecificPhysicalBlob& existingBlobRecord : blobsOnThisFrame) {
        double iou = calculateIoU(reportedFullBlob.boundingBox, existingBlobRecord.currentBoundingBox);
        bool isContained = existingBlobRecord.currentBoundingBox.contains(reportedFullBlob.centroid);
        //cv::Point2f reportedCentroidCv(static_cast<float>(reportedFullBlob.centroid.x()), static_cast<float>(reportedFullBlob.centroid.y()));
        //double distSq = Tracking::sqDistance(reportedCentroidCv, existingBlobRecord.currentCentroid);

        if (iou > PHYSICAL_BLOB_IOU_THRESHOLD || (isContained && iou > 0.01) /*|| distSq < PHYSICAL_BLOB_CENTROID_MAX_DIST_SQ*/) {
            matchedPhysicalBlob = &existingBlobRecord;
            // qDebug().noquote() << dmsg << "Matched existing PhysicalBlobID:" << matchedPhysicalBlob->uniqueId << "IoU:" << iou << "Contained:" << isContained;
            break;
        }
    }

    if (matchedPhysicalBlob) {
        matchedPhysicalBlob->participatingWormTrackerIDs.insert(signedWormId);
        // Update the physical blob's representation with this new information
        matchedPhysicalBlob->currentBoundingBox = matchedPhysicalBlob->currentBoundingBox.united(reportedFullBlob.boundingBox);
        matchedPhysicalBlob->currentArea = qMax(matchedPhysicalBlob->currentArea, reportedFullBlob.area); // ideally should not change
        if(matchedPhysicalBlob->currentBoundingBox.isValid()){
            QPointF newCenter = matchedPhysicalBlob->currentBoundingBox.center();
            matchedPhysicalBlob->currentCentroid = cv::Point2f(static_cast<float>(newCenter.x()), static_cast<float>(newCenter.y()));
        }
        m_wormToPhysicalBlobIdMap[signedWormId] = matchedPhysicalBlob->uniqueId;
        // qDebug().noquote() << dmsg << "Added to existing PhysicalBlobID:" << matchedPhysicalBlob->uniqueId << ". Participants:" << matchedPhysicalBlob->participatingWormTrackerIDs;
    } else {
        FrameSpecificPhysicalBlob newPhysicalBlob;
        newPhysicalBlob.uniqueId = m_nextPhysicalBlobId++;
        newPhysicalBlob.frameNumber = frameNumber;
        newPhysicalBlob.currentBoundingBox = reportedFullBlob.boundingBox;
        if(newPhysicalBlob.currentBoundingBox.isValid()){
            QPointF center = newPhysicalBlob.currentBoundingBox.center();
            newPhysicalBlob.currentCentroid = cv::Point2f(static_cast<float>(center.x()), static_cast<float>(center.y()));
        } else {
            newPhysicalBlob.currentCentroid = cv::Point2f(static_cast<float>(reportedFullBlob.centroid.x()), static_cast<float>(reportedFullBlob.centroid.y()));
        }
        newPhysicalBlob.currentArea = reportedFullBlob.area;
        newPhysicalBlob.participatingWormTrackerIDs.insert(signedWormId);
        // newPhysicalBlob.timeFirstReported = QDateTime::currentDateTime();

        blobsOnThisFrame.append(newPhysicalBlob);
        m_wormToPhysicalBlobIdMap[signedWormId] = newPhysicalBlob.uniqueId;
        // qDebug().noquote() << dmsg << "Created new PhysicalBlobID:" << newPhysicalBlob.uniqueId << ". Participants:" << newPhysicalBlob.participatingWormTrackerIDs;
    }

    // Merge state is fully handled with the frame-specific physical blob representation
}

void TrackingManager::processFrameSpecificSplit(int signedWormId, int frameNumber,
                                               const QList<Tracking::DetectedBlob>& allSplitCandidates,
                                               const Tracking::DetectedBlob& chosenCandidate,
                                               WormTracker* reportingTrackerInstance)
{
    int conceptualWormId = getUnsignedWormId(signedWormId);
    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|procFrameSpecSplit - Candidates:").arg(signedWormId).arg(frameNumber) << allSplitCandidates.size() << "Chosen @" << chosenCandidate.centroid.x() << "," << chosenCandidate.centroid.y() << "Area:" << chosenCandidate.area;

    // Get reference to current frame's physical blobs
    QList<FrameSpecificPhysicalBlob>& blobsOnThisFrame = m_frameMergeRecords[frameNumber];

    // Step 1: Create PhysicalBlobIds for all split candidates if they don't exist
    QList<int> candidatePhysicalBlobIds;
    int chosenCandidatePhysicalBlobId = -1; // Track which physical blob ID corresponds to the chosen candidate

    for (const Tracking::DetectedBlob& candidate : allSplitCandidates) {
        if (!candidate.isValid) continue;

        // Check if this candidate matches an existing physical blob
        FrameSpecificPhysicalBlob* existingBlob = nullptr;
        for (FrameSpecificPhysicalBlob& blob : blobsOnThisFrame) {
            double iou = calculateIoU(candidate.boundingBox, blob.currentBoundingBox);
            bool isContained = blob.currentBoundingBox.contains(candidate.centroid);

            if (iou > PHYSICAL_BLOB_IOU_THRESHOLD || (isContained && iou > 0.01)) {
                existingBlob = &blob;
                break;
            }
        }

        int currentBlobId = -1;
        if (existingBlob) {
            // Add this WT to the existing blob's participants
            existingBlob->participatingWormTrackerIDs.insert(signedWormId);
            currentBlobId = existingBlob->uniqueId;
            candidatePhysicalBlobIds.append(currentBlobId);
            TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Matched existing PhysicalBlobID:").arg(signedWormId).arg(frameNumber) << currentBlobId;
        } else {
            // Create new physical blob for this candidate
            FrameSpecificPhysicalBlob newPhysicalBlob;
            newPhysicalBlob.uniqueId = m_nextPhysicalBlobId++;
            newPhysicalBlob.frameNumber = frameNumber;
            newPhysicalBlob.currentBoundingBox = candidate.boundingBox;
            newPhysicalBlob.currentCentroid = cv::Point2f(static_cast<float>(candidate.centroid.x()), static_cast<float>(candidate.centroid.y()));
            newPhysicalBlob.currentArea = candidate.area;
            newPhysicalBlob.participatingWormTrackerIDs.insert(signedWormId);
            newPhysicalBlob.selectedByWormTrackerId = 0; // Initially unselected

            blobsOnThisFrame.append(newPhysicalBlob);
            currentBlobId = newPhysicalBlob.uniqueId;
            candidatePhysicalBlobIds.append(currentBlobId);
            TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Created new PhysicalBlobID:").arg(signedWormId).arg(frameNumber) << currentBlobId << "for split candidate";
        }

        // Check if this is the chosen candidate (by comparing centroids)
        if (qFuzzyCompare(candidate.centroid.x(), chosenCandidate.centroid.x()) &&
            qFuzzyCompare(candidate.centroid.y(), chosenCandidate.centroid.y())) {
            chosenCandidatePhysicalBlobId = currentBlobId;
            TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Identified chosen candidate as PhysicalBlobID:").arg(signedWormId).arg(frameNumber) << currentBlobId;
        }
    }

    // Set the preferred blob for this worm to be the chosen candidate
    if (chosenCandidatePhysicalBlobId != -1) {
        m_wormToPhysicalBlobIdMap[conceptualWormId] = chosenCandidatePhysicalBlobId;
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** SETTING PREFERRED BLOB *** Worm").arg(signedWormId).arg(frameNumber) << conceptualWormId
                          << "prefers PhysicalBlobID:" << chosenCandidatePhysicalBlobId;
    } else {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** WARNING: Could not identify chosen candidate in physical blobs ***").arg(signedWormId).arg(frameNumber);
    }

    // Try immediate resolution using our new method
    if (attemptImmediateSplitResolution(signedWormId, frameNumber, allSplitCandidates, chosenCandidate, reportingTrackerInstance)) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Split successfully resolved immediately").arg(signedWormId).arg(frameNumber);
    } else {
        // Fallback - assign the first valid blob or go lost
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Immediate resolution failed, using fallback (go lost)").arg(signedWormId).arg(frameNumber);
        QMetaObject::invokeMethod(reportingTrackerInstance, "resumeTrackingWithAssignedTarget",
                                 Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        m_splitResolutionMap[frameNumber][signedWormId] = Tracking::DetectedBlob();
    }
}


bool TrackingManager::attemptImmediateSplitResolution(int conceptualWormId, int frameNumber,
                                                    const QList<Tracking::DetectedBlob>& allCandidates,
                                                    const Tracking::DetectedBlob& chosenCandidate,
                                                    WormTracker* trackerInstance) {
    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|=== STARTING SPLIT RESOLUTION ===").arg(conceptualWormId).arg(frameNumber);
    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Chosen candidate Area:").arg(conceptualWormId).arg(frameNumber) << chosenCandidate.area
                      << "Pos:" << chosenCandidate.centroid.x() << "," << chosenCandidate.centroid.y();

    if (!trackerInstance) { qWarning() << QString("TM: %1|FN%2|No tracker instance.").arg(conceptualWormId).arg(frameNumber); return false; }
    if (!chosenCandidate.isValid) { TRACKING_DEBUG() << QString("TM: %1|FN%2|Invalid candidate. Forcing lost.").arg(conceptualWormId).arg(frameNumber); QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob())); return true; }

    // Find physical blob IDs for this worm
    QList<int> thisWormPhysicalBlobIds;
    if (m_frameMergeRecords.contains(frameNumber)) {
        for (const FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
            if (blob.participatingWormTrackerIDs.contains(conceptualWormId)) {
                thisWormPhysicalBlobIds.append(blob.uniqueId);
            }
        }
    }

    if (thisWormPhysicalBlobIds.isEmpty()) {
        TRACKING_DEBUG() << QString("TM: %1|FN%2|No PhysicalBlobIds found for this worm. Cannot proceed with resolution.").arg(conceptualWormId).arg(frameNumber);
        QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget", Qt::QueuedConnection,
                                 Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        m_splitResolutionMap[frameNumber][conceptualWormId] = Tracking::DetectedBlob();
        return true;
    }

    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|This worm's PhysicalBlobIds:").arg(conceptualWormId).arg(frameNumber) << thisWormPhysicalBlobIds;

    // Show current blob assignments for debugging
    for (const FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|PhysicalBlob").arg(conceptualWormId).arg(frameNumber) << blob.uniqueId
                          << "selectedBy:" << blob.selectedByWormTrackerId
                          << "participants:" << blob.participatingWormTrackerIDs;
    }

    // Check if a preferred blob is already assigned to this worm
    int preferredBlobId = m_wormToPhysicalBlobIdMap.value(conceptualWormId, -1);
    Tracking::DetectedBlob blobToAssign;
    blobToAssign.isValid = false;
    bool resolutionSuccess = false;

    // Look for already resolved worms on this frame that might have taken blobs we're interested in
    QMap<int, QList<int>> otherWormBlobAssignments; // wormId -> list of blobIds

    if (m_splitResolutionMap.contains(frameNumber)) {
        QMap<int, Tracking::DetectedBlob> resolvedBlobs = m_splitResolutionMap[frameNumber];
        for (auto it = resolvedBlobs.begin(); it != resolvedBlobs.end(); ++it) {
            int otherWormId = it.key();
            if (otherWormId == conceptualWormId) continue;

            QList<int> otherWormBlobIds;
            for (const FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.selectedByWormTrackerId == otherWormId) {
                    otherWormBlobIds.append(blob.uniqueId);
                }
            }

            if (!otherWormBlobIds.isEmpty()) {
                otherWormBlobAssignments[otherWormId] = otherWormBlobIds;
                TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Worm").arg(conceptualWormId).arg(frameNumber) << otherWormId << "already has blobs assigned:" << otherWormBlobIds;
            }
        }
    }

    // First, try to find our preferred blob if it's available
    if (preferredBlobId != -1) {
        bool blobTaken = false;

        // Check if any other worm has already been assigned this blob
        for (auto it = otherWormBlobAssignments.begin(); it != otherWormBlobAssignments.end(); ++it) {
            if (it.value().contains(preferredBlobId)) {
                blobTaken = true;
                TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Preferred blob").arg(conceptualWormId).arg(frameNumber) << preferredBlobId << "already taken by worm" << it.key();
                break;
            }
        }

        if (!blobTaken) {
            // Try to find and claim this blob
            for (FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.uniqueId == preferredBlobId && blob.selectedByWormTrackerId == 0) {
                    // Claim the blob
                    blob.selectedByWormTrackerId = conceptualWormId;

                    // Create the blob to assign
                    blobToAssign.isValid = true;
                    blobToAssign.centroid = QPointF(blob.currentCentroid.x, blob.currentCentroid.y);
                    blobToAssign.boundingBox = blob.currentBoundingBox;
                    blobToAssign.area = blob.currentArea;

                    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** CLAIMED PREFERRED BLOB ***").arg(conceptualWormId).arg(frameNumber) << preferredBlobId
                                      << "Area:" << blobToAssign.area
                                      << "Pos:" << blobToAssign.centroid.x() << "," << blobToAssign.centroid.y();
                    resolutionSuccess = true;
                    break;
                }
            }
        }
    }

    // If we couldn't claim our preferred blob, look for any available alternative
    if (!blobToAssign.isValid) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** PREFERRED BLOB UNAVAILABLE - LOOKING FOR ALTERNATIVES ***").arg(conceptualWormId).arg(frameNumber);
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|PreferredBlobId was:").arg(conceptualWormId).arg(frameNumber) << preferredBlobId;

        // Get the worm tracker instance to find its last known position
        //WormTracker* tracker = nullptr;
        //for (WormTracker* trackerPtr : m_wormTrackersList) {
        //    if (trackerPtr && getSignedWormId(trackerPtr->getWormId(), trackerPtr->getDirection()) == conceptualWormId) {
        //        tracker = trackerPtr;
        //        break;
        //    }
        // }

        if (trackerInstance) {
            // Find all unassigned blobs that this worm is participating in
            struct BlobWithDistance {
                FrameSpecificPhysicalBlob* blob;
                double distance;
            };
            QList<BlobWithDistance> availableBlobs;

            // Get the last known position
            QPointF lastKnownPos;
            if (chosenCandidate.isValid) {
                // Use the chosen candidate as reference if available
                lastKnownPos = chosenCandidate.centroid;
            } else {
                // Try to use the tracker's last known position
                cv::Point2f trackerPos = trackerInstance->getLastKnownPosition();
                lastKnownPos = QPointF(trackerPos.x, trackerPos.y);
            }

            // Find all available blobs and calculate distances
            for (FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.participatingWormTrackerIDs.contains(conceptualWormId) && blob.selectedByWormTrackerId == 0) {
                    QPointF blobPos(blob.currentCentroid.x, blob.currentCentroid.y);
                    double dx = blobPos.x() - lastKnownPos.x();
                    double dy = blobPos.y() - lastKnownPos.y();
                    double distSq = dx*dx + dy*dy;

                    BlobWithDistance bwd;
                    bwd.blob = &blob;
                    bwd.distance = distSq;
                    availableBlobs.append(bwd);
                }
            }

            // Sort blobs by distance (closest first)
            std::sort(availableBlobs.begin(), availableBlobs.end(),
                     [](const BlobWithDistance& a, const BlobWithDistance& b) {
                         return a.distance < b.distance;
                     });

            // Use the closest blob if available
            if (!availableBlobs.isEmpty()) {
                FrameSpecificPhysicalBlob* closestBlob = availableBlobs.first().blob;

                // Claim this blob
                closestBlob->selectedByWormTrackerId = conceptualWormId;

                // Update the mapping
                m_wormToPhysicalBlobIdMap[conceptualWormId] = closestBlob->uniqueId;

                // Create the blob to assign
                blobToAssign.isValid = true;
                blobToAssign.centroid = QPointF(closestBlob->currentCentroid.x, closestBlob->currentCentroid.y);
                blobToAssign.boundingBox = closestBlob->currentBoundingBox;
                blobToAssign.area = closestBlob->currentArea;

                TRACKING_DEBUG() << QString("TM: %1|FN%2|*** CLAIMED ALTERNATIVE BLOB ***").arg(conceptualWormId).arg(frameNumber) << closestBlob->uniqueId
                                  << "Area:" << blobToAssign.area
                                  << "Pos:" << blobToAssign.centroid.x() << "," << blobToAssign.centroid.y()
                                  << "Distance:" << QString::number(std::sqrt(availableBlobs.first().distance), 'f', 2);
                resolutionSuccess = true;
            } else {
                TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|No available alternative blobs found.").arg(conceptualWormId).arg(frameNumber);
            }
        } else {
            // Fallback to original method if we can't find the tracker
            TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|Couldn't find tracker, using original method.").arg(conceptualWormId).arg(frameNumber);

            // Find any unassigned blob that this worm is participating in
            for (FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
                if (blob.participatingWormTrackerIDs.contains(conceptualWormId) && blob.selectedByWormTrackerId == 0) {
                    // Claim this blob
                    blob.selectedByWormTrackerId = conceptualWormId;

                    // Update the mapping
                    m_wormToPhysicalBlobIdMap[conceptualWormId] = blob.uniqueId;

                    // Create the blob to assign
                    blobToAssign.isValid = true;
                    blobToAssign.centroid = QPointF(blob.currentCentroid.x, blob.currentCentroid.y);
                    blobToAssign.boundingBox = blob.currentBoundingBox;
                    blobToAssign.area = blob.currentArea;

                    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** CLAIMED FALLBACK BLOB ***").arg(conceptualWormId).arg(frameNumber) << blob.uniqueId
                                      << "Area:" << blobToAssign.area
                                      << "Pos:" << blobToAssign.centroid.x() << "," << blobToAssign.centroid.y();
                    resolutionSuccess = true;
                    break;
                }
            }
        }
    }

    // Assign blob or go lost
    if (blobToAssign.isValid) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** FINAL ASSIGNMENT *** Worm").arg(conceptualWormId).arg(frameNumber) << conceptualWormId
                          << "gets blob Area:" << blobToAssign.area
                          << "Pos:" << blobToAssign.centroid.x() << "," << blobToAssign.centroid.y();
        QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget",
                                 Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, blobToAssign));
        m_splitResolutionMap[frameNumber][conceptualWormId] = blobToAssign;
    } else {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|*** NO VALID BLOB - GOING LOST ***").arg(conceptualWormId).arg(frameNumber);
        QMetaObject::invokeMethod(trackerInstance, "resumeTrackingWithAssignedTarget",
                                 Qt::QueuedConnection, Q_ARG(Tracking::DetectedBlob, Tracking::DetectedBlob()));
        m_splitResolutionMap[frameNumber][conceptualWormId] = Tracking::DetectedBlob();
    }

    // Show final blob assignments after this worm's resolution
    TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|=== POST-RESOLUTION BLOB ASSIGNMENTS ===").arg(conceptualWormId).arg(frameNumber);
    for (const FrameSpecificPhysicalBlob& blob : m_frameMergeRecords[frameNumber]) {
        TRACKING_DEBUG().noquote() << QString("TM: %1|FN%2|PhysicalBlob").arg(conceptualWormId).arg(frameNumber) << blob.uniqueId
                          << "selectedBy:" << blob.selectedByWormTrackerId
                          << "Area:" << blob.currentArea;
    }

    return true;
}

// This function has been replaced by immediate resolution in processFrameSpecificSplit

// This function has been replaced by immediate resolution in processFrameSpecificSplit

// --- Helper and Utility functions (calculateIoU, launchWormTrackers, progress, finish handlers etc.) ---
// These remain largely the same as your version with parallel video processing.
// calculateIoU is used by processFrameSpecificMerge.
// launchWormTrackers, updateOverallProgress, checkForAllTrackersFinished, outputTracksToCsv,
// handleWormTrackerFinished, handleWormTrackerError, handleWormTrackerProgress are mostly independent
// of the merge group internal logic, dealing with tracker lifecycle and overall progress.

double TrackingManager::calculateIoU(const QRectF& r1, const QRectF& r2) const {
    QRectF intersection = r1.intersected(r2);
    double intersectionArea = intersection.width() * intersection.height();
    if (intersectionArea <= 0) return 0.0;
    double unionArea = (r1.width() * r1.height()) + (r2.width() * r2.height()) - intersectionArea;
    return unionArea > 0 ? (intersectionArea / unionArea) : 0.0;
}

void TrackingManager::launchWormTrackers() { /* ... same as your version ... */
    if (m_cancelRequested) { m_isTrackingRunning = false; emit trackingCancelled(); return; }
    if (m_initialWormInfos.empty()) { emit trackingStatusUpdate("No worms selected."); emit trackingFinishedSuccessfully(""); m_isTrackingRunning = false; return; }
    m_expectedTrackersToFinish = 0; m_finishedTrackersCount = 0; m_individualTrackerProgress.clear();
    m_wormIdToForwardTrackerInstanceMap.clear(); m_wormIdToBackwardTrackerInstanceMap.clear();
    for(QPointer<QThread> t : m_trackerThreads) { if(t) delete t; } m_trackerThreads.clear();

    for (const auto& info : m_initialWormInfos) {
        int wId = info.id; QRectF iRoi = info.initialRoi;
        if (!m_finalProcessedForwardFrames.empty() || m_keyFrameNum == (m_videoFrameSize.width > 0 ? static_cast<int>(m_videoFps * (m_totalFramesInVideoHint > 0 ? m_totalFramesInVideoHint : 1) / m_videoFps) -1 : 0) ) {
            WormTracker* trk = new WormTracker(wId, iRoi, WormTracker::TrackingDirection::Forward, m_keyFrameNum);
            trk->setFrames(&m_finalProcessedForwardFrames); QThread* thr = new QThread(); trk->moveToThread(thr);
            trk->setProperty("wormId", wId); trk->setProperty("direction", "Forward"); m_trackerThreads.append(thr);
            connect(thr, &QThread::started, trk, &WormTracker::startTracking);
            connect(trk, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(thr, &QThread::finished, trk, &QObject::deleteLater);
            connect(thr, &QThread::finished, this, [this, tPtr=QPointer<QThread>(thr), wId](){if(tPtr){/*qDebug()<<"FwdTrkThr fin" << wId;*/}});
            connect(trk, &WormTracker::positionUpdated, this, &TrackingManager::handleFrameUpdate);
            connect(trk, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(trk, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);
            m_wormTrackersList.append(trk); m_wormIdToForwardTrackerInstanceMap[wId] = trk; m_individualTrackerProgress[trk] = 0;
            thr->start(); m_expectedTrackersToFinish++;
        }
        if (!m_finalProcessedReversedFrames.empty() || m_keyFrameNum == 0) {
            WormTracker* trk = new WormTracker(wId, iRoi, WormTracker::TrackingDirection::Backward, m_keyFrameNum);
            trk->setFrames(&m_finalProcessedReversedFrames); QThread* thr = new QThread(); trk->moveToThread(thr);
            trk->setProperty("wormId", wId); trk->setProperty("direction", "Backward"); m_trackerThreads.append(thr);
            connect(thr, &QThread::started, trk, &WormTracker::startTracking);
            connect(trk, &WormTracker::finished, this, &TrackingManager::handleWormTrackerFinished);
            connect(thr, &QThread::finished, trk, &QObject::deleteLater);
            connect(thr, &QThread::finished, this, [this, tPtr=QPointer<QThread>(thr), wId](){if(tPtr){/*qDebug()<<"BwdTrkThr fin"<< wId;*/}});
            connect(trk, &WormTracker::positionUpdated, this, &TrackingManager::handleFrameUpdate);
            connect(trk, &WormTracker::errorOccurred, this, &TrackingManager::handleWormTrackerError);
            connect(trk, &WormTracker::progress, this, &TrackingManager::handleWormTrackerProgress);
            m_wormTrackersList.append(trk); m_wormIdToBackwardTrackerInstanceMap[wId] = trk; m_individualTrackerProgress[trk] = 0;
            thr->start(); m_expectedTrackersToFinish++;
        }
    }
    if (m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()) { emit trackingFailed("No trackers launched."); m_isTrackingRunning = false; }
    else if (m_expectedTrackersToFinish > 0) { emit trackingStatusUpdate(QString("Launching %1 trackers...").arg(m_expectedTrackersToFinish)); }
    updateOverallProgress();
}
void TrackingManager::updateOverallProgress() { /* ... same as your version ... */
    if (m_cancelRequested && !m_isTrackingRunning) { emit overallTrackingProgress(m_videoProcessingOverallProgress); return; }
    if (!m_isTrackingRunning && !m_cancelRequested) { emit overallTrackingProgress(0); return; }
    double totalProgressValue = 0.0; double videoProcWeight = (m_expectedTrackersToFinish > 0 || m_initialWormInfos.empty()) ? 0.20 : 1.0;
    double trackersWeight = 1.0 - videoProcWeight;
    totalProgressValue += (static_cast<double>(m_videoProcessingOverallProgress) / 100.0) * videoProcWeight;
    double overallTrackerPercentage = 0.0;
    { QMutexLocker locker(&m_dataMutex);
        if (m_expectedTrackersToFinish > 0) {
            double currentPoints = 0; for(int p : m_individualTrackerProgress.values()) currentPoints += p;
            currentPoints += (static_cast<double>(m_finishedTrackersCount) * 100.0);
            double maxPoints = static_cast<double>(m_expectedTrackersToFinish) * 100.0;
            if (maxPoints > 0) overallTrackerPercentage = currentPoints / maxPoints;
            overallTrackerPercentage = qBound(0.0, overallTrackerPercentage, 1.0);
        } else if (m_videoProcessingOverallProgress == 100 && m_initialWormInfos.empty()) overallTrackerPercentage = 1.0;
        else if (m_videoProcessingOverallProgress == 100 && m_expectedTrackersToFinish == 0 && !m_initialWormInfos.empty()) overallTrackerPercentage = 0.0;
    }
    if (m_expectedTrackersToFinish > 0 || (m_initialWormInfos.empty() && videoProcWeight < 1.0) ) totalProgressValue += overallTrackerPercentage * trackersWeight;
    emit overallTrackingProgress(qBound(0, static_cast<int>(totalProgressValue * 100.0), 100));
}
void TrackingManager::checkForAllTrackersFinished() { /* ... same as your version ... */
    QMutexLocker locker(&m_dataMutex);
    bool allDoneOrCancelled = false;
    if (m_isTrackingRunning && (m_finishedTrackersCount >= m_expectedTrackersToFinish)) allDoneOrCancelled = true;
    else if (m_cancelRequested && (m_finishedTrackersCount >= m_expectedTrackersToFinish || m_expectedTrackersToFinish == 0) ) allDoneOrCancelled = true;

    if (allDoneOrCancelled) {
        bool wasCancelled = m_cancelRequested; m_isTrackingRunning = false;

        // Clear any remaining split resolutions
        m_splitResolutionMap.clear();

        locker.unlock();
        if (wasCancelled) {
            m_finalTracks.clear(); for (WormObject* w : m_wormObjectsMap.values()) { if(w) m_finalTracks[w->getId()] = w->getTrackHistory(); }
            if (!m_finalTracks.empty()) emit allTracksUpdated(m_finalTracks);
            emit trackingStatusUpdate("Tracking cancelled."); emit trackingCancelled();
        } else {
            m_finalTracks.clear(); for (WormObject* w : m_wormObjectsMap.values()) { if(w) m_finalTracks[w->getId()] = w->getTrackHistory(); }
            emit allTracksUpdated(m_finalTracks);
            QString csvPath;
            if (m_videoPath.isEmpty()) {
                csvPath = "tracks.csv";
            } else if (!m_videoSpecificDirectory.isEmpty() && QDir(m_videoSpecificDirectory).exists()) {
                csvPath = QDir(m_videoSpecificDirectory).filePath(QFileInfo(m_videoPath).completeBaseName() + "_tracks.csv");
            } else {
                // Fallback to video directory if video-specific directory is not available
                csvPath = QDir(QFileInfo(m_videoPath).absolutePath()).filePath(QFileInfo(m_videoPath).completeBaseName() + "_tracks.csv");
            }
            if (outputTracksToCsv(m_finalTracks, csvPath)) {
                emit trackingStatusUpdate("Tracks saved: " + csvPath);

                // Populate merge history storage in one batch before saving JSON state
                populateMergeHistoryInStorage();

                // Save frame-atomic JSON state instead of thresholded video
                saveFrameAtomicStateToJson(m_videoSpecificDirectory,
                                           m_nextPhysicalBlobId,
                                           m_frameMergeRecords,
                                           m_splitResolutionMap,
                                           m_wormToPhysicalBlobIdMap,
                                           m_thresholdSettings);

                emit trackingFinishedSuccessfully(csvPath);
            }
            else { emit trackingStatusUpdate("Failed to save CSV: " + csvPath); emit trackingFailed("Failed to save CSV."); }
        }
        // Only cleanup immediately if not saving video, otherwise defer until video saving completes
        if (!m_isVideoSaving) {
            QMetaObject::invokeMethod(this, "cleanupThreadsAndObjects", Qt::QueuedConnection);
        } else {
            TRACKING_DEBUG() << "TrackingManager: Deferring memory cleanup until video saving completes";
        }
    } else if (!m_isTrackingRunning && m_cancelRequested) {
        locker.unlock(); emit trackingCancelled();
    }
}
void TrackingManager::handleWormTrackerFinished() { /* ... same as your version ... */
    WormTracker* ft = qobject_cast<WormTracker*>(sender()); if (!ft) { QMutexLocker l(&m_dataMutex); if(m_expectedTrackersToFinish > m_finishedTrackersCount) m_finishedTrackersCount++; l.unlock(); checkForAllTrackersFinished(); return; }
    int cId = ft->getWormId(); WormTracker::TrackingDirection dir = ft->getDirection();
    QMutexLocker locker(&m_dataMutex); m_wormTrackersList.removeOne(ft);
    if(dir == WormTracker::TrackingDirection::Forward) m_wormIdToForwardTrackerInstanceMap.remove(cId); else m_wormIdToBackwardTrackerInstanceMap.remove(cId);
    m_individualTrackerProgress.remove(ft); m_finishedTrackersCount++; locker.unlock();
    updateOverallProgress(); checkForAllTrackersFinished();
}
void TrackingManager::handleWormTrackerError(int reportingWormId, QString errorMessage) { /* ... same as your version ... */
    WormTracker* et = qobject_cast<WormTracker*>(sender());
    QMutexLocker locker(&m_dataMutex);
    if (m_cancelRequested || !m_isTrackingRunning) { /* Handle error during cancel/stop */ if(et && m_wormTrackersList.removeOne(et)){ m_individualTrackerProgress.remove(et); if(m_expectedTrackersToFinish > m_finishedTrackersCount) m_finishedTrackersCount++;} locker.unlock(); updateOverallProgress(); checkForAllTrackersFinished(); return; }
    locker.unlock();
    //qWarning() << "Error from tracker" << reportingWormId << ":" << errorMessage;
    locker.relock();
    if (et && m_wormTrackersList.removeOne(et)) {
        m_individualTrackerProgress.remove(et);
        if(et->getDirection() == WormTracker::TrackingDirection::Forward) m_wormIdToForwardTrackerInstanceMap.remove(reportingWormId); else m_wormIdToBackwardTrackerInstanceMap.remove(reportingWormId);
        m_finishedTrackersCount++;
    } else if (!et && m_expectedTrackersToFinish > m_finishedTrackersCount) { m_finishedTrackersCount++; }
    locker.unlock();
    emit trackingStatusUpdate(QString("Error tracker %1.").arg(reportingWormId));
    updateOverallProgress(); checkForAllTrackersFinished();
}
void TrackingManager::handleWormTrackerProgress(int, int percentDone) { /* ... same as your version ... */
    if (m_cancelRequested || !m_isTrackingRunning) return; WormTracker* trk = qobject_cast<WormTracker*>(sender());
    QMutexLocker locker(&m_dataMutex); if (trk && m_wormTrackersList.contains(trk)) { m_individualTrackerProgress[trk] = percentDone; locker.unlock(); updateOverallProgress(); }
}
bool TrackingManager::outputTracksToCsv(const Tracking::AllWormTracks& tracks, const QString& outputFilePath) const { /* ... same as your version ... */
    if (outputFilePath.isEmpty()) return false; QFile f(outputFilePath); if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) return false;
    QTextStream o(&f); o << "WormID,Frame,PositionX,PositionY,RoiX,RoiY,RoiWidth,RoiHeight,Quality\n";
    for (auto const& [wId, tps] : tracks) {
        for (const Tracking::WormTrackPoint& p : tps) {
            o << wId << "," << p.frameNumberOriginal << "," << QString::number(p.position.x, 'f', 4) << "," << QString::number(p.position.y, 'f', 4) << ","
              << QString::number(p.roi.x(), 'f', 2) << "," << QString::number(p.roi.y(), 'f', 2) << ","
              << QString::number(p.roi.width(), 'f', 2) << "," << QString::number(p.roi.height(), 'f', 2) << ","
              << static_cast<int>(p.quality) << "\n"; }
        o << "\n";
    }
    f.close(); return f.error() == QFile::NoError;
}

QString TrackingManager::createVideoSpecificDirectory(const QString& dataDirectory, const QString& videoPath) {
    if (dataDirectory.isEmpty() || videoPath.isEmpty()) {
        qWarning() << "TrackingManager: Invalid data directory or video path";
        return QString();
    }

    QFileInfo videoInfo(videoPath);
    QString videoBaseName = videoInfo.completeBaseName(); // Gets filename without extension
    QString videoSpecificPath = QDir(dataDirectory).absoluteFilePath(videoBaseName);

    QDir videoSpecificDir(videoSpecificPath);
    if (!videoSpecificDir.exists()) {
        if (QDir().mkpath(videoSpecificPath)) {
            TRACKING_DEBUG() << "TrackingManager: Created video-specific directory:" << videoSpecificPath;
        } else {
            qWarning() << "TrackingManager: Failed to create video-specific directory:" << videoSpecificPath;
            return QString();
        }
    } else {
        TRACKING_DEBUG() << "TrackingManager: Using existing video-specific directory:" << videoSpecificPath;
    }

    return videoSpecificPath;
}

// Batch-populate merge groups into the central TrackingDataStorage.
// Converts FrameSpecificPhysicalBlob records (m_frameMergeRecords) into
// the storage representation: QMap<int, QList<QList<int>>> (frame -> list of groups).
void TrackingManager::populateMergeHistoryInStorage() {
    if (!m_storage) return;
    QMutexLocker locker(&m_dataMutex);
    // Iterate frames in m_frameMergeRecords and convert each FrameSpecificPhysicalBlob.participatingWormTrackerIDs
    // into a QList<int> group of unsigned conceptual IDs, then write to storage.
    for (auto it = m_frameMergeRecords.constBegin(); it != m_frameMergeRecords.constEnd(); ++it) {
        int frameNum = it.key();
        const QList<FrameSpecificPhysicalBlob>& blobs = it.value();
        QList<QList<int>> groups;
        groups.reserve(blobs.size());
        for (const FrameSpecificPhysicalBlob& pb : blobs) {
            QList<int> group;
            group.reserve(pb.participatingWormTrackerIDs.size());
            for (int signedId : pb.participatingWormTrackerIDs) {
                int unsignedId = getUnsignedWormId(signedId);
                if (unsignedId >= 0 && !group.contains(unsignedId)) group.append(unsignedId);
            }
            if (!group.isEmpty()) groups.append(group);
        }
        // Write to storage (storage will silently ignore invalid frames)
        m_storage->setMergeGroupsForFrame(frameNum, groups);
    }
}


void TrackingManager::saveThresholdSettings(const QString& directoryPath, const Thresholding::ThresholdSettings& settings) {
    if (directoryPath.isEmpty()) {
        qWarning() << "TrackingManager: Cannot save threshold settings - empty directory path";
        return;
    }

    QJsonObject jsonObj = thresholdSettingsToJson(settings);
    QJsonDocument doc(jsonObj);

    QString filePath = QDir(directoryPath).absoluteFilePath("thresh_settings.json");
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(doc.toJson());
        file.close();
        TRACKING_DEBUG() << "TrackingManager: Saved threshold settings to:" << filePath;
    } else {
        qWarning() << "TrackingManager: Failed to save threshold settings to:" << filePath;
    }
}

void TrackingManager::saveInputBlobs(const QString& directoryPath, const std::vector<Tracking::InitialWormInfo>& worms) {
    if (directoryPath.isEmpty()) {
        qWarning() << "TrackingManager: Cannot save input blobs - empty directory path";
        return;
    }

    QJsonArray wormsArray;
    for (const auto& worm : worms) {
        wormsArray.append(initialWormInfoToJson(worm));
    }

    QJsonObject rootObj;
    rootObj["worms"] = wormsArray;
    rootObj["count"] = static_cast<int>(worms.size());

    QJsonDocument doc(rootObj);
    QString filePath = QDir(directoryPath).absoluteFilePath("input_blobs.json");
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(doc.toJson());
        file.close();
        TRACKING_DEBUG() << "TrackingManager: Saved input blobs to:" << filePath;
    } else {
        qWarning() << "TrackingManager: Failed to save input blobs to:" << filePath;
    }
}

bool TrackingManager::compareThresholdSettings(const QString& filePath, const Thresholding::ThresholdSettings& currentSettings) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "TrackingManager: Cannot read threshold settings file:" << filePath;
        return false;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError) {
        qWarning() << "TrackingManager: JSON parse error in threshold settings:" << parseError.errorString();
        return false;
    }

    QJsonObject stored = doc.object();
    QJsonObject current = thresholdSettingsToJson(currentSettings);

    // Compare key settings and log differences
    bool match = true;
    QStringList differences;

    if (stored["algorithm"].toInt() != current["algorithm"].toInt()) {
        differences << QString("algorithm: %1 vs %2").arg(stored["algorithm"].toInt()).arg(current["algorithm"].toInt());
        match = false;
    }
    if (stored["globalThresholdValue"].toInt() != current["globalThresholdValue"].toInt()) {
        differences << QString("globalThresholdValue: %1 vs %2").arg(stored["globalThresholdValue"].toInt()).arg(current["globalThresholdValue"].toInt());
        match = false;
    }
    if (stored["assumeLightBackground"].toBool() != current["assumeLightBackground"].toBool()) {
        differences << QString("assumeLightBackground: %1 vs %2").arg(stored["assumeLightBackground"].toBool()).arg(current["assumeLightBackground"].toBool());
        match = false;
    }
    if (stored["adaptiveBlockSize"].toInt() != current["adaptiveBlockSize"].toInt()) {
        differences << QString("adaptiveBlockSize: %1 vs %2").arg(stored["adaptiveBlockSize"].toInt()).arg(current["adaptiveBlockSize"].toInt());
        match = false;
    }
    if (qAbs(stored["adaptiveCValue"].toDouble() - current["adaptiveCValue"].toDouble()) >= 0.001) {
        differences << QString("adaptiveCValue: %1 vs %2").arg(stored["adaptiveCValue"].toDouble()).arg(current["adaptiveCValue"].toDouble());
        match = false;
    }
    if (stored["enableBlur"].toBool() != current["enableBlur"].toBool()) {
        differences << QString("enableBlur: %1 vs %2").arg(stored["enableBlur"].toBool()).arg(current["enableBlur"].toBool());
        match = false;
    }
    if (stored["blurKernelSize"].toInt() != current["blurKernelSize"].toInt()) {
        differences << QString("blurKernelSize: %1 vs %2").arg(stored["blurKernelSize"].toInt()).arg(current["blurKernelSize"].toInt());
        match = false;
    }
    if (qAbs(stored["blurSigmaX"].toDouble() - current["blurSigmaX"].toDouble()) >= 0.001) {
        differences << QString("blurSigmaX: %1 vs %2").arg(stored["blurSigmaX"].toDouble()).arg(current["blurSigmaX"].toDouble());
        match = false;
    }

    if (!match) {
        TRACKING_DEBUG() << "TrackingManager: Threshold settings differ:";
        for (const QString& diff : differences) {
            TRACKING_DEBUG() << "  " << diff;
        }
    } else {
        TRACKING_DEBUG() << "TrackingManager: Threshold settings match stored values";
    }

    return match;
}

QJsonObject TrackingManager::thresholdSettingsToJson(const Thresholding::ThresholdSettings& settings) const {
    QJsonObject obj;
    obj["algorithm"] = static_cast<int>(settings.algorithm);
    obj["globalThresholdValue"] = settings.globalThresholdValue;
    obj["assumeLightBackground"] = settings.assumeLightBackground;
    obj["adaptiveBlockSize"] = settings.adaptiveBlockSize;
    obj["adaptiveCValue"] = settings.adaptiveCValue;
    obj["enableBlur"] = settings.enableBlur;
    obj["blurKernelSize"] = settings.blurKernelSize;
    obj["blurSigmaX"] = settings.blurSigmaX;
    return obj;
}

QJsonObject TrackingManager::initialWormInfoToJson(const Tracking::InitialWormInfo& worm) const {
    QJsonObject obj;
    obj["id"] = worm.id;

    QJsonObject roiObj;
    roiObj["x"] = worm.initialRoi.x();
    roiObj["y"] = worm.initialRoi.y();
    roiObj["width"] = worm.initialRoi.width();
    roiObj["height"] = worm.initialRoi.height();
    obj["initialRoi"] = roiObj;

    return obj;
}

size_t TrackingManager::getProcessedVideoMemoryUsage() const {
    size_t totalBytes = 0;

    // Calculate memory usage of forward frames
    for (const cv::Mat& frame : m_finalProcessedForwardFrames) {
        if (!frame.empty()) {
            totalBytes += frame.total() * frame.elemSize();
        }
    }

    // Calculate memory usage of backward frames
    for (const cv::Mat& frame : m_finalProcessedReversedFrames) {
        if (!frame.empty()) {
            totalBytes += frame.total() * frame.elemSize();
        }
    }

    return totalBytes;
}

void TrackingManager::clearProcessedVideoMemory() {
    TRACKING_DEBUG() << "TrackingManager: Clearing processed video memory...";

    // Calculate memory usage before clearing
    size_t memoryUsageBefore = getProcessedVideoMemoryUsage();
    size_t forwardSize = m_finalProcessedForwardFrames.size();
    size_t backwardSize = m_finalProcessedReversedFrames.size();

    // Clear and shrink the vectors to free memory
    m_finalProcessedForwardFrames.clear();
    m_finalProcessedReversedFrames.clear();
    std::vector<cv::Mat>().swap(m_finalProcessedForwardFrames);
    std::vector<cv::Mat>().swap(m_finalProcessedReversedFrames);

    // Also clear intermediate chunk storage if still present
    m_assembledForwardFrameChunks.clear();
    m_assembledBackwardFrameChunks.clear();

    // Report memory savings
    double memoryMB = memoryUsageBefore / (1024.0 * 1024.0);
    TRACKING_DEBUG() << "TrackingManager: Cleared" << forwardSize << "forward frames and" << backwardSize
             << "backward frames from memory, freed" << QString::number(memoryMB, 'f', 1) << "MB";

    QString statusMessage = QString("Processed video data cleared from memory (freed %1 MB)")
                           .arg(QString::number(memoryMB, 'f', 1));
    emit trackingStatusUpdate(statusMessage);
}

// ===== VideoSaverWorker Implementation =====

VideoSaverWorker::VideoSaverWorker(QObject* parent) : QObject(parent), m_fps(0.0) {
    // Connect self-cleanup
    connect(this, &VideoSaverWorker::savingComplete, this, &VideoSaverWorker::deleteLater);
    connect(this, &VideoSaverWorker::savingError, this, &VideoSaverWorker::deleteLater);
}

void VideoSaverWorker::receiveVideoData(std::vector<cv::Mat> reversedFrames,
                                       std::vector<cv::Mat> forwardFrames,
                                       const QString& outputPath,
                                       double fps,
                                       cv::Size frameSize) {
    // Now running in background thread - safe to receive large data
    qDebug() << "VideoSaverWorker: Received data - Reversed frames:" << reversedFrames.size()
             << "Forward frames:" << forwardFrames.size() << "Output:" << outputPath;

    // Validate input parameters before proceeding
    if (outputPath.isEmpty()) {
        emit savingError("Empty output path provided");
        return;
    }

    if (fps <= 0) {
        emit savingError("Invalid FPS value: " + QString::number(fps));
        return;
    }

    if (frameSize.width <= 0 || frameSize.height <= 0) {
        emit savingError("Invalid frame size: " + QString::number(frameSize.width) + "x" + QString::number(frameSize.height));
        return;
    }

    if (reversedFrames.empty() && forwardFrames.empty()) {
        emit savingError("No frames provided for saving");
        return;
    }

    // Validate frame consistency
    cv::Size expectedSize = frameSize;
    for (size_t i = 0; i < reversedFrames.size() && i < 5; ++i) {  // Check first 5 frames
        if (!reversedFrames[i].empty() && (reversedFrames[i].size() != expectedSize)) {
            qWarning() << "VideoSaverWorker: Frame size mismatch in reversed frames at index" << i
                      << "Expected:" << expectedSize.width << "x" << expectedSize.height
                      << "Got:" << reversedFrames[i].size().width << "x" << reversedFrames[i].size().height;
        }
    }

    for (size_t i = 0; i < forwardFrames.size() && i < 5; ++i) {  // Check first 5 frames
        if (!forwardFrames[i].empty() && (forwardFrames[i].size() != expectedSize)) {
            qWarning() << "VideoSaverWorker: Frame size mismatch in forward frames at index" << i
                      << "Expected:" << expectedSize.width << "x" << expectedSize.height
                      << "Got:" << forwardFrames[i].size().width << "x" << forwardFrames[i].size().height;
        }
    }

    m_reversedFrames = std::move(reversedFrames);
    m_forwardFrames = std::move(forwardFrames);
    m_outputPath = outputPath;
    m_fps = fps;
    m_frameSize = frameSize;

    qDebug() << "VideoSaverWorker: Data move complete - Reversed frames:" << m_reversedFrames.size()
             << "Forward frames:" << m_forwardFrames.size();

    // Start saving immediately
    startSaving();
}

void VideoSaverWorker::startSaving() {

    try {
        qDebug() << "VideoSaverWorker: Starting video save process";
        qDebug() << "VideoSaverWorker: Video parameters - FPS:" << m_fps << "Size:" << m_frameSize.width << "x" << m_frameSize.height;

        // Create video writer
        cv::VideoWriter writer;
        int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // MJPEG codec for better compatibility

        if (!writer.open(m_outputPath.toStdString(), fourcc, m_fps, m_frameSize, false)) {
            qDebug() << "VideoSaverWorker: Failed to open video writer";
            emit savingError("Failed to open video writer for: " + m_outputPath);
            return;
        }

        qDebug() << "VideoSaverWorker: Video writer opened successfully";

        // Calculate total frames for progress reporting
        size_t totalFrames = m_reversedFrames.size() + m_forwardFrames.size();
        size_t framesWritten = 0;

        qDebug() << "VideoSaverWorker: Total frames to write:" << totalFrames;

        // Write reversed frames in reverse order (they represent frames 0->keyframe, but are stored keyframe->0)
        // So we need to write them in reverse to get the correct chronological order (0->keyframe)
        qDebug() << "VideoSaverWorker: Writing reversed frames in reverse order, count:" << m_reversedFrames.size();
        for (auto it = m_reversedFrames.rbegin(); it != m_reversedFrames.rend(); ++it) {
            const auto& frame = *it;
            if (!frame.empty()) {
                writer.write(frame);
                framesWritten++;

                // Report progress every 25 frames for more frequent updates
                if (framesWritten % 25 == 0) {
                    int progress = static_cast<int>((framesWritten * 100) / totalFrames);
                    qDebug() << "VideoSaverWorker: Progress:" << progress << "% (" << framesWritten << "/" << totalFrames << ")";
                    emit savingProgress(progress);
                }
            }
        }
        qDebug() << "VideoSaverWorker: Finished writing reversed frames";

        // Write forward frames in normal order (these represent frames keyframe->end)
        qDebug() << "VideoSaverWorker: Writing forward frames in normal order, count:" << m_forwardFrames.size();
        for (const auto& frame : m_forwardFrames) {
            if (!frame.empty()) {
                writer.write(frame);
                framesWritten++;

                // Report progress every 25 frames for more frequent updates
                if (framesWritten % 25 == 0) {
                    int progress = static_cast<int>((framesWritten * 100) / totalFrames);
                    qDebug() << "VideoSaverWorker: Progress:" << progress << "% (" << framesWritten << "/" << totalFrames << ")";
                    emit savingProgress(progress);
                }
            }
        }
        qDebug() << "VideoSaverWorker: Finished writing forward frames, total written:" << framesWritten;

        writer.release();
        qDebug() << "VideoSaverWorker: Video writer released successfully";
        emit savingProgress(100);
        qDebug() << "VideoSaverWorker: Video saving completed successfully:" << m_outputPath;
        emit savingComplete(m_outputPath);

    } catch (const std::exception& e) {
        qDebug() << "VideoSaverWorker: Exception during video saving:" << e.what();
        emit savingError("Exception during video saving: " + QString::fromStdString(e.what()));
    } catch (...) {
        qDebug() << "VideoSaverWorker: Unknown exception during video saving";
        emit savingError("Unknown exception during video saving");
    }
}

// ===== TrackingManager Video Saving Methods =====

void TrackingManager::startVideoSaving() {
    if (m_finalProcessedForwardFrames.empty() && m_finalProcessedReversedFrames.empty()) {
        TRACKING_DEBUG() << "TrackingManager: No processed frames to save";
        return;
    }

    // Determine output path
    if (m_videoSpecificDirectory.isEmpty()) {
        qWarning() << "TrackingManager: No video-specific directory available for saving thresholded video";
        return;
    }

    QFileInfo videoInfo(m_videoPath);
    QString baseName = videoInfo.completeBaseName();
    m_savedVideoPath = QDir(m_videoSpecificDirectory).absoluteFilePath(baseName + "_thresholded.avi");

    TRACKING_DEBUG() << "TrackingManager: Starting video saving to" << m_savedVideoPath;

    // Create worker and thread
    VideoSaverWorker* worker = new VideoSaverWorker();
    m_videoSaverThread = new QThread();

    // Move worker to thread
    worker->moveToThread(m_videoSaverThread);

    // Set up connections - connect to receiveVideoData instead of startSaving
    connect(this, &TrackingManager::sendVideoDataToWorker,
            worker, &VideoSaverWorker::receiveVideoData);
    connect(worker, &VideoSaverWorker::savingComplete, this, &TrackingManager::handleVideoSavingComplete);
    connect(worker, &VideoSaverWorker::savingError, this, &TrackingManager::handleVideoSavingError);
    connect(worker, &VideoSaverWorker::savingProgress, this, [this](int progress) {
        emit trackingStatusUpdate(QString("Saving thresholded video: %1%").arg(progress));
    });

    // Log data size before transfer
    TRACKING_DEBUG() << "TrackingManager: About to transfer video data - Reversed frames:" << m_finalProcessedReversedFrames.size()
                     << "Forward frames:" << m_finalProcessedForwardFrames.size()
                     << "Total memory:" << getProcessedVideoMemoryUsage() << "bytes";

    // Start the thread first
    TRACKING_DEBUG() << "TrackingManager: Starting video saver thread";
    m_isVideoSaving = true; // Mark video saving as active
    m_videoSaverThread->start();

    // Wait a brief moment to ensure thread is fully started
    QThread::msleep(10);

    // Then send data via signal (worker is now in background thread)
    TRACKING_DEBUG() << "TrackingManager: Emitting sendVideoDataToWorker signal";
    emit sendVideoDataToWorker(std::move(m_finalProcessedReversedFrames), std::move(m_finalProcessedForwardFrames),
                              m_savedVideoPath, m_videoFps, m_videoFrameSize);

    TRACKING_DEBUG() << "TrackingManager: Data transfer signal emitted - local vectors now have sizes:"
                     << m_finalProcessedReversedFrames.size() << "and" << m_finalProcessedForwardFrames.size();
}

void TrackingManager::handleVideoSavingComplete(const QString& savedVideoPath) {
    TRACKING_DEBUG() << "TrackingManager: Video saving completed successfully:" << savedVideoPath;
    emit trackingStatusUpdate("Thresholded video saved: " + QFileInfo(savedVideoPath).fileName());

    m_isVideoSaving = false; // Mark video saving as complete

    // Clean up thread and worker
    if (m_videoSaverThread) {
        m_videoSaverThread->quit();
        m_videoSaverThread->wait(5000); // Wait up to 5 seconds
        m_videoSaverThread->deleteLater();
        m_videoSaverThread = nullptr;
    }

    // Now perform the deferred cleanup
    TRACKING_DEBUG() << "TrackingManager: Performing deferred memory cleanup after video save completion";
    QMetaObject::invokeMethod(this, "cleanupThreadsAndObjects", Qt::QueuedConnection);
}

void TrackingManager::handleVideoSavingError(const QString& errorMessage) {
    qWarning() << "TrackingManager: Video saving failed:" << errorMessage;
    emit trackingStatusUpdate("Warning: Failed to save thresholded video - " + errorMessage);

    m_isVideoSaving = false; // Mark video saving as complete (even though it failed)

    // Clean up thread and worker
    if (m_videoSaverThread) {
        m_videoSaverThread->quit();
        m_videoSaverThread->wait(5000); // Wait up to 5 seconds
        m_videoSaverThread->deleteLater();
        m_videoSaverThread = nullptr;
    }

    // Still perform the deferred cleanup even on error
    TRACKING_DEBUG() << "TrackingManager: Performing deferred memory cleanup after video save error";
    QMetaObject::invokeMethod(this, "cleanupThreadsAndObjects", Qt::QueuedConnection);
}

QString TrackingManager::getSavedVideoPath() const {
    return m_savedVideoPath;
}

bool TrackingManager::startRetrackingProcess(const QString& thresholdedVideoPath,
                                           int fixBlobId,
                                           const QRectF& initialROI,
                                           int startFrame,
                                           int endFrame,
                                           bool replaceExisting,
                                           bool extendTrack) {
    TRACKING_DEBUG() << "TrackingManager: Starting retracking process for Fix blob" << fixBlobId
                     << "from frame" << startFrame << "to" << endFrame
                     << "using video:" << thresholdedVideoPath;

    // Check if tracking is already running
    if (m_isTrackingRunning) {
        qWarning() << "TrackingManager: Cannot start retracking while main tracking is running";
        return false;
    }

    // Validate parameters
    if (thresholdedVideoPath.isEmpty() || !QFileInfo::exists(thresholdedVideoPath)) {
        qWarning() << "TrackingManager: Thresholded video file does not exist:" << thresholdedVideoPath;
        return false;
    }

    if (startFrame >= endFrame || startFrame < 0) {
        qWarning() << "TrackingManager: Invalid frame range for retracking:" << startFrame << "to" << endFrame;
        return false;
    }

    if (initialROI.isEmpty()) {
        qWarning() << "TrackingManager: Invalid ROI for retracking";
        return false;
    }

    // TODO: Implement the actual retracking logic here
    // This would involve:
    // 1. Opening the saved thresholded video
    // 2. Creating a specialized tracker for the frame range
    // 3. Running tracking on the specified frames
    // 4. Integrating results with existing track data

    // For now, provide placeholder feedback
    qDebug() << "TrackingManager: Retracking process prepared but not yet fully implemented";
    qDebug() << "TrackingManager: Parameters - ROI:" << initialROI
             << "Replace:" << replaceExisting << "Extend:" << extendTrack;

    emit trackingStatusUpdate(QString("Retracking for Fix Blob ID %1 completed (placeholder)").arg(fixBlobId));

    return true;
}
