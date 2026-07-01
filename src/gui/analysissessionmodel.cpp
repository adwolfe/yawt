#include "analysissessionmodel.h"
#include "videometadatastore.h"
#include "../utils/yawtjsonio.h"

#include <QCoreApplication>
#include <QDataStream>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMap>
#include <QMimeData>
#include <QPixmap>
#include <QTimer>

// ─────────────────────────────────────────────────────────────────────────────
// Static helpers
// ─────────────────────────────────────────────────────────────────────────────

QColor AnalysisSessionModel::colormapColor(int index, int total)
{
    if (total <= 0) return Qt::gray;
    const double hue = static_cast<double>(index) / static_cast<double>(total);
    return QColor::fromHsvF(hue, 0.85, 0.92);
}

QIcon AnalysisSessionModel::makeColorIcon(const QColor& c)
{
    QPixmap pix(14, 14);
    pix.fill(c.isValid() ? c : Qt::gray);
    return QIcon(pix);
}

/** Read worms.json and return the IDs of Worm and Fix items. */
QList<int> AnalysisSessionModel::parseWormIds(const QString& wormsJsonPath)
{
    QJsonParseError err;
    const QJsonDocument doc = YawtJsonIO::readJsonDocument(wormsJsonPath, &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) return {};

    QList<int> ids;
    const QJsonArray items = doc.object().value("items").toArray();
    for (const QJsonValue& v : items) {
        if (!v.isObject()) continue;
        const QJsonObject obj = v.toObject();
        const QString type = obj.value("type").toString();
        if (type == "Worm" || type == "Fix")
            ids.append(obj.value("id").toInt());
    }
    return ids;
}

/** Load the tracks section of worms.json into an AllWormTracks map. */
Tracking::AllWormTracks AnalysisSessionModel::loadTracksFromJson(const QString& wormsJsonPath)
{
    Tracking::AllWormTracks tracks;

    QJsonParseError err;
    const QJsonDocument doc = YawtJsonIO::readJsonDocument(wormsJsonPath, &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) return tracks;

    const QJsonObject tracksObj = doc.object().value("tracks").toObject();
    for (auto it = tracksObj.constBegin(); it != tracksObj.constEnd(); ++it) {
        bool ok = false;
        const int wormId = it.key().toInt(&ok);
        if (!ok || !it.value().isArray()) continue;

        std::vector<Tracking::WormTrackPoint> points;
        for (const QJsonValue& pv : it.value().toArray()) {
            if (!pv.isObject()) continue;
            const QJsonObject pObj = pv.toObject();
            Tracking::WormTrackPoint p;
            p.frameNumberOriginal = pObj.value("frame").toInt();
            if (pObj.contains("position") && pObj["position"].isObject()) {
                const QJsonObject pos = pObj["position"].toObject();
                p.position = cv::Point2f(static_cast<float>(pos.value("x").toDouble()),
                                         static_cast<float>(pos.value("y").toDouble()));
            }
            p.quality = static_cast<Tracking::TrackPointQuality>(pObj.value("quality").toInt());

            // Blob-derived morphology (optional, written post-centerline)
            if (pObj.contains("area"))
                p.area = static_cast<float>(pObj.value("area").toDouble());
            if (pObj.contains("aspectRatio"))
                p.aspectRatio = static_cast<float>(pObj.value("aspectRatio").toDouble());

            // Body length from centerline points
            if (pObj.contains("centerlinePoints") && pObj["centerlinePoints"].isArray()) {
                const QJsonArray clArr = pObj["centerlinePoints"].toArray();
                float arcLen = 0.f;
                cv::Point2f prev{};
                bool hasPrev = false;
                for (const QJsonValue& cv : clArr) {
                    if (!cv.isArray() || cv.toArray().size() < 2) continue;
                    cv::Point2f pt(static_cast<float>(cv.toArray()[0].toDouble()),
                                   static_cast<float>(cv.toArray()[1].toDouble()));
                    if (hasPrev) {
                        cv::Point2f d = pt - prev;
                        arcLen += std::sqrt(d.x*d.x + d.y*d.y);
                    }
                    prev = pt;
                    hasPrev = true;
                }
                if (arcLen > 0.f) p.bodyLength = arcLen;
            }

            // Head/tail tips
            if (pObj.contains("tips") && pObj["tips"].isObject()) {
                const QJsonObject tips = pObj["tips"].toObject();
                if (tips.contains("head") && tips.contains("tail")) {
                    const QJsonObject h = tips["head"].toObject();
                    const QJsonObject t = tips["tail"].toObject();
                    p.headTip = cv::Point2f(static_cast<float>(h.value("x").toDouble()),
                                            static_cast<float>(h.value("y").toDouble()));
                    p.tailTip = cv::Point2f(static_cast<float>(t.value("x").toDouble()),
                                            static_cast<float>(t.value("y").toDouble()));
                    p.hasTips = true;
                }
            }

            points.push_back(p);
        }
        if (!points.empty()) {
            std::sort(points.begin(), points.end(),
                [](const Tracking::WormTrackPoint& a, const Tracking::WormTrackPoint& b) {
                    return a.frameNumberOriginal < b.frameNumberOriginal;
                });
            tracks[wormId] = std::move(points);
        }
    }
    return tracks;
}

/**
 * Build the list of human-readable warnings for a video proc run.
 * Currently checks:
 *  1. Pixel scale missing (umPerPixel == 0)
 *  2. Start point or end point absent from roi_points.json
 */
QStringList AnalysisSessionModel::buildWarnings(const QString& procDir,
                                                const QString& yawtDir,
                                                const QString& baseName,
                                                double umPerPixel)
{
    QStringList warnings;

    if (umPerPixel <= 0.0)
        warnings << "No pixel scale set (µm/pixel is unknown)";

    // Check roi_points.json for StartPoint and EndPoint
    const QString roiPath = QDir(procDir).absoluteFilePath("roi_points.json");
    QFile f(roiPath);
    if (!f.open(QIODevice::ReadOnly)) {
        warnings << "Missing start point and end point (roi_points.json not found)";
    } else {
        QJsonParseError err;
        const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
        bool hasStart = false, hasEnd = false;
        if (err.error == QJsonParseError::NoError && doc.isObject()) {
            for (const QJsonValue& v : doc.object().value("items").toArray()) {
                const QString type = v.toObject().value("type").toString();
                if (type == "Start Point") hasStart = true;
                if (type == "End Point")   hasEnd   = true;
            }
        }
        if (!hasStart && !hasEnd)
            warnings << "Missing start point and end point";
        else if (!hasStart)
            warnings << "Missing start point";
        else if (!hasEnd)
            warnings << "Missing end point";
    }

    Q_UNUSED(yawtDir)
    Q_UNUSED(baseName)
    return warnings;
}

void AnalysisSessionModel::loadRoiReferencePoints(VideoItem& vid)
{
    const QString roiPath = QDir(vid.procDir).absoluteFilePath("roi_points.json");
    QFile f(roiPath);
    if (!f.open(QIODevice::ReadOnly)) return;

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) return;

    const QJsonArray items = doc.object().value("items").toArray();
    for (const QJsonValue& v : items) {
        if (!v.isObject()) continue;
        const QJsonObject obj = v.toObject();
        const QString type = obj.value("type").toString();
        const QJsonObject centroid = obj.value("initialCentroid").toObject();
        const QPointF pt(centroid.value("x").toDouble(),
                         centroid.value("y").toDouble());

        if (type == "Start Point") {
            vid.hasStartPoint = true;
            vid.startPoint = pt;
        } else if (type == "End Point") {
            vid.hasEndPoint = true;
            vid.endPoint = pt;
        } else if (type == "Control Point" || type == "Center") {
            vid.hasCenterPoint = true;
            vid.centerPoint = pt;
        }
    }
}

/** Find the most recent PROC_* subfolder inside videoSubDir (sort descending by name). */
QString AnalysisSessionModel::findMostRecentProc(const QString& videoSubDir)
{
    const QStringList procs = QDir(videoSubDir).entryList(
        QStringList() << "PROC_*", QDir::Dirs | QDir::NoDotAndDotDot,
        QDir::Name | QDir::Reversed);
    if (procs.isEmpty()) return {};
    return QDir(videoSubDir).absoluteFilePath(procs.first());
}

// ─────────────────────────────────────────────────────────────────────────────
// Color recalculation
// ─────────────────────────────────────────────────────────────────────────────

void AnalysisSessionModel::recalcGroupColors(int g)
{
    if (g < 0 || g >= m_groups.size()) return;

    int total = 0;
    for (const auto& vid : m_groups[g].videos)
        total += vid.worms.size();

    int idx = 0;
    for (auto& vid : m_groups[g].videos)
        for (auto& worm : vid.worms)
            worm.color = colormapColor(idx++, total);
}

// ─────────────────────────────────────────────────────────────────────────────
// Persistence helpers
// ─────────────────────────────────────────────────────────────────────────────

QString AnalysisSessionModel::stateFilePath(const QString& yawtDir)
{
    return QDir(yawtDir).absoluteFilePath("analysis_state.json");
}

void AnalysisSessionModel::saveState() const
{
    if (m_yawtDir.isEmpty()) return;

    QJsonArray groupsArr;
    for (const auto& g : m_groups) {
        QJsonArray videosArr;
        for (const auto& v : g.videos) {
            QJsonArray checkedArr;
            for (const auto& w : v.worms)
                if (w.checked) checkedArr.append(w.id);

            QJsonObject vObj;
            vObj["baseName"]       = v.baseName;
            vObj["procStamp"]      = v.procStamp;
            vObj["checkedWormIds"] = checkedArr;
            videosArr.append(vObj);
        }
        QJsonObject gObj;
        gObj["name"]   = g.name;
        gObj["videos"] = videosArr;
        groupsArr.append(gObj);
    }

    QJsonObject root;
    root["version"] = 1;
    root["groups"]  = groupsArr;

    QFile f(stateFilePath(m_yawtDir));
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

void AnalysisSessionModel::scheduleStateSave()
{
    if (!m_saveTimer) return;
    m_saveTimer->start(800);   // debounce: save 800 ms after the last change
}

/**
 * Merge saved state with the current disk contents.
 *
 * diskVideos: baseName → (procDir, procStamp)   (all videos found on disk)
 *
 * Rules:
 *  - Groups and their video assignments are restored from the state file.
 *  - A video whose procStamp matches what's on disk is restored as-is
 *    (including saved check states).
 *  - A video present in the state but with a *newer* proc on disk keeps its
 *    group assignment but gets fresh track data and all worms re-checked.
 *  - A video present in the state but missing from disk is silently dropped.
 *  - A disk video not in the state at all is added to "Unassigned".
 */
void AnalysisSessionModel::loadAndMergeState(
    const QString& yawtDir,
    const QMap<QString, QPair<QString,QString>>& diskVideos)
{
    // ── Load state file ───────────────────────────────────────────────────────
    QJsonObject root;
    {
        QFile f(stateFilePath(yawtDir));
        if (f.open(QIODevice::ReadOnly)) {
            QJsonParseError err;
            const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
            if (err.error == QJsonParseError::NoError && doc.isObject())
                root = doc.object();
        }
    }

    // Track which disk videos have been placed somewhere
    QSet<QString> placed;
    int processedVideos = 0;
    const int totalVideos = qMax(1, diskVideos.size());

    if (!root.isEmpty()) {
        // ── Restore groups from state ─────────────────────────────────────────
        m_groups.clear();

        const QJsonArray groupsArr = root.value("groups").toArray();
        for (const QJsonValue& gv : groupsArr) {
            const QJsonObject gObj = gv.toObject();
            GroupItem gi;
            gi.name = gObj.value("name").toString();

            const QJsonArray videosArr = gObj.value("videos").toArray();
            for (const QJsonValue& vv : videosArr) {
                const QJsonObject vObj = vv.toObject();
                const QString baseName  = vObj.value("baseName").toString();
                const QString savedStamp = vObj.value("procStamp").toString();

                // Does this video still exist on disk?
                if (!diskVideos.contains(baseName)) continue;

                const auto& [diskProcDir, diskStamp] = diskVideos[baseName];
                placed.insert(baseName);
                emit directoryScanProgress(++processedVideos,
                                           totalVideos,
                                           QStringLiteral("Loading %1").arg(baseName));
                QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 10);

                // Load the worms.json for the current (possibly updated) proc
                const QString wormsJson = QDir(diskProcDir).absoluteFilePath("worms.json");
                const QList<int> wormIds = parseWormIds(wormsJson);
                if (wormIds.isEmpty()) continue;

                VideoItem vid;
                vid.baseName  = baseName;
                vid.procDir   = diskProcDir;
                vid.procStamp = diskStamp;
                VideoMetadataStore::loadUmPerPixel(yawtDir, baseName, vid.umPerPixel);
                VideoMetadataStore::loadFps(yawtDir, baseName, vid.fps);
                vid.warnings  = buildWarnings(diskProcDir, yawtDir, baseName, vid.umPerPixel);
                loadRoiReferencePoints(vid);

                const bool reprocessed = (diskStamp != savedStamp);

                // Rebuild the saved check set for this video
                QSet<int> savedChecked;
                if (!reprocessed) {
                    for (const QJsonValue& cv : vObj.value("checkedWormIds").toArray())
                        savedChecked.insert(cv.toInt());
                }

                for (int i = 0; i < wormIds.size(); ++i) {
                    WormItem w;
                    w.id      = wormIds[i];
                    w.color   = Qt::gray;
                    w.label   = QString("Worm %1").arg(i + 1);
                    // Reprocessed → all checked; otherwise restore saved state
                    w.checked = reprocessed ? true : savedChecked.contains(wormIds[i]);
                    vid.worms.append(w);
                }
                gi.videos.append(std::move(vid));
            }
            m_groups.append(std::move(gi));
        }

        // Ensure "Unassigned" exists as the first group
        if (m_groups.isEmpty() || m_groups.first().name != "Unassigned")
            m_groups.prepend(GroupItem{"Unassigned", {}});

    } else {
        // No state file: start fresh with just Unassigned
        m_groups.clear();
        m_groups.append(GroupItem{"Unassigned", {}});
    }

    // ── Add any disk videos not yet placed → Unassigned ──────────────────────
    for (auto it = diskVideos.constBegin(); it != diskVideos.constEnd(); ++it) {
        if (placed.contains(it.key())) continue;

        const QString& baseName   = it.key();
        const QString& diskProcDir = it.value().first;
        const QString& diskStamp  = it.value().second;
        emit directoryScanProgress(++processedVideos,
                                   totalVideos,
                                   QStringLiteral("Loading %1").arg(baseName));
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 10);

        const QString wormsJson = QDir(diskProcDir).absoluteFilePath("worms.json");
        const QList<int> wormIds = parseWormIds(wormsJson);
        if (wormIds.isEmpty()) continue;

        VideoItem vid;
        vid.baseName  = baseName;
        vid.procDir   = diskProcDir;
        vid.procStamp = diskStamp;
        VideoMetadataStore::loadUmPerPixel(yawtDir, baseName, vid.umPerPixel);
        VideoMetadataStore::loadFps(yawtDir, baseName, vid.fps);
        vid.warnings  = buildWarnings(diskProcDir, yawtDir, baseName, vid.umPerPixel);
        loadRoiReferencePoints(vid);

        for (int i = 0; i < wormIds.size(); ++i) {
            WormItem w;
            w.id      = wormIds[i];
            w.checked = true;
            w.color   = Qt::gray;
            w.label   = QString("Worm %1").arg(i + 1);
            vid.worms.append(w);
        }
        m_groups[0].videos.append(std::move(vid));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Construction / population
// ─────────────────────────────────────────────────────────────────────────────

AnalysisSessionModel::AnalysisSessionModel(QObject* parent)
    : QAbstractItemModel(parent)
{
    m_groups.append(GroupItem{"Unassigned", {}});

    // Debounce timer for saving check-state changes
    m_saveTimer = new QTimer(this);
    m_saveTimer->setSingleShot(true);
    connect(m_saveTimer, &QTimer::timeout, this, &AnalysisSessionModel::saveState);
}

void AnalysisSessionModel::scanYawtDirectory(const QString& yawtDir)
{
    m_yawtDir = yawtDir;

    // ── Build disk inventory: baseName → (procDir, procStamp) ────────────────
    QMap<QString, QPair<QString,QString>> diskVideos;
    const QStringList videoDirs = QDir(yawtDir).entryList(
        QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);

    emit directoryScanStarted(qMax(1, videoDirs.size()));
    int scannedDirs = 0;
    for (const QString& baseName : videoDirs) {
        emit directoryScanProgress(++scannedDirs,
                                   qMax(1, videoDirs.size()),
                                   QStringLiteral("Scanning %1").arg(baseName));
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 10);

        const QString subDir  = QDir(yawtDir).absoluteFilePath(baseName);
        const QString procDir = findMostRecentProc(subDir);
        if (procDir.isEmpty()) continue;
        const QString stamp   = QFileInfo(procDir).fileName().mid(5); // strip "PROC_"
        diskVideos[baseName]  = {procDir, stamp};
    }

    emit directoryScanStarted(qMax(1, diskVideos.size()));

    // ── Merge with saved state (preserves group assignments) ─────────────────
    beginResetModel();
    loadAndMergeState(yawtDir, diskVideos);

    // Recalculate colors for every group
    for (int g = 0; g < m_groups.size(); ++g)
        recalcGroupColors(g);

    ++m_dataRevision;
    endResetModel();
    emit directoryScanFinished();

    // Persist the (possibly updated) state immediately
    saveState();
}

void AnalysisSessionModel::addGroup(const QString& name)
{
    if (name.trimmed().isEmpty()) return;
    const int row = m_groups.size();
    beginInsertRows({}, row, row);
    m_groups.append(GroupItem{name, {}});
    ++m_dataRevision;
    endInsertRows();
    saveState();  // structural change
}

void AnalysisSessionModel::setCheckedForProcDir(const QString& procDir, bool checked)
{
    for (auto& g : m_groups) {
        for (auto& v : g.videos) {
            if (v.procDir == procDir) {
                for (auto& w : v.worms)
                    w.checked = checked;
            }
        }
    }
    // Emit dataChanged for all worm indices — simplest with a reset
    // (only called infrequently, so the cost is acceptable)
    emit dataChanged(index(0,0), index(m_groups.size()-1, 0));
    ++m_checkRevision;
    emit checkedWormIdsChanged();
}

void AnalysisSessionModel::setCheckedWormIds(const QSet<int>& ids)
{
    for (auto& g : m_groups)
        for (auto& v : g.videos)
            for (auto& w : v.worms)
                w.checked = ids.contains(w.id);

    emit dataChanged(index(0,0), index(m_groups.size()-1, 0));
    ++m_checkRevision;
    emit checkedWormIdsChanged();
}

QList<AnalysisSessionModel::AnalysisGroupData>
AnalysisSessionModel::getGroupedData() const
{
    QList<AnalysisGroupData> result;

    for (const auto& g : m_groups) {
        AnalysisGroupData gd;
        gd.name = g.name;

        for (const auto& vid : g.videos) {
            if (!vid.tracksLoaded) {
                const QString wormsJson = QDir(vid.procDir).absoluteFilePath("worms.json");
                vid.tracks = loadTracksFromJson(wormsJson);
                vid.tracksLoaded = true;
            }

            for (const auto& worm : vid.worms) {
                if (!worm.checked) continue;

                const auto it = vid.tracks.find(worm.id);
                if (it == vid.tracks.end() || it->second.empty()) continue;

                AnalysisWormEntry entry;
                entry.wormId       = worm.id;
                entry.label        = worm.label;
                entry.color        = worm.color;
                entry.umPerPixel   = vid.umPerPixel;
                entry.fps          = vid.fps;
                entry.videoBaseName = vid.baseName;
                entry.points       = it->second;  // full copy, sorted in loadTracksFromJson
                entry.hasStartPoint = vid.hasStartPoint;
                entry.startPoint    = vid.startPoint;
                entry.hasEndPoint   = vid.hasEndPoint;
                entry.endPoint      = vid.endPoint;
                entry.hasCenterPoint = vid.hasCenterPoint;
                entry.centerPoint    = vid.centerPoint;
                gd.worms.append(std::move(entry));
            }
        }

        if (!gd.worms.isEmpty())
            result.append(std::move(gd));
    }

    return result;
}

QSet<int> AnalysisSessionModel::checkedWormIds() const
{
    QSet<int> result;
    for (const auto& g : m_groups)
        for (const auto& v : g.videos)
            for (const auto& w : v.worms)
                if (w.checked) result.insert(w.id);
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// QAbstractItemModel interface
// ─────────────────────────────────────────────────────────────────────────────

QModelIndex AnalysisSessionModel::index(int row, int column,
                                        const QModelIndex& parent) const
{
    if (row < 0 || column != 0) return {};

    if (!parent.isValid()) {
        // Root → return a group node
        if (row < m_groups.size())
            return createIndex(row, 0, kGroupSentinel);
        return {};
    }

    if (isGroup(parent)) {
        const int g = parent.row();
        if (g < m_groups.size() && row < m_groups[g].videos.size())
            return createIndex(row, 0, static_cast<quintptr>(g));
        return {};
    }

    if (isVideo(parent)) {
        const int g = static_cast<int>(parent.internalId());
        const int v = parent.row();
        if (g < m_groups.size() && v < m_groups[g].videos.size()
                && row < m_groups[g].videos[v].worms.size())
            return createIndex(row, 0,
                kWormFlag | (static_cast<quintptr>(g) << 16)
                          | static_cast<quintptr>(v));
        return {};
    }

    return {};
}

QModelIndex AnalysisSessionModel::parent(const QModelIndex& child) const
{
    if (!child.isValid()) return {};

    const quintptr id = child.internalId();

    if (id == kGroupSentinel)
        return {};  // group node → parent is root

    if (id & kWormFlag) {
        // Worm node → parent is the video node
        const int g = static_cast<int>((id >> 16) & 0xFFFF);
        const int v = static_cast<int>(id & 0xFFFF);
        return createIndex(v, 0, static_cast<quintptr>(g));
    }

    // Video node → parent is the group node
    const int g = static_cast<int>(id);
    return createIndex(g, 0, kGroupSentinel);
}

int AnalysisSessionModel::rowCount(const QModelIndex& parent) const
{
    if (!parent.isValid())
        return m_groups.size();

    if (isGroup(parent)) {
        const int g = parent.row();
        return (g < m_groups.size()) ? m_groups[g].videos.size() : 0;
    }

    if (isVideo(parent)) {
        const int g = static_cast<int>(parent.internalId());
        const int v = parent.row();
        if (g < m_groups.size() && v < m_groups[g].videos.size())
            return m_groups[g].videos[v].worms.size();
    }

    return 0;
}

int AnalysisSessionModel::columnCount(const QModelIndex&) const
{
    return 1;
}

QVariant AnalysisSessionModel::data(const QModelIndex& idx, int role) const
{
    if (!idx.isValid()) return {};

    // ── Group node ──────────────────────────────────────────────────────────
    if (isGroup(idx)) {
        const auto& g = m_groups[idx.row()];
        switch (role) {
        case Qt::EditRole:
            return g.name;   // plain name for the in-place editor
        case Qt::DisplayRole: {
            int n = 0;
            for (const auto& v : g.videos) n += v.worms.size();
            return QString("%1  (%2 worms)").arg(g.name).arg(n);
        }
        case Qt::FontRole: {
            QFont f;
            f.setBold(true);
            return f;
        }
        case Qt::CheckStateRole: {
            // Tristate across all worms in every video in this group
            int checked = 0, total = 0;
            for (const auto& v : g.videos)
                for (const auto& w : v.worms) { ++total; if (w.checked) ++checked; }
            if (total == 0 || checked == 0)    return Qt::Unchecked;
            if (checked == total)              return Qt::Checked;
            return Qt::PartiallyChecked;
        }
        default: return {};
        }
    }

    // ── Video node ──────────────────────────────────────────────────────────
    if (isVideo(idx)) {
        const int g = groupRowOf(idx);
        const int v = idx.row();
        if (g >= m_groups.size() || v >= m_groups[g].videos.size()) return {};
        const auto& vid = m_groups[g].videos[v];

        switch (role) {
        case Qt::DisplayRole:
            return QString("%1  [%2]").arg(vid.baseName, vid.procStamp);
        case Qt::ToolTipRole:
            return vid.warnings.isEmpty()
                ? QVariant()
                : QVariant(vid.warnings.join("\n"));
        case Qt::UserRole:   // WarningsRole — QStringList, consumed by the delegate
            return QVariant(vid.warnings);
        case Qt::CheckStateRole: {
            // Tristate: all checked → Checked, none → Unchecked, mixed → PartiallyChecked
            int checkedCount = 0;
            for (const auto& w : vid.worms) if (w.checked) ++checkedCount;
            if (checkedCount == 0)                       return Qt::Unchecked;
            if (checkedCount == vid.worms.size())        return Qt::Checked;
            return Qt::PartiallyChecked;
        }
        default: return {};
        }
    }

    // ── Worm node ───────────────────────────────────────────────────────────
    if (isWorm(idx)) {
        const int g = groupRowOf(idx);
        const int v = videoRowOf(idx);
        const int w = idx.row();
        if (g >= m_groups.size()
                || v >= m_groups[g].videos.size()
                || w >= m_groups[g].videos[v].worms.size())
            return {};
        const auto& worm = m_groups[g].videos[v].worms[w];

        switch (role) {
        case Qt::DisplayRole:    return worm.label;
        case Qt::DecorationRole: return makeColorIcon(worm.color);
        case Qt::CheckStateRole: return worm.checked ? Qt::Checked : Qt::Unchecked;
        default: return {};
        }
    }

    return {};
}

bool AnalysisSessionModel::setData(const QModelIndex& idx,
                                   const QVariant& value, int role)
{
    if (!idx.isValid()) return false;

    // ── Group rename ─────────────────────────────────────────────────────────
    if (isGroup(idx) && role == Qt::EditRole) {
        const QString name = value.toString().trimmed();
        if (name.isEmpty()) return false;
        m_groups[idx.row()].name = name;
        ++m_dataRevision;
        emit dataChanged(idx, idx, {Qt::DisplayRole, Qt::EditRole});
        saveState();
        return true;
    }

    if (role != Qt::CheckStateRole) return false;

    // ── Group node: cascade to all videos and all worms ──────────────────────
    if (isGroup(idx)) {
        const int g = idx.row();
        if (g >= m_groups.size()) return false;

        const bool checked = (value.toInt() != Qt::Unchecked);
        for (auto& vid : m_groups[g].videos)
            for (auto& w : vid.worms)
                w.checked = checked;

        // Notify the group node itself
        emit dataChanged(idx, idx, {Qt::CheckStateRole});

        // Notify each video child and its worms
        const int vidCount = m_groups[g].videos.size();
        for (int v = 0; v < vidCount; ++v) {
            const QModelIndex vidIdx = index(v, 0, idx);
            emit dataChanged(vidIdx, vidIdx, {Qt::CheckStateRole});
            const int wormCount = m_groups[g].videos[v].worms.size();
            if (wormCount > 0)
                emit dataChanged(index(0, 0, vidIdx),
                                 index(wormCount - 1, 0, vidIdx),
                                 {Qt::CheckStateRole});
        }
        ++m_checkRevision;
    emit checkedWormIdsChanged();
        scheduleStateSave();
        return true;
    }

    // ── Video node: cascade to all worms ────────────────────────────────────
    if (isVideo(idx)) {
        const int g = groupRowOf(idx);
        const int v = idx.row();
        if (g >= m_groups.size() || v >= m_groups[g].videos.size()) return false;

        // Treat PartiallyChecked clicks (from toggling a mixed state) as Checked
        const bool checked = (value.toInt() != Qt::Unchecked);
        auto& worms = m_groups[g].videos[v].worms;
        for (auto& w : worms) w.checked = checked;

        // Notify: the video node itself + all its worm children
        emit dataChanged(idx, idx, {Qt::CheckStateRole});
        if (!worms.isEmpty()) {
            emit dataChanged(index(0,         0, idx),
                             index(worms.size()-1, 0, idx),
                             {Qt::CheckStateRole});
        }
        // Refresh the parent group's tristate indicator
        const QModelIndex parentGroup = parent(idx);
        if (parentGroup.isValid())
            emit dataChanged(parentGroup, parentGroup, {Qt::CheckStateRole});

        ++m_checkRevision;
    emit checkedWormIdsChanged();
        scheduleStateSave();  // debounced check-state save
        return true;
    }

    // ── Worm node: single toggle ─────────────────────────────────────────────
    if (isWorm(idx)) {
        const int g = groupRowOf(idx);
        const int v = videoRowOf(idx);
        const int w = idx.row();
        if (g >= m_groups.size()
                || v >= m_groups[g].videos.size()
                || w >= m_groups[g].videos[v].worms.size())
            return false;

        m_groups[g].videos[v].worms[w].checked = (value.toInt() == Qt::Checked);
        emit dataChanged(idx, idx, {Qt::CheckStateRole});

        // Refresh the parent video node's tristate indicator
        const QModelIndex parentVideo = parent(idx);
        if (parentVideo.isValid()) {
            emit dataChanged(parentVideo, parentVideo, {Qt::CheckStateRole});
            // And the grandparent group node
            const QModelIndex parentGroup = parent(parentVideo);
            if (parentGroup.isValid())
                emit dataChanged(parentGroup, parentGroup, {Qt::CheckStateRole});
        }

        ++m_checkRevision;
    emit checkedWormIdsChanged();
        scheduleStateSave();  // debounced check-state save
        return true;
    }

    return false;
}

Qt::ItemFlags AnalysisSessionModel::flags(const QModelIndex& idx) const
{
    if (!idx.isValid()) return Qt::NoItemFlags;

    if (isGroup(idx))
        return Qt::ItemIsEnabled | Qt::ItemIsDropEnabled | Qt::ItemIsUserCheckable
             | Qt::ItemIsEditable;

    if (isVideo(idx))
        return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled
             | Qt::ItemIsUserCheckable;

    if (isWorm(idx))
        return Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;

    return Qt::NoItemFlags;
}

/** No-op override: Qt calls removeRows on the drag source after dropMimeData returns true.
 *  We have already moved the data inside dropMimeData, so we silently succeed here. */
bool AnalysisSessionModel::removeRows(int /*row*/, int /*count*/,
                                      const QModelIndex& /*parent*/)
{
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Drag-and-drop
// ─────────────────────────────────────────────────────────────────────────────

static const char* kMimeType = "application/x-yawt-video-node";

Qt::DropActions AnalysisSessionModel::supportedDropActions() const
{
    return Qt::MoveAction;
}

QStringList AnalysisSessionModel::mimeTypes() const
{
    return { QString(kMimeType) };
}

QMimeData* AnalysisSessionModel::mimeData(const QModelIndexList& indexes) const
{
    if (indexes.isEmpty()) return nullptr;

    // Only encode the first video-level index we find
    for (const QModelIndex& idx : indexes) {
        if (!isVideo(idx)) continue;

        auto* mime = new QMimeData();
        QByteArray encoded;
        QDataStream ds(&encoded, QIODevice::WriteOnly);
        ds << static_cast<qint32>(groupRowOf(idx)) << static_cast<qint32>(idx.row());
        mime->setData(kMimeType, encoded);
        return mime;
    }
    return nullptr;
}

bool AnalysisSessionModel::canDropMimeData(const QMimeData* data,
                                           Qt::DropAction /*action*/,
                                           int /*row*/, int /*column*/,
                                           const QModelIndex& parent) const
{
    if (!data->hasFormat(kMimeType)) return false;
    // Only allow drop onto a group node (not root, not video, not worm)
    return parent.isValid() && isGroup(parent);
}

bool AnalysisSessionModel::dropMimeData(const QMimeData* data,
                                        Qt::DropAction action,
                                        int /*row*/, int /*column*/,
                                        const QModelIndex& parent)
{
    if (!canDropMimeData(data, action, 0, 0, parent)) return false;

    QDataStream ds(data->data(kMimeType));
    qint32 srcGroupRow, srcVideoRow;
    ds >> srcGroupRow >> srcVideoRow;

    const int dstGroupRow = parent.row();

    if (srcGroupRow < 0 || srcGroupRow >= m_groups.size()) return false;
    if (srcVideoRow < 0 || srcVideoRow >= m_groups[srcGroupRow].videos.size()) return false;
    if (dstGroupRow < 0 || dstGroupRow >= m_groups.size()) return false;
    if (srcGroupRow == dstGroupRow) return false; // no-op: same group

    // Use targeted remove + insert so the view keeps its expansion state.
    // (beginResetModel would collapse every group node.)

    const QModelIndex srcGroupIdx = index(srcGroupRow, 0);
    const QModelIndex dstGroupIdx = index(dstGroupRow, 0);
    const int dstInsertRow = m_groups[dstGroupRow].videos.size(); // append to end

    // 1. Remove from source group
    beginRemoveRows(srcGroupIdx, srcVideoRow, srcVideoRow);
    VideoItem vid = m_groups[srcGroupRow].videos.takeAt(srcVideoRow);
    endRemoveRows();

    // 2. Insert into destination group
    beginInsertRows(dstGroupIdx, dstInsertRow, dstInsertRow);
    m_groups[dstGroupRow].videos.append(std::move(vid));
    endInsertRows();

    // 3. Recalculate colors for both groups and notify worm nodes of the change
    recalcGroupColors(srcGroupRow);
    recalcGroupColors(dstGroupRow);
    ++m_dataRevision;

    // Emit dataChanged for all worm children (colors changed) + group labels
    const auto notifyGroup = [this](int g) {
        const QModelIndex gIdx = index(g, 0);
        emit dataChanged(gIdx, gIdx, {Qt::DisplayRole, Qt::CheckStateRole});
        for (int v = 0; v < m_groups[g].videos.size(); ++v) {
            const QModelIndex vIdx = index(v, 0, gIdx);
            const int wc = m_groups[g].videos[v].worms.size();
            if (wc > 0)
                emit dataChanged(index(0, 0, vIdx), index(wc-1, 0, vIdx),
                                 {Qt::DecorationRole});
        }
    };
    notifyGroup(srcGroupRow);
    notifyGroup(dstGroupRow);

    saveState();
    return true;
}
