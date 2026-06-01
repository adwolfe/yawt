#include "analysissessionmodel.h"
#include "videometadatastore.h"

#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMimeData>
#include <QPixmap>

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
    QFile f(wormsJsonPath);
    if (!f.open(QIODevice::ReadOnly)) return {};

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
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
    QFile f(wormsJsonPath);
    if (!f.open(QIODevice::ReadOnly)) return tracks;

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
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
// Construction / population
// ─────────────────────────────────────────────────────────────────────────────

AnalysisSessionModel::AnalysisSessionModel(QObject* parent)
    : QAbstractItemModel(parent)
{
    // Always start with the "Unassigned" group
    m_groups.append(GroupItem{"Unassigned", {}});
}

void AnalysisSessionModel::scanYawtDirectory(const QString& yawtDir)
{
    beginResetModel();

    // Keep any user-created groups but clear their video lists first —
    // we are about to re-populate from disk.  Then rebuild "Unassigned".
    // For simplicity: preserve group names but clear all video data,
    // then re-populate Unassigned from disk.
    const QStringList preservedGroupNames = [&]{
        QStringList n;
        for (int i = 1; i < m_groups.size(); ++i) // skip index 0 (Unassigned)
            n << m_groups[i].name;
        return n;
    }();

    m_groups.clear();
    m_groups.append(GroupItem{"Unassigned", {}});
    for (const QString& name : preservedGroupNames)
        m_groups.append(GroupItem{name, {}});

    // Scan: each immediate subdirectory of yawtDir is a video basename
    const QStringList videoDirs = QDir(yawtDir).entryList(
        QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);

    for (const QString& videoBaseName : videoDirs) {
        const QString videoSubDir = QDir(yawtDir).absoluteFilePath(videoBaseName);
        const QString procDir     = findMostRecentProc(videoSubDir);
        if (procDir.isEmpty()) continue;

        const QString wormsJson = QDir(procDir).absoluteFilePath("worms.json");
        const QList<int> wormIds = parseWormIds(wormsJson);
        if (wormIds.isEmpty()) continue;

        VideoItem vid;
        vid.baseName  = videoBaseName;
        vid.procDir   = procDir;
        vid.procStamp = QFileInfo(procDir).fileName().mid(5); // strip "PROC_"

        // Load full track data and scale calibration for analysis plots
        vid.tracks = loadTracksFromJson(wormsJson);
        VideoMetadataStore::loadUmPerPixel(yawtDir, videoBaseName, vid.umPerPixel);

        for (int i = 0; i < wormIds.size(); ++i) {
            WormItem w;
            w.id      = wormIds[i];
            w.checked = true;
            w.color   = Qt::gray;          // recalculated below
            w.label   = QString("Worm %1").arg(i + 1);
            vid.worms.append(w);
        }

        m_groups[0].videos.append(std::move(vid));
    }

    recalcGroupColors(0);
    endResetModel();
}

void AnalysisSessionModel::addGroup(const QString& name)
{
    if (name.trimmed().isEmpty()) return;
    const int row = m_groups.size();
    beginInsertRows({}, row, row);
    m_groups.append(GroupItem{name, {}});
    endInsertRows();
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
    emit checkedWormIdsChanged();
}

void AnalysisSessionModel::setCheckedWormIds(const QSet<int>& ids)
{
    for (auto& g : m_groups)
        for (auto& v : g.videos)
            for (auto& w : v.worms)
                w.checked = ids.contains(w.id);

    emit dataChanged(index(0,0), index(m_groups.size()-1, 0));
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
            for (const auto& worm : vid.worms) {
                if (!worm.checked) continue;

                const auto it = vid.tracks.find(worm.id);
                if (it == vid.tracks.end() || it->second.empty()) continue;

                AnalysisWormEntry entry;
                entry.wormId       = worm.id;
                entry.label        = worm.label;
                entry.color        = worm.color;
                entry.umPerPixel   = vid.umPerPixel;
                entry.videoBaseName = vid.baseName;
                entry.points       = it->second;  // full copy, sorted in loadTracksFromJson
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
    if (!idx.isValid() || !isWorm(idx)) return false;
    if (role != Qt::CheckStateRole) return false;

    const int g = groupRowOf(idx);
    const int v = videoRowOf(idx);
    const int w = idx.row();
    if (g >= m_groups.size()
            || v >= m_groups[g].videos.size()
            || w >= m_groups[g].videos[v].worms.size())
        return false;

    m_groups[g].videos[v].worms[w].checked = (value.toInt() == Qt::Checked);
    emit dataChanged(idx, idx, {Qt::CheckStateRole});
    emit checkedWormIdsChanged();
    return true;
}

Qt::ItemFlags AnalysisSessionModel::flags(const QModelIndex& idx) const
{
    if (!idx.isValid()) return Qt::NoItemFlags;

    if (isGroup(idx))
        return Qt::ItemIsEnabled | Qt::ItemIsDropEnabled;

    if (isVideo(idx))
        return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled;

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

    // Move video — use beginResetModel so all persistent indexes stay valid
    beginResetModel();
    VideoItem vid = m_groups[srcGroupRow].videos.takeAt(srcVideoRow);
    m_groups[dstGroupRow].videos.append(std::move(vid));
    recalcGroupColors(srcGroupRow);
    recalcGroupColors(dstGroupRow);
    endResetModel();

    // Return true so Qt's InternalMove/DragDrop machinery knows we handled it;
    // our removeRows no-op prevents double-deletion.
    return true;
}
