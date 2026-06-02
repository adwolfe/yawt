#ifndef ANALYSISSESSIONMODEL_H
#define ANALYSISSESSIONMODEL_H

#include "trackingcommon.h"   // Tracking::AllWormTracks, WormTrackPoint

#include <QAbstractItemModel>
#include <QColor>
#include <QIcon>
#include <QList>
#include <QMimeData>
#include <QPointF>
#include <QSet>
#include <QString>
#include <QTimer>

/**
 * AnalysisSessionModel — 3-level tree model for the Analysis tab worm list.
 *
 * Tree structure:
 *   Root
 *   ├── Group  "Unassigned"       (level 0; always row 0)
 *   │   ├── Video  "basename  [PROC_...]"  (level 1; draggable)
 *   │   │   ├── ☑ Worm 1          (level 2; checkable, colored icon)
 *   │   │   └── ...
 *   │   └── ...
 *   └── Group  "Wild Type"        (level 0; user-created)
 *       └── ...
 *
 * internalId encoding (quintptr == 64-bit on macOS):
 *   Group nodes : UINT64_MAX                              (sentinel; parent is root)
 *   Video nodes : (quintptr)groupRow                     (parent is that group)
 *   Worm  nodes : WORM_FLAG | (groupRow<<16) | videoRow  (parent is that video)
 *
 * IMPORTANT: check `id == UINT64_MAX` BEFORE testing bit 63, because UINT64_MAX
 * has ALL bits set (including bit 63) and would otherwise be misidentified as a worm node.
 *
 * Colors are assigned per-group via an HSV colormap spread across all worms in the group.
 * Recalculated whenever videos move between groups.
 */
class AnalysisSessionModel : public QAbstractItemModel
{
    Q_OBJECT

public:
    // ── Data structures ──────────────────────────────────────────────────────
    struct WormItem {
        int     id      = 0;
        bool    checked = true;
        QColor  color;
        QString label;   // "Worm 1", "Worm 2", …
    };

    struct VideoItem {
        QString       baseName;    // video filename without extension
        QString       procDir;     // absolute path to PROC_* folder
        QString       procStamp;   // the "yyyy-MM-dd-HHmmss" part of the folder name
        QList<WormItem> worms;

        // Track data loaded from worms.json — available for analysis plots
        Tracking::AllWormTracks tracks;
        double umPerPixel = 0.0;   // µm/pixel from _metadata.json (0 if unknown)
        double fps        = 0.0;   // frames/s from _metadata.json (0 if unknown)

        // Validation warnings (empty = no issues)
        QStringList warnings;

        bool hasStartPoint = false;
        QPointF startPoint;
        bool hasEndPoint = false;
        QPointF endPoint;
        bool hasCenterPoint = false;
        QPointF centerPoint;
    };

    // ── Grouped data structs (returned by getGroupedData) ─────────────────────
    /** One worm's complete data as seen by the analysis plots. */
    struct AnalysisWormEntry {
        int     wormId;
        QString label;         // "Worm 1" …
        QColor  color;         // group colormap color
        double  umPerPixel;    // from the video this worm belongs to
        double  fps;           // from the video this worm belongs to (0 if unknown)
        QString videoBaseName;
        std::vector<Tracking::WormTrackPoint> points;  // sorted by frame

        bool hasStartPoint = false;
        QPointF startPoint;
        bool hasEndPoint = false;
        QPointF endPoint;
        bool hasCenterPoint = false;
        QPointF centerPoint;
    };

    /** All checked worms that belong to one group. */
    struct AnalysisGroupData {
        QString name;
        QList<AnalysisWormEntry> worms;
    };

    /**
     * Build a snapshot of all checked worms organised by group.
     * Groups with no checked worms are omitted.
     * Track points are sorted by frameNumberOriginal.
     * Call this from paintEvent; the data is a deep copy (safe after model resets).
     */
    QList<AnalysisGroupData> getGroupedData() const;

    struct GroupItem {
        QString          name;
        QList<VideoItem> videos;
    };

    // ── Construction / population ─────────────────────────────────────────────
    explicit AnalysisSessionModel(QObject* parent = nullptr);

    /** Scan a yawt directory, populate "Unassigned" with all found proc runs. */
    void scanYawtDirectory(const QString& yawtDir);

    /** Add a user-named group at the end of the group list. */
    void addGroup(const QString& name);

    /** Set checked state for all worms in the given proc directory. */
    void setCheckedForProcDir(const QString& procDir, bool checked);

    /** Set checked state for specific worm IDs across all groups/videos. */
    void setCheckedWormIds(const QSet<int>& ids);

    /** Return the set of all currently checked worm IDs (across all groups). */
    QSet<int> checkedWormIds() const;

    // ── QAbstractItemModel interface ──────────────────────────────────────────
    QModelIndex index(int row, int column,
                      const QModelIndex& parent = {}) const override;
    QModelIndex parent(const QModelIndex& child) const override;
    int  rowCount  (const QModelIndex& parent = {}) const override;
    int  columnCount(const QModelIndex& parent = {}) const override;
    QVariant     data (const QModelIndex& index, int role = Qt::DisplayRole) const override;
    bool         setData(const QModelIndex& index, const QVariant& value,
                         int role = Qt::EditRole) override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    bool removeRows(int row, int count, const QModelIndex& parent = {}) override;

    // ── Drag-and-drop ─────────────────────────────────────────────────────────
    Qt::DropActions supportedDropActions() const override;
    QStringList     mimeTypes() const override;
    QMimeData*      mimeData(const QModelIndexList& indexes) const override;
    bool canDropMimeData(const QMimeData* data, Qt::DropAction action,
                         int row, int column,
                         const QModelIndex& parent) const override;
    bool dropMimeData(const QMimeData* data, Qt::DropAction action,
                      int row, int column,
                      const QModelIndex& parent) override;

signals:
    /** Emitted whenever any worm's checked state changes. */
    void checkedWormIdsChanged();

private:
    // ── Internal helpers ──────────────────────────────────────────────────────
    static constexpr quintptr kGroupSentinel = ~quintptr(0); // UINT64_MAX
    static constexpr quintptr kWormFlag      = quintptr(1) << 63;

    // Level detection — must check sentinel first (it has all bits set)
    static bool isGroup(const QModelIndex& i)
        { return i.isValid() && i.internalId() == kGroupSentinel; }
    static bool isWorm(const QModelIndex& i)
        { return i.isValid() && i.internalId() != kGroupSentinel
                             && (i.internalId() & kWormFlag); }
    static bool isVideo(const QModelIndex& i)
        { return i.isValid() && i.internalId() != kGroupSentinel
                             && !(i.internalId() & kWormFlag); }

    // Decode indices from internalId
    static int groupRowOf(const QModelIndex& i) {
        if (isGroup(i))  return i.row();
        if (isVideo(i))  return static_cast<int>(i.internalId());
        // worm node: bits 16..31
        return static_cast<int>((i.internalId() >> 16) & 0xFFFF);
    }
    static int videoRowOf(const QModelIndex& i) {
        if (isWorm(i))  return static_cast<int>(i.internalId() & 0xFFFF);
        if (isVideo(i)) return i.row();
        return -1;
    }

    void recalcGroupColors(int groupRow);
    static QList<int>                parseWormIds(const QString& wormsJsonPath);
    static Tracking::AllWormTracks   loadTracksFromJson(const QString& wormsJsonPath);
    static QString                   findMostRecentProc(const QString& videoSubDir);
    static QStringList               buildWarnings(const QString& procDir,
                                                   const QString& yawtDir,
                                                   const QString& baseName,
                                                   double umPerPixel);
    static void                      loadRoiReferencePoints(VideoItem& vid);
    static QColor     colormapColor(int index, int total);
    static QIcon      makeColorIcon(const QColor& c);

    // Persistence helpers
    static QString    stateFilePath(const QString& yawtDir);
    void              saveState() const;
    void              loadAndMergeState(
                          const QString& yawtDir,
                          const QMap<QString, QPair<QString,QString>>& diskVideos);
    void              scheduleStateSave();   // debounced, for check-state changes

    QList<GroupItem> m_groups;
    QString          m_yawtDir;              // set during scan, used for auto-save
    QTimer*          m_saveTimer = nullptr;  // debounce timer for check-state saves
};

#endif // ANALYSISSESSIONMODEL_H
