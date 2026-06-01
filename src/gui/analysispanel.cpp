#include "analysispanel.h"
#include "analysissessionmodel.h"
#include "analysisgroupwidgets.h"
#include "trackingdatastorage.h"
#include "../plugins/pluginloader.h"
#include "../plugins/pluginplotwidget.h"
#include "../utils/yawtpaths.h"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QHelpEvent>
#include <QInputDialog>
#include <QListWidget>
#include <QMdiArea>
#include <QMdiSubWindow>
#include <QPainter>
#include <QPushButton>
#include <QSpinBox>
#include <QSplitter>
#include <QStyledItemDelegate>
#include <QToolTip>
#include <QTreeView>
#include <QHeaderView>

// ─────────────────────────────────────────────────────────────────────────────
// AnalysisWarningDelegate — paints a red "!" badge at the right of video nodes
// that have validation warnings, and surfaces the tooltip when hovered.
// ─────────────────────────────────────────────────────────────────────────────
class AnalysisWarningDelegate : public QStyledItemDelegate
{
    Q_OBJECT
public:
    explicit AnalysisWarningDelegate(QObject* parent = nullptr)
        : QStyledItemDelegate(parent) {}

    void paint(QPainter* p, const QStyleOptionViewItem& opt,
               const QModelIndex& idx) const override
    {
        QStyledItemDelegate::paint(p, opt, idx);

        const QStringList warnings = idx.data(Qt::UserRole).toStringList();
        if (warnings.isEmpty()) return;

        const QRect badge = badgeRect(opt);
        p->save();
        p->setRenderHint(QPainter::Antialiasing, true);

        // Red circle
        p->setPen(Qt::NoPen);
        p->setBrush(QColor(0xD0, 0x20, 0x20));
        p->drawEllipse(badge);

        // White "!"
        p->setPen(Qt::white);
        QFont f = p->font();
        f.setBold(true);
        f.setPixelSize(badge.height() - 4);
        p->setFont(f);
        p->drawText(badge, Qt::AlignCenter, "!");

        p->restore();
    }

    bool helpEvent(QHelpEvent* ev, QAbstractItemView* view,
                   const QStyleOptionViewItem& opt,
                   const QModelIndex& idx) override
    {
        const QStringList warnings = idx.data(Qt::UserRole).toStringList();
        if (!warnings.isEmpty() && badgeRect(opt).contains(ev->pos())) {
            QToolTip::showText(ev->globalPos(),
                               "<b>Missing data:</b><br>" +
                               warnings.join("<br>"),
                               view);
            return true;
        }
        QToolTip::hideText();
        return QStyledItemDelegate::helpEvent(ev, view, opt, idx);
    }

private:
    static QRect badgeRect(const QStyleOptionViewItem& opt)
    {
        constexpr int kSize   = 14;
        constexpr int kMargin = 4;
        const int x = opt.rect.right() - kSize - kMargin;
        const int y = opt.rect.top() + (opt.rect.height() - kSize) / 2;
        return QRect(x, y, kSize, kSize);
    }
};

#include "analysispanel.moc"   // needed because the class is defined in a .cpp

// ─────────────────────────────────────────────────────────────────────────────
// AnalysisPanel
// ─────────────────────────────────────────────────────────────────────────────

const char* AnalysisPanel::kPlotNames[AnalysisPanel::kPlotCount] = {
    "Tracks (by worm)",
    "Tracks (speed-coloured)",
    "Speed Timeline",
    "Speed Distribution",
    "Reversals"
};

AnalysisPanel::AnalysisPanel(TrackingDataStorage* storage, QObject* parent)
    : QObject(parent), m_storage(storage)
{
    // The Analysis tab is intentionally decoupled from the Processing tab's
    // real-time storage signals.  The session model is populated from disk only
    // at well-defined moments (setYawtDirectory), never on every worm-click.
}

void AnalysisPanel::setup(const Widgets& widgets)
{
    w = widgets;

    // Splitter sizing
    if (w.splitter) {
        w.splitter->setStretchFactor(0, 0);
        w.splitter->setStretchFactor(1, 1);
        w.splitter->setSizes({220, 600});
    }

    // Speed range spinbox limits
    if (w.speedRangeMinSpin) { w.speedRangeMinSpin->setRange(0.0, 1e6); }
    if (w.speedRangeMaxSpin) { w.speedRangeMaxSpin->setRange(0.0, 1e6); w.speedRangeMaxSpin->setValue(1.0); }

    // Session model — owns all group/video/worm data
    m_sessionModel = new AnalysisSessionModel(this);

    if (w.wormListView) {
        w.wormListView->setModel(m_sessionModel);
        w.wormListView->setHeaderHidden(true);
        w.wormListView->setRootIsDecorated(true);
        w.wormListView->setSelectionMode(QAbstractItemView::SingleSelection);
        w.wormListView->setDragEnabled(true);
        w.wormListView->setAcceptDrops(true);
        w.wormListView->setDragDropMode(QAbstractItemView::DragDrop);
        w.wormListView->setDefaultDropAction(Qt::MoveAction);
        w.wormListView->setDropIndicatorShown(true);
        w.wormListView->setMouseTracking(true);   // needed for hover tooltips
        w.wormListView->setItemDelegate(new AnalysisWarningDelegate(w.wormListView));
    }

    // ↻ button → force rescan of the yawt directory
    if (w.refreshBtn) {
        connect(w.refreshBtn, &QPushButton::clicked, this, [this]() {
            if (!m_yawtDir.isEmpty())
                setYawtDirectory(m_yawtDir);
        });
    }

    // "+" button → add a named group
    if (w.addGroupBtn) {
        connect(w.addGroupBtn, &QPushButton::clicked, this, [this]() {
            if (!m_sessionModel) return;
            const QString name = QInputDialog::getText(
                nullptr, "New Group", "Group name:");
            if (!name.trimmed().isEmpty()) {
                m_sessionModel->addGroup(name.trimmed());
                // Expand the new group immediately
                if (w.wormListView) {
                    const QModelIndex newGroup = m_sessionModel->index(
                        m_sessionModel->rowCount() - 1, 0);
                    w.wormListView->expand(newGroup);
                }
            }
        });
    }

    // When any worm's check state changes, forward to plots
    connect(m_sessionModel, &AnalysisSessionModel::checkedWormIdsChanged,
            this, &AnalysisPanel::onSessionCheckedChanged);

    // Plot selector
    if (w.plotSelector) {
        for (int i = 0; i < kPlotCount; ++i) {
            auto* item = new QListWidgetItem(kPlotNames[i], w.plotSelector);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(Qt::Unchecked);
        }
        connect(w.plotSelector, &QListWidget::itemChanged,
                this, &AnalysisPanel::onPlotItemChanged);
    }

    m_subWindows.resize(kPlotCount, nullptr);

    // Settings → propagate to open plots
    auto propagateAll = [this]() {
        if (!w.mdiArea) return;
        for (auto* sw : w.mdiArea->subWindowList())
            if (auto* pw = sw->widget()) propagateSettings(pw);
    };
    if (w.arenaShapeCombo)
        connect(w.arenaShapeCombo, qOverload<int>(&QComboBox::currentIndexChanged),
                this, propagateAll);
    if (w.arenaSizeSpin)
        connect(w.arenaSizeSpin, qOverload<int>(&QSpinBox::valueChanged),
                this, propagateAll);
    if (w.speedRangeCheck)
        connect(w.speedRangeCheck, &QCheckBox::toggled, this, propagateAll);
    if (w.speedRangeMinSpin)
        connect(w.speedRangeMinSpin, qOverload<double>(&QDoubleSpinBox::valueChanged),
                this, propagateAll);
    if (w.speedRangeMaxSpin)
        connect(w.speedRangeMaxSpin, qOverload<double>(&QDoubleSpinBox::valueChanged),
                this, propagateAll);
}

void AnalysisPanel::setYawtDirectory(const QString& yawtDir)
{
    if (yawtDir.isEmpty() || !m_sessionModel) return;
    m_yawtDir = yawtDir;
    m_sessionModel->scanYawtDirectory(yawtDir);
    if (w.wormListView) {
        w.wormListView->collapseAll();
        const int groupCount = m_sessionModel->rowCount();
        for (int i = 0; i < groupCount; ++i)
            w.wormListView->expand(m_sessionModel->index(i, 0));
    }
    loadPlugins();
}

void AnalysisPanel::loadPlugins()
{
    if (!w.plotSelector) return;

    // Close and remove any existing plugin sub-windows
    for (auto* sw : m_pluginSubWindows.values()) {
        if (sw && w.mdiArea) w.mdiArea->removeSubWindow(sw);
        if (sw) sw->deleteLater();
    }
    m_pluginSubWindows.clear();

    // Remove existing plugin items from the selector (keep built-ins at top)
    while (w.plotSelector->count() > kPlotCount)
        delete w.plotSelector->takeItem(kPlotCount);

    // Load plugins from all search dirs (project > user > bundled)
    const QStringList searchDirs = YawtPaths::pluginSearchDirs(m_yawtDir);
    m_plugins = PluginLoader::loadAll(searchDirs);

    // Add valid plugins to the selector with a separator label first
    bool separatorAdded = false;
    for (const PlotPluginSpec& spec : m_plugins) {
        if (!spec.isValid) continue;
        if (!separatorAdded) {
            auto* sep = new QListWidgetItem("── Plugins ──", w.plotSelector);
            sep->setFlags(Qt::NoItemFlags);
            sep->setForeground(QApplication::palette().mid());
            separatorAdded = true;
        }
        auto* item = new QListWidgetItem(spec.name, w.plotSelector);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);
        if (!spec.description.isEmpty())
            item->setToolTip(spec.description);
    }
}

void AnalysisPanel::setPixelSizeUmPerPixel(double v)
{
    m_umPerPixel = v;
    if (!w.mdiArea) return;
    for (auto* sw : w.mdiArea->subWindowList())
        if (auto* pw = sw->widget()) propagateSettings(pw);
}

void AnalysisPanel::setVideoFps(double v)
{
    m_videoFps = v;
    if (!w.mdiArea) return;
    for (auto* sw : w.mdiArea->subWindowList())
        if (auto* pw = sw->widget()) propagateSettings(pw);
}

void AnalysisPanel::setSelectedWormIds(const QSet<int>& ids)
{
    if (!m_sessionModel) return;
    m_selectedWormIds = ids;
    m_sessionModel->setCheckedWormIds(ids);
    // checkedWormIdsChanged → onSessionCheckedChanged will propagate to plots
}


void AnalysisPanel::onSessionCheckedChanged()
{
    if (!m_sessionModel) return;
    m_selectedWormIds = m_sessionModel->checkedWormIds();
    propagateSelectionToPlots();
    emit wormSelectionChanged(m_selectedWormIds);
}

void AnalysisPanel::onPlotItemChanged(QListWidgetItem* item)
{
    if (!w.plotSelector || !w.mdiArea) return;
    const int idx = w.plotSelector->row(item);
    if (idx < 0) return;

    // ── Built-in plots ─────────────────────────────────────────────────────
    if (idx < kPlotCount) {
        if (item->checkState() == Qt::Checked) {
            QWidget* plotWidget = createPlotWidget(idx);
            if (!plotWidget) return;
            auto* sw = w.mdiArea->addSubWindow(plotWidget);
            sw->setWindowTitle(kPlotNames[idx]);
            sw->show();
            m_subWindows[idx] = sw;
        } else {
            if (m_subWindows[idx]) {
                w.mdiArea->removeSubWindow(m_subWindows[idx]);
                m_subWindows[idx]->deleteLater();
                m_subWindows[idx] = nullptr;
            }
        }
        updateMdiLayout();
        return;
    }

    // ── Plugin plots ───────────────────────────────────────────────────────
    // Find which plugin this item corresponds to (skip separator items)
    int pluginIdx = -1;
    int pluginCounter = 0;
    for (int i = kPlotCount; i < w.plotSelector->count(); ++i) {
        QListWidgetItem* it = w.plotSelector->item(i);
        if (!(it->flags() & Qt::ItemIsUserCheckable)) continue; // separator
        if (it == item) { pluginIdx = pluginCounter; break; }
        ++pluginCounter;
    }
    if (pluginIdx < 0 || pluginIdx >= m_plugins.size()) return;
    const PlotPluginSpec& spec = m_plugins[pluginIdx];

    if (item->checkState() == Qt::Checked) {
        if (m_pluginSubWindows.contains(item)) return; // already open
        auto* pw = new PluginPlotWidget(spec, m_sessionModel);
        auto* sw = w.mdiArea->addSubWindow(pw);
        sw->setWindowTitle(spec.name);
        sw->show();
        m_pluginSubWindows[item] = sw;
    } else {
        if (m_pluginSubWindows.contains(item)) {
            w.mdiArea->removeSubWindow(m_pluginSubWindows[item]);
            m_pluginSubWindows[item]->deleteLater();
            m_pluginSubWindows.remove(item);
        }
    }
    updateMdiLayout();
}

void AnalysisPanel::propagateSelectionToPlots()
{
    // Selection is driven by AnalysisSessionModel; individual plot widgets
    // update themselves when the model signals a change.
}

void AnalysisPanel::propagateSettings(QWidget* pw)
{
    if (!pw) return;
    // All group widgets share the same minimal interface.
    if (auto* p = qobject_cast<GroupTrackXYWidget*>(pw))
        p->setVideoFps(m_videoFps);
    else if (auto* p = qobject_cast<GroupSpeedTimelineWidget*>(pw))
        p->setVideoFps(m_videoFps);
    else if (auto* p = qobject_cast<GroupSpeedBoxWidget*>(pw))
        p->setVideoFps(m_videoFps);
    else if (auto* p = qobject_cast<GroupReversalWidget*>(pw))
        p->setVideoFps(m_videoFps);
}

void AnalysisPanel::updateMdiLayout()
{
    if (w.mdiArea) w.mdiArea->tileSubWindows();
}

QWidget* AnalysisPanel::createPlotWidget(int plotIndex)
{
    QWidget* pw = nullptr;
    switch (plotIndex) {
    case 0: pw = new GroupTrackXYWidget(m_sessionModel, GroupTrackXYWidget::Mode::TrackColor);  break;
    case 1: pw = new GroupTrackXYWidget(m_sessionModel, GroupTrackXYWidget::Mode::SpeedColor);  break;
    case 2: pw = new GroupSpeedTimelineWidget(m_sessionModel);                                  break;
    case 3: pw = new GroupSpeedBoxWidget(m_sessionModel);                                       break;
    case 4: pw = new GroupReversalWidget(m_sessionModel);                                       break;
    default: return nullptr;
    }
    propagateSettings(pw);
    return pw;
}
