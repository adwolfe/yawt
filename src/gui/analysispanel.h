#ifndef ANALYSISPANEL_H
#define ANALYSISPANEL_H

#include <QObject>
#include <QWidget>
#include <QSet>
#include <QMap>
#include <QList>

class QComboBox;
class QSpinBox;
class QCheckBox;
class QDoubleSpinBox;
class QTreeView;
class QListWidget;
class QListWidgetItem;
class QMdiArea;
class QMdiSubWindow;
class QSplitter;
class QStandardItemModel;
class QStandardItem;
class QGroupBox;
class TrackingDataStorage;
class TrackXYPlotWidget;

namespace TableItems { struct ClickedItem; }

// Speed over time: one line per worm, X = frame, Y = speed
class SpeedTimelineWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SpeedTimelineWidget(TrackingDataStorage* storage, QWidget* parent = nullptr);
    void setPixelSizeUmPerPixel(double umPerPixel);
    void setVideoFps(double fps);
    void setVisibleWormIds(const QSet<int>& ids);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void refreshItems(const QList<TableItems::ClickedItem>& items);
    QColor colorForId(int id) const;

    TrackingDataStorage* m_storage = nullptr;
    double m_umPerPixel = 0.0;
    double m_videoFps = 0.0;
    QSet<int> m_visibleWormIds;
    QList<TableItems::ClickedItem> m_items;
};

// Average speed bar chart: one bar per worm
class AverageSpeedWidget : public QWidget
{
    Q_OBJECT
public:
    explicit AverageSpeedWidget(TrackingDataStorage* storage, QWidget* parent = nullptr);
    void setPixelSizeUmPerPixel(double umPerPixel);
    void setVideoFps(double fps);
    void setVisibleWormIds(const QSet<int>& ids);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void refreshItems(const QList<TableItems::ClickedItem>& items);
    QColor colorForId(int id) const;

    TrackingDataStorage* m_storage = nullptr;
    double m_umPerPixel = 0.0;
    double m_videoFps = 0.0;
    QSet<int> m_visibleWormIds;
    QList<TableItems::ClickedItem> m_items;
};

// Reversal count bar chart: counts direction reversals per worm
class ReversalCountWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ReversalCountWidget(TrackingDataStorage* storage, QWidget* parent = nullptr);
    void setPixelSizeUmPerPixel(double umPerPixel);
    void setVideoFps(double fps);
    void setVisibleWormIds(const QSet<int>& ids);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void refreshItems(const QList<TableItems::ClickedItem>& items);
    QColor colorForId(int id) const;

    TrackingDataStorage* m_storage = nullptr;
    double m_umPerPixel = 0.0;
    double m_videoFps = 0.0;
    QSet<int> m_visibleWormIds;
    QList<TableItems::ClickedItem> m_items;
};

/**
 * AnalysisPanel — controller for the Analysis tab.
 *
 * Not a widget itself; operates on widgets defined in mainwindow.ui.
 * The QMdiArea, worm list model, and plot sub-windows are still managed
 * here since they are dynamic/model-driven and can't live in a .ui file.
 *
 * Usage:
 *   m_analysisPanel = new AnalysisPanel(storage, this);
 *   m_analysisPanel->setup({ ui->analysisArenaShapeCombo, ... });
 */
class AnalysisPanel : public QObject
{
    Q_OBJECT
public:
    struct Widgets {
        QComboBox*       arenaShapeCombo    = nullptr;
        QSpinBox*        arenaSizeSpin      = nullptr;
        QCheckBox*       speedRangeCheck    = nullptr;
        QDoubleSpinBox*  speedRangeMinSpin  = nullptr;
        QDoubleSpinBox*  speedRangeMaxSpin  = nullptr;
        QTreeView*       wormListView       = nullptr;
        QListWidget*     plotSelector       = nullptr;
        QMdiArea*        mdiArea            = nullptr;
        QSplitter*       splitter           = nullptr;  ///< for setSizes on first show
    };

    explicit AnalysisPanel(TrackingDataStorage* storage, QObject* parent = nullptr);

    /** Wire the controller to the UI widgets. Call once, after setupUi(). */
    void setup(const Widgets& w);

    void setPixelSizeUmPerPixel(double umPerPixel);
    void setVideoFps(double fps);

public slots:
    void setSelectedWormIds(const QSet<int>& ids);
    void onStorageItemsChanged(const QList<TableItems::ClickedItem>& items);

signals:
    void wormSelectionChanged(const QSet<int>& ids);

private slots:
    void onWormItemChanged(QStandardItem* item);
    void onPlotItemChanged(QListWidgetItem* item);

private:
    void rebuildWormList(const QList<TableItems::ClickedItem>& items);
    void propagateSelectionToPlots();
    void propagateSettings(QWidget* w);
    void updateMdiLayout();
    QWidget* createPlotWidget(int plotIndex);
    static QIcon makeColorIcon(const QColor& color);

    // UI widget pointers (non-owning, sourced from mainwindow.ui)
    Widgets w;

    // Owned by this controller (not in the .ui file — dynamic/model-driven)
    QStandardItemModel* m_wormListModel = nullptr;
    QList<QMdiSubWindow*> m_subWindows;

    static constexpr int kPlotCount = 5;
    static const char* kPlotNames[kPlotCount];

    TrackingDataStorage* m_storage           = nullptr;
    QSet<int>            m_selectedWormIds;
    bool                 m_updatingSelection  = false;
    double               m_umPerPixel         = 0.0;
    double               m_videoFps           = 0.0;
};

#endif // ANALYSISPANEL_H
