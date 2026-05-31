#ifndef ANALYSISPANEL_H
#define ANALYSISPANEL_H

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

// Main analysis tab panel
class AnalysisPanel : public QWidget
{
    Q_OBJECT
public:
    explicit AnalysisPanel(TrackingDataStorage* storage, QWidget* parent = nullptr);
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
    void buildUi();
    void rebuildWormList(const QList<TableItems::ClickedItem>& items);
    void propagateSelectionToPlots();
    void propagateSettings(QWidget* w);
    void updateMdiLayout();
    QWidget* createPlotWidget(int plotIndex);
    static QIcon makeColorIcon(const QColor& color);

    // Left-pane settings
    QComboBox*       m_arenaShapeCombo     = nullptr;
    QSpinBox*        m_arenaSizeSpin       = nullptr;
    QCheckBox*       m_speedRangeCheck     = nullptr;
    QDoubleSpinBox*  m_speedRangeMinSpin   = nullptr;
    QDoubleSpinBox*  m_speedRangeMaxSpin   = nullptr;

    // Worm list
    QStandardItemModel* m_wormListModel = nullptr;
    QTreeView*          m_wormListView  = nullptr;

    // Plot selector
    QListWidget* m_plotSelector = nullptr;

    // MDI area
    QMdiArea* m_mdiArea = nullptr;

    // Active sub-windows (index matches m_plotSelector row)
    QList<QMdiSubWindow*> m_subWindows;

    static constexpr int kPlotCount = 5;
    static const char* kPlotNames[kPlotCount];

    TrackingDataStorage* m_storage        = nullptr;
    QSet<int>            m_selectedWormIds;
    bool                 m_updatingSelection = false;
    double               m_umPerPixel = 0.0;
    double               m_videoFps   = 0.0;
};

#endif // ANALYSISPANEL_H
