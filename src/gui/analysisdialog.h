#ifndef ANALYSISDIALOG_H
#define ANALYSISDIALOG_H

#include <QColor>
#include <QDialog>
#include <QList>
#include <QWidget>

class QTabWidget;
class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;
class TrackingDataStorage;

namespace TableItems {
struct ClickedItem;
}

class TrackXYPlotWidget : public QWidget
{
    Q_OBJECT

public:
    enum class PlotMode {
        TrackColor,
        SpeedColor
    };

    explicit TrackXYPlotWidget(TrackingDataStorage *storage,
                               PlotMode mode = PlotMode::TrackColor,
                               QWidget *parent = nullptr);
    void setArenaShape(int index);
    void setArenaSizeMm(int sizeMm);
    void setPixelSizeUmPerPixel(double umPerPixel);
    void setVideoFps(double fps);
    void setSpeedLutRange(bool enabled, double minSpeed, double maxSpeed);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    enum class ArenaShape {
        Round,
        Square
    };

    void refreshItems(const QList<TableItems::ClickedItem> &items);
    QColor colorForItem(int itemId) const;

    TrackingDataStorage *m_storage = nullptr;
    QList<TableItems::ClickedItem> m_items;
    ArenaShape m_arenaShape = ArenaShape::Round;
    double m_arenaSizeMm = 50.0;
    double m_umPerPixel = 0.0;
    double m_videoFps = 0.0;
    PlotMode m_plotMode = PlotMode::TrackColor;
    bool m_speedRangeEnabled = false;
    double m_speedRangeMin = 0.0;
    double m_speedRangeMax = 0.0;
};

class AnalysisDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AnalysisDialog(TrackingDataStorage *storage, QWidget *parent = nullptr);
    void setPixelSizeUmPerPixel(double umPerPixel);
    void setVideoFps(double fps);

private:
    QWidget *buildSetupTab();
    void syncTrackWidgetSetup(TrackXYPlotWidget *widget);

    QTabWidget *m_tabWidget = nullptr;
    QComboBox *m_arenaShapeCombo = nullptr;
    QSpinBox *m_arenaSizeSpin = nullptr;
    QCheckBox *m_speedRangeCheck = nullptr;
    QDoubleSpinBox *m_speedRangeMinSpin = nullptr;
    QDoubleSpinBox *m_speedRangeMaxSpin = nullptr;
    TrackXYPlotWidget *m_tracksWidget = nullptr;
    TrackXYPlotWidget *m_speedTracksWidget = nullptr;
};

#endif // ANALYSISDIALOG_H
