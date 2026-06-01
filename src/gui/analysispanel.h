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
class QPushButton;
class QTreeView;
class QListWidget;
class QListWidgetItem;
class QMdiArea;
class QMdiSubWindow;
class QSplitter;
class AnalysisSessionModel;
class TrackingDataStorage;

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
        QPushButton*     addGroupBtn        = nullptr;
        QListWidget*     plotSelector       = nullptr;
        QMdiArea*        mdiArea            = nullptr;
        QSplitter*       splitter           = nullptr;  ///< for setSizes on first show
    };

    explicit AnalysisPanel(TrackingDataStorage* storage, QObject* parent = nullptr);

    /** Wire the controller to the UI widgets. Call once, after setupUi(). */
    void setup(const Widgets& w);

    void setPixelSizeUmPerPixel(double umPerPixel);
    void setVideoFps(double fps);

    /** Scan (or re-scan) all proc runs found under the given yawt directory. */
    void setYawtDirectory(const QString& yawtDir);

public slots:
    void setSelectedWormIds(const QSet<int>& ids);

signals:
    void wormSelectionChanged(const QSet<int>& ids);

private slots:
    void onSessionCheckedChanged();
    void onPlotItemChanged(QListWidgetItem* item);

private:
    void propagateSelectionToPlots();
    void propagateSettings(QWidget* pw);
    void updateMdiLayout();
    QWidget* createPlotWidget(int plotIndex);

    // UI widget pointers (non-owning, sourced from mainwindow.ui)
    Widgets w;

    // Owned by this controller (not in the .ui file — dynamic/model-driven)
    AnalysisSessionModel* m_sessionModel = nullptr;
    QList<QMdiSubWindow*> m_subWindows;

    static constexpr int kPlotCount = 5;
    static const char* kPlotNames[kPlotCount];

    TrackingDataStorage* m_storage      = nullptr;
    QSet<int>            m_selectedWormIds;
    double               m_umPerPixel  = 0.0;
    double               m_videoFps    = 0.0;
    QString              m_yawtDir;
};

#endif // ANALYSISPANEL_H
