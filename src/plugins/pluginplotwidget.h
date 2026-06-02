#pragma once

#include "plotpluginspec.h"
#include "pluginengine.h"
#include "../gui/analysissessionmodel.h"

#include <QFutureWatcher>
#include <QString>
#include <QWidget>

/**
 * PluginPlotWidget — renders a single user-defined plugin plot.
 *
 * Evaluation runs on a background thread (QtConcurrent) so the GUI
 * never blocks. While computing, a "Computing…" label is shown.
 * Connects to the session model's checkedWormIdsChanged signal so it
 * refreshes automatically when the worm selection changes.
 */
class PluginPlotWidget : public QWidget
{
    Q_OBJECT
public:
    explicit PluginPlotWidget(const PlotPluginSpec& spec,
                              AnalysisSessionModel* model,
                              QWidget* parent = nullptr);
    ~PluginPlotWidget() override;

    void setRoiPoints(const PluginRoiPoints& roi);

public slots:
    void refreshData();

protected:
    void paintEvent(QPaintEvent*) override;

private slots:
    void onComputationFinished();

private:
    void paintBox(QPainter& p, const QRect& area);
    void paintBar(QPainter& p, const QRect& area);
    void paintLine(QPainter& p, const QRect& area);
    void paintStatus(QPainter& p, const QRect& area, const QString& msg);

    PlotPluginSpec            m_spec;
    AnalysisSessionModel*     m_model   = nullptr;
    PluginRoiPoints           m_roi;
    PluginEngine::PluginResult m_result;

    bool m_computing = false;
    bool m_pendingRefresh = false;  // another refresh queued while computing
    QString m_activeCacheKey;

    QFutureWatcher<PluginEngine::PluginResult> m_watcher;
};
