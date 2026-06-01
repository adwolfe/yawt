#ifndef ANALYSISGROUPWIDGETS_H
#define ANALYSISGROUPWIDGETS_H

/**
 * Group-aware analysis plot widgets for the Analysis tab MDI area.
 *
 * All widgets take an AnalysisSessionModel* and rebuild from getGroupedData()
 * whenever the model resets (group/video/worm structure changes).
 *
 * Widget summary:
 *   GroupTrackXYWidget       — individual worm paths coloured by group colour
 *   GroupSpeedTimelineWidget — per-group mean speed line over time
 *   GroupSpeedBoxWidget      — box-and-whisker: per-worm avg speed per group
 *   GroupReversalWidget      — bar chart: mean reversal count ± SD per group
 */

#include "analysissessionmodel.h"

#include <QList>
#include <QWidget>

class AnalysisSessionModel;

// ─────────────────────────────────────────────────────────────────────────────
// GroupTrackXYWidget
// Individual worm tracks from all groups; each worm line coloured by its
// group-colormap colour.  Has an optional speed-colour mode where each line
// segment is tinted by the worm's instantaneous speed.
// ─────────────────────────────────────────────────────────────────────────────
class GroupTrackXYWidget : public QWidget
{
    Q_OBJECT
public:
    enum class Mode { TrackColor, SpeedColor };

    explicit GroupTrackXYWidget(AnalysisSessionModel* model,
                                Mode mode = Mode::TrackColor,
                                QWidget* parent = nullptr);

    void setVideoFps(double fps);

public slots:
    void refreshData();

protected:
    void paintEvent(QPaintEvent*) override;

private:
    AnalysisSessionModel* m_model = nullptr;
    Mode   m_mode;
    double m_fps   = 0.0;
    QList<AnalysisSessionModel::AnalysisGroupData> m_data;
};

// ─────────────────────────────────────────────────────────────────────────────
// GroupSpeedTimelineWidget
// One line per group: the mean per-frame speed across all checked worms in
// that group.  X = time, Y = speed (µm/s if calibrated, else px/s).
// ─────────────────────────────────────────────────────────────────────────────
class GroupSpeedTimelineWidget : public QWidget
{
    Q_OBJECT
public:
    explicit GroupSpeedTimelineWidget(AnalysisSessionModel* model,
                                     QWidget* parent = nullptr);

    void setVideoFps(double fps);

public slots:
    void refreshData();

protected:
    void paintEvent(QPaintEvent*) override;

private:
    AnalysisSessionModel* m_model = nullptr;
    double m_fps = 0.0;
    QList<AnalysisSessionModel::AnalysisGroupData> m_data;
};

// ─────────────────────────────────────────────────────────────────────────────
// GroupSpeedBoxWidget
// Box-and-whisker plot (one box per group).
// Each data point is a single worm's average speed over its tracked frames.
// Box: Q1–Q3; centre line: median; whiskers: 1.5×IQR; dots: outliers.
// ─────────────────────────────────────────────────────────────────────────────
class GroupSpeedBoxWidget : public QWidget
{
    Q_OBJECT
public:
    explicit GroupSpeedBoxWidget(AnalysisSessionModel* model,
                                 QWidget* parent = nullptr);

    void setVideoFps(double fps);

public slots:
    void refreshData();

protected:
    void paintEvent(QPaintEvent*) override;

private:
    AnalysisSessionModel* m_model = nullptr;
    double m_fps = 0.0;
    QList<AnalysisSessionModel::AnalysisGroupData> m_data;
};

// ─────────────────────────────────────────────────────────────────────────────
// GroupReversalWidget
// Bar chart: one bar per group showing mean per-worm reversal count.
// Error bars show ±1 SD.  Individual data points are drawn as dots.
// ─────────────────────────────────────────────────────────────────────────────
class GroupReversalWidget : public QWidget
{
    Q_OBJECT
public:
    explicit GroupReversalWidget(AnalysisSessionModel* model,
                                 QWidget* parent = nullptr);

    void setVideoFps(double fps);  // kept for interface symmetry; unused here

public slots:
    void refreshData();

protected:
    void paintEvent(QPaintEvent*) override;

private:
    AnalysisSessionModel* m_model = nullptr;
    double m_fps = 0.0;  // unused currently
    QList<AnalysisSessionModel::AnalysisGroupData> m_data;
};

#endif // ANALYSISGROUPWIDGETS_H
