// trackingprogressdialog.h
#ifndef TRACKINGPROGRESSDIALOG_H
#define TRACKINGPROGRESSDIALOG_H

#include <QDialog>
#include <QString>
#include <vector>
#include "trackingcommon.h" // Contains Thresholding::ThresholdAlgorithm definition

// Forward declarations
namespace Ui {
class TrackingProgressDialog;
}
class QProgressBar;
class QLabel;
class QPushButton;

class TrackingProgressDialog : public QDialog {
    Q_OBJECT
    // Make the globally defined Thresholding::ThresholdAlgorithm known to this class's meta-object system
    // This allows us to use QMetaEnum to get string representations.
    //Q_ENUM(Thresholding::ThresholdAlgorithm)

public:
    explicit TrackingProgressDialog(QWidget *parent = nullptr);
    ~TrackingProgressDialog();

    // numberOfWormsWithTracks: optional count of how many of the provided items
    // already have track data associated with them (default 0 for backward compatibility).
    void setTrackingParameters(const QString& videoPath,
                               int keyFrame,
                               const Thresholding::ThresholdSettings& settings,
                               int numberOfWorms,
                               int totalFramesInVideo,
                               int numberOfWormsWithTracks = 0);

    // Returns whether the "Only track missing worms" checkbox is checked in the dialog.
    // If true, callers should filter the provided worm list to include only items
    // that do not already have existing track data.
    bool onlyTrackMissingChecked() const;

public slots:
    void updateStatusMessage(const QString& message);
    void updateOverallProgress(int percentage);
    void onTrackingSuccessfullyFinished();
    void onTrackingFailed(const QString& reason);
    void onTrackingCancelledByManager();

signals:
    void beginTrackingRequested();
    void cancelTrackingRequested();

private slots:
    void onBeginButtonClicked();
    void onCancelButtonClicked();

private:
    Ui::TrackingProgressDialog *ui;

    QString m_videoPath;
    int m_keyFrame;
    Thresholding::ThresholdSettings m_thresholdSettings;
    int m_numberOfWorms;
    int m_totalFramesInVideo;

    // Number of provided items that already have track data associated with them.
    // This is used to display quick diagnostics in the tracking dialog (e.g.
    // "8 blobs detected — 0 have track data").
    int m_numberOfWormsWithTracks;

    bool m_isTrackingRunning;

    QString formatThresholdSettings(const Thresholding::ThresholdSettings& settings) const;
};

#endif // TRACKINGPROGRESSDIALOG_H

