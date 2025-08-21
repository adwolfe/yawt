#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QItemSelection>
#include <QButtonGroup> // For interaction modes
#include <QResizeEvent>
#include <QStandardItemModel>
#include <QList>
#include <QSet>
#include <QTimer>

// Forward declarations
namespace Ui { class MainWindow; }
class BlobTableModel;
class AnnotationTableModel;
class ColorDelegate;
class ItemTypeDelegate;
class TrackingProgressDialog;
class TrackingManager;
class TrackingDataStorage;
// Retracking forward declarations removed

// Include VideoLoader header for enums and QFlags type
#include "videoloader.h"    // For VideoLoader::ViewModeOption, VideoLoader::ViewModeOptions etc.
#include "trackingcommon.h" // For AllWormTracks and Tracking::DetectedBlob
#include "trackingdatastorage.h" // Central data storage

QT_BEGIN_NAMESPACE
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    
protected:
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;

public slots:
    // File/Directory Operations
    void chooseWorkingDirectory();

    // Video Playback and Frame Navigation
    void initiateFrameDisplay(const QString& filePath, int totalFrames, double fps, QSize frameSize);
    void updateFrameDisplay(int currentFrameNumber, const QImage& currentFrame);
    void updateMiniLoaderCrop(int currentFrameNumber, const QImage& currentFrame);
    void seekFrame(int frame);
    void frameSliderMoved(int value);

    void goToFirstFrame();
    void goToLastFrame();
    void onAnnotationTableClicked(const QModelIndex& index);
    
public slots:
    // Debug control slots
    void toggleTrackingDebug();

    // VideoLoader Interaction Mode Toggles (using QButtonGroup or individual slots)
    void panModeButtonClicked();
    void roiModeButtonClicked();
    void cropModeButtonClicked();
    void editBlobsModeButtonClicked();
    void editTracksModeButtonClicked();

    // VideoLoader View Mode Option Toggles (Individual checkable buttons)
    void onViewThresholdToggled(bool checked); // Connected to ui->showThreshButton's toggled()
    void onViewBlobsToggled(bool checked);     // Connected to a new ui->viewBlobsButton's toggled()
    void onViewTracksToggled(bool checked);    // Connected to a new ui->viewTracksButton's toggled()

    // Thresholding Parameter Updates
    void updateThresholdAlgorithmSettings();
    void setGlobalThresholdType(bool isAuto);
    void setAdaptiveThresholdType(int index);
    void setGlobalThresholdValue(int value);
    void setAdaptiveBlockSize(int value);
    void setAdaptiveCValue(double value);
    void setBlurEnabled(bool checked);
    void setBlurKernel(int value);
    void setBackgroundAssumption(int index);

    // Blob/Item Handling
    void handleBlobClickedForAddition(const Tracking::DetectedBlob& blobData);
    void handleRemoveBlobsClicked();
    void handleDeleteSelectedBlobClicked();

    
    // Retracking functionality
    // Retracking functionality removed


    // Tracking Process
    void onStartTrackingActionTriggered();
    void handleBeginTrackingFromDialog();
    void handleCancelTrackingFromDialog();
    void acceptTracksFromManager(const Tracking::AllWormTracks& tracks);
    void performPostTrackingMemoryCleanup();
    
    // Playback speed control
    void setupPlaybackSpeedComboBox();
    void onPlaybackSpeedChanged(int index);
    void updatePlaybackSpeedComboBox(double speedMultiplier);

    // Table View and VideoLoader Sync
    void updateVisibleTracksInVideoLoader(const QItemSelection &selected, const QItemSelection &deselected);
    // ROI handling: when the user draws an ROI in VideoLoader, add it to the BlobTableModel as an ROI item
    void handleRoiDefined(const QRectF& roi);

    // Slots to react to VideoLoader mode changes (for UI sync)
    void syncInteractionModeButtons(VideoLoader::InteractionMode newMode);
    void syncViewModeOptionButtons(VideoLoader::ViewModeOptions newModes); // Updated for QFlags
    // Slot: receive list of visible worm IDs from MiniLoader and update merge history/filtering
    void onMiniLoaderVisibleWormsUpdated(const QList<int>& visibleIds);

    // Poll timer tick: periodically poll MiniLoader(s) for visible IDs (fallback if signal missed)
    void onMiniLoaderPollTimeout();




private:
    void setupConnections();
    void initializeUIStates();
    void setupInteractionModeButtonGroup(); // Renamed for clarity
    void resizeTableColumns(); // Resize WormTableView columns to fit contents

    // Helper to keep the pair of play/pause buttons in sync.
    // Implemented in the .cpp file. Use this everywhere instead of duplicating logic.
    void setPlayButtonsState(bool playing, bool blockSignals = true);
    // Convenience: query whether either of the mirrored play buttons is checked.
    bool arePlayButtonsChecked() const;

    Ui::MainWindow *ui;
    BlobTableModel *m_blobTableModel;
    AnnotationTableModel *m_annotationTableModel;
    ColorDelegate *m_colorDelegate;
    ItemTypeDelegate *m_itemTypeDelegate;
    TrackingProgressDialog *m_trackingProgressDialog;
    TrackingManager *m_trackingManager;
    TrackingDataStorage *m_trackingDataStorage; // Central data storage

    QButtonGroup *m_interactionModeButtonGroup; // For Pan, ROI, Crop, EditBlobs, EditTracks
    
    // Tracking state
    bool m_hasCompletedTracking; // Whether initial tracking has been completed
    
    // ROI size factor spinbox
    double roiFactorSpinBoxD;
    
    // Model for merge/split events table
    QStandardItemModel* m_mergeSplitModel;
    // Currently visible worm IDs inside MiniLoader crop (updated via signal)
    QSet<int> m_miniLoaderVisibleIds;

    // Polling fallback: timer to query mini loaders periodically and update visible IDs if changed
    QTimer* m_miniLoaderPollTimer;
    // Last polled set to detect changes and avoid redundant UI updates
    QSet<int> m_lastPolledVisibleIds;

    // Helper to rebuild the merge history text widget for a given frame using a provided visible set
    QString buildMergeHistoryText(int currentFrameNumber, const QSet<int>& visibleSet) const;
};
#endif // MAINWINDOW_H
