#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QItemSelection>
#include <QButtonGroup>
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
class TrackingDataStorage;
class AppController; // Application controller (owns core non-UI components)

// Include VideoLoader header for enums and QFlags type
#include "widgets/videoloader.h"    // For VideoLoader::ViewModeOption, VideoLoader::ViewModeOptions etc.
#include "../data/trackingcommon.h" // For Tracking::AllWormTracks and Tracking::DetectedBlob
#include "../data/trackingdatastorage.h" // Central data storage

QT_BEGIN_NAMESPACE
QT_END_NAMESPACE

/**
 * MainWindow
 *
 * The primary UI class. After recent refactors, MainWindow no longer creates or owns the
 * TrackingManager or the TrackingProgressDialog. Those responsibilities are delegated to
 * AppController. MainWindow obtains models and the central storage from AppController and
 * remains responsible for view-layer logic (widget wiring, playback controls, selection handling).
 *
 * Notes:
 *  - MainWindow keeps a pointer to AppController (typically created in MainWindow ctor).
 *  - MainWindow may keep non-owning pointers to models/storage obtained from the controller for
 *    backward-compatible wiring with existing UI code paths.
 *  - The TrackingProgressDialog is created and owned by AppController (if shown via
 *    AppController::showTrackingDialog). MainWindow should call into the controller to show the dialog.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
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

    // Tracking Process (UI entry points)
    // MainWindow triggers tracking flows via AppController. These slots exist so the
    // existing UI wiring remains compatible with the refactor.
    void onStartTrackingActionTriggered();           // User clicked tracking button -> ask AppController to show dialog / start tracking.
    void handleBeginTrackingFromDialog();            // Kept as a slot for legacy direct-dialog flows (may be invoked by controller-owned dialog via a connection).
    void handleCancelTrackingFromDialog();           // Kept for compatibility.
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

    // Models & delegates (non-owning or parented to MainWindow as appropriate)
    BlobTableModel *m_blobTableModel;
    AnnotationTableModel *m_annotationTableModel;
    ColorDelegate *m_colorDelegate;
    ItemTypeDelegate *m_itemTypeDelegate;

    // AppController owns TrackingManager, TrackingDataStorage, and models.
    // MainWindow keeps a pointer to the controller to request high-level operations.
    AppController *m_appController; // Controller owning storage, manager and models

    // Central data storage pointer (may be obtained from AppController for convenience).
    // MainWindow does not own this; lifetime is managed by the controller (or parent).
    TrackingDataStorage *m_trackingDataStorage;

    QButtonGroup *m_interactionModeButtonGroup; // For Pan, ROI, Crop, EditBlobs, EditTracks

    // Tracking/view state
    bool m_hasCompletedTracking; // Whether initial tracking has been completed

    // ROI size factor spinbox
    double roiFactorSpinBoxD;

    // Merge/Split events model
    QStandardItemModel* m_mergeSplitModel;

    // Polling fallback: timer to query mini loaders periodically and update visible IDs if changed
    QTimer* m_miniLoaderPollTimer;
    // Last polled set to detect changes and avoid redundant UI updates
    QSet<int> m_lastPolledVisibleIds;
};

#endif // MAINWINDOW_H