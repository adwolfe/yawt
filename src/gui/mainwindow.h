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
#include <QPointer>

// Forward declarations
namespace Ui { class MainWindow; }
class BlobTableModel;
class AnnotationTableModel;
class ColorDelegate;
class ItemTypeDelegate;
class ItemTypeFilterProxyModel;
class TrackingDataStorage;
class AppController; // Application controller (owns core non-UI components)
class AnalysisDialog;
class AnalysisPanel;
class CapturePanel;

// Include VideoLoader header for enums and QFlags type
#include "widgets/videoloader.h"    // For VideoLoader::ViewModeOption, VideoLoader::ViewModeOptions etc.
#include "../data/trackingcommon.h" // For Tracking::AllWormTracks and Tracking::DetectedBlob
#include "../data/trackingdatastorage.h" // Central data storage

QT_BEGIN_NAMESPACE
QT_END_NAMESPACE

/**
 * MainWindow
 *
 * Primary UI shell for YAWT. MainWindow owns widgets and view-layer behavior (menus, playback,
 * selection, mode toggles) and binds its views to models provided by AppController. It routes
 * high-level user actions to AppController, which orchestrates non-UI logic (storage, tracking).
 *
 * Responsibilities:
 *  - Manage and wire UI widgets (VideoLoader, MiniLoader, table views, delegates).
 *  - Bind to controller-provided models (BlobTableModel, AnnotationTableModel).
 *  - Handle user interactions: file selection, playback, ROI creation, threshold controls.
 *  - Keep UI in sync with VideoLoader interaction/view modes and visible tracks.
 *
 * Non-responsibilities:
 *  - Does not own TrackingManager, TrackingDataStorage, or worker threads.
 *  - Does not implement tracking algorithms or non-UI orchestration.
 *
 * Threading and lifetime:
 *  - Lives on the GUI thread.
 *  - Holds a pointer to AppController (typically parented to MainWindow); QObject parent/child
 *    semantics apply for deterministic lifetimes of controller-owned components.
 *
 * Key interactions:
 *  - AppController: requests to start/cancel tracking; obtains models/storage; receives progress
 *    and final tracks via signals.
 *  - VideoLoader/MiniLoader: frame navigation, ROI gestures, visible-worm updates.
 *  - Models: BlobTableModel and AnnotationTableModel are set on their respective views.
 *
 * Signals/slots overview:
 *  - Slots cover file/directory selection, playback/navigation, ROI creation, threshold updates,
 *    and tracking entry points (onStartTrackingActionTriggered(), acceptTracksFromManager()).
 *
 * Notes:
 *  - TrackingProgressDialog can be owned/wired by AppController. Prefer invoking
 *    AppController::showTrackingDialog(...) rather than constructing dialogs here.
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
    bool eventFilter(QObject* obj, QEvent* event) override;

/**
 * Slots: UI event handlers and high-level entry points.
 * - File/Directory: chooseWorkingDirectory()
 * - Video: initiateFrameDisplay(), updateFrameDisplay(), updateMiniLoaderCrop(), seekFrame(), frameSliderMoved()
 * - Thresholding: setGlobalThresholdType(), setAdaptiveThresholdType(), setGlobalThresholdValue(), setAdaptiveBlockSize(), setAdaptiveCValue(), setBlurEnabled(), setBlurKernel(), setBackgroundAssumption()
 * - Tracking: onStartTrackingActionTriggered(), handleBeginTrackingFromDialog(), handleCancelTrackingFromDialog(), acceptTracksFromManager(), performPostTrackingMemoryCleanup()
 * - UI sync: syncInteractionModeButtons(), syncViewModeOptionButtons(), updateVisibleTracksInVideoLoader()
 */
public slots:
    // File/Directory Operations
    void chooseWorkingDirectory();
    void loadRunFromDirectory();
    void loadRunFromDirectoryPath(const QString& directoryPath);

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

    // Debug tab: reruns centerline computation with current snake spinbox values.
    void onRerunCenterlineClicked();
    // Debug tab: called when a rerun centerline pass makes progress.
    void onDebugCenterlineProgress(int percentage);
    // Debug tab: called when a rerun centerline pass finishes.
    void onDebugCenterlineFinished();
    // Debug tab: export per-stage diagnostic images + log for the currently
    // selected worm at the current frame.
    void onExportProcessClicked();

    // VideoLoader Interaction Mode Toggles (using QButtonGroup or individual slots)
    void panModeButtonClicked();
    void roiModeButtonClicked();
    void pointModeButtonClicked();
    void cropModeButtonClicked();
    void editBlobsModeButtonClicked();
    void editTracksModeButtonClicked();

    // VideoLoader View Mode Option Toggles (Individual checkable buttons)
    void onViewThresholdToggled(bool checked); // Connected to ui->showThreshButton's toggled()
    void onViewBlobsToggled(bool checked);     // Connected to a new ui->viewBlobsButton's toggled()
    void onViewTracksToggled(bool checked);    // Connected to a new ui->viewTracksButton's toggled()
    void onViewSkeletonsToggled(bool checked); // Connected to ui->skeletonButton's toggled()

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
    /** 
     * Handle a newly drawn ROI from the VideoLoader.
     * Adds the ROI as a worm candidate in the BlobTableModel for the current keyframe.
     * @param roi Rectangle in video coordinates drawn by the user.
     */
    void handleRoiDefined(const QRectF& roi);
    void handlePointDefined(const QPointF& point);

    // Slots to react to VideoLoader mode changes (for UI sync)
    void syncInteractionModeButtons(VideoLoader::InteractionMode newMode);
    void syncViewModeOptionButtons(VideoLoader::ViewModeOptions newModes); // Updated for QFlags
    void onPlaybackStateChanged(bool isPlaying, double currentSpeed);
    void resultsButtonClicked();
    void onDebugImageTableSelectionChanged();
    void onMainTabChanged(int index);

private:
    void setupConnections();
    void initializeUIStates();
    /** Called when the user finishes manually editing the pixel-size spinbox.
     *  Persists the new value to the current video's metadata JSON. */
    void onPixelSizeSpinEditingFinished();
    void setupInteractionModeButtonGroup(); // Renamed for clarity
    void resizeTableColumns(); // Resize WormTableView columns to fit contents
    void updateWormTimeline();
    bool applyThresholdSettingsFromJsonFile(const QString& filePath);
    bool loadRunFromDirectoryInternal(const QString& directoryPath);
    void populateDebugImageTable(const QString& dir);
    void onDebugTabChanged(bool active);
    void runDebugExport(bool silent);

    /**
     * Keep the mirrored play/pause buttons in sync with the current playback state.
     * Use this helper instead of duplicating UI toggle logic.
     * @param playing When true, reflect that playback is active; otherwise paused/stopped.
     * @param blockSignals When true, temporarily block signals on the widgets to avoid feedback loops.
     */
    void setPlayButtonsState(bool playing, bool blockSignals = true);
    /**
     * Convenience query indicating whether any of the mirrored play buttons is currently checked.
     * @return true if either play button is checked; false otherwise.
     */
    bool arePlayButtonsChecked() const;

    /** Qt Designer-generated UI form. Owned by MainWindow via QObject parent hierarchy. */
    Ui::MainWindow *ui;

    // Models & delegates
    /** Non-owning pointer to the blob model (owned by AppController). Bound via proxy models to views. */
    BlobTableModel *m_blobTableModel;
    /** Non-owning pointer to the annotation model (owned by AppController). Bound to annotation view. */
    AnnotationTableModel *m_annotationTableModel;
    /** Color cell delegate created/parented to MainWindow; owned by MainWindow unless reparented. */
    ColorDelegate *m_colorDelegate;
    /** Item type delegate created/parented to MainWindow; owned by MainWindow unless reparented. */
    ItemTypeDelegate *m_itemTypeDelegate;
    /** Proxy models to split worms vs ROI/points in separate views. */
    ItemTypeFilterProxyModel *m_wormProxyModel = nullptr;
    ItemTypeFilterProxyModel *m_roiProxyModel = nullptr;

    // Controller and storage
    /**
     * Application controller that owns core non-UI components:
     * - TrackingManager, TrackingDataStorage, and the core models.
     * Typically constructed in MainWindow and parented to MainWindow, so lifetime is tied to UI.
     */
    AppController *m_appController; // Controller owning storage, manager and models
    /**
     * Non-owning pointer to the central data storage.
     * Owned by AppController; retrieved for convenience to avoid repeated lookups.
     */
    TrackingDataStorage *m_trackingDataStorage;

    QPointer<AnalysisDialog> m_analysisDialog;
    AnalysisPanel* m_analysisPanel = nullptr;
    CapturePanel* m_capturePanel = nullptr;  // QObject controller, not a widget

    // Tracks the currently-loaded video so pixel-size edits can be saved to JSON.
    QString m_currentVideoDataDir;
    QString m_currentVideoBaseName;

    // Tab-switch state: saved worm ID selection before entering Analysis tab
    int  m_savedAnalysisWormId = -1;
    bool m_analysisTabActive   = false;

    /** Button group for interaction modes (Pan, ROI, Crop, EditBlobs, EditTracks). Owned by MainWindow. */
    QButtonGroup *m_interactionModeButtonGroup;

    // Tracking/view state
    /** Whether initial tracking has been completed in the current session. Pure UI state. */
    bool m_hasCompletedTracking;
    double m_videoFps = 0.0;
    bool m_isVideoPlaying = false;
    int m_lastMiniLoaderFrame = -1;
    bool m_startEndSelectionActive = false;
    TableItems::ItemType m_nextStartEndPointType = TableItems::ItemType::StartPoint;

    /** Backing value for the ROI size multiplier spinbox. Pure UI state. */
    double roiFactorSpinBoxD;

    // Debug image viewer state
    QString m_debugExportDir;   // last successful export output directory
    bool m_debugTabActive = false; // true while the Debug tab is the current tab
};

#endif // MAINWINDOW_H
