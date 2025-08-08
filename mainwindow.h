#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QItemSelection>
#include <QButtonGroup> // For interaction modes
#include <QResizeEvent>

// Forward declarations
namespace Ui { class MainWindow; }
class BlobTableModel;
class ColorDelegate;
class ItemTypeDelegate;
class TrackingProgressDialog;
class TrackingManager;
class TrackingDataStorage;

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
    void seekFrame(int frame);
    void frameSliderMoved(int value);
    void goToFirstFrame();
    void goToLastFrame();

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

    // Slots to react to VideoLoader mode changes (for UI sync)
    void syncInteractionModeButtons(VideoLoader::InteractionMode newMode);
    void syncViewModeOptionButtons(VideoLoader::ViewModeOptions newModes); // Updated for QFlags




private:
    void setupConnections();
    void initializeUIStates();
    void setupInteractionModeButtonGroup(); // Renamed for clarity
    void resizeTableColumns(); // Resize WormTableView columns to fit contents



    Ui::MainWindow *ui;
    BlobTableModel *m_blobTableModel;
    ColorDelegate *m_colorDelegate;
    ItemTypeDelegate *m_itemTypeDelegate;
    TrackingProgressDialog *m_trackingProgressDialog;
    TrackingManager *m_trackingManager;
    TrackingDataStorage *m_trackingDataStorage; // Central data storage

    QButtonGroup *m_interactionModeButtonGroup; // For Pan, ROI, Crop, EditBlobs, EditTracks
    
    // ROI size factor spinbox
    double roiFactorSpinBoxD;
};
#endif // MAINWINDOW_H
