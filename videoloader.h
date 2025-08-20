#ifndef VIDEOLOADER_H
#define VIDEOLOADER_H

#include <QWidget>
#include <QString>
#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QRectF>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QCursor>
#include <QList>
#include <QMap>
#include <QSet>
#include <vector>
#include <QColor>
#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QWaitCondition>
#include <QQueue>
#include <QDateTime>
#include <QAtomicInt>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include "trackingcommon.h" // Contains TrackedItem, DetectedBlob, etc.
#include "trackingdatastorage.h" // Central data storage

// Forward declarations
class TrackingDataStorage;

// Cached frame structure
struct CachedFrame {
    int frameNumber;
    cv::Mat rawFrame;
    cv::Mat thresholdedFrame;
    QDateTime lastAccessed;
    bool isValid;

    CachedFrame() : frameNumber(-1), isValid(false) {}
    CachedFrame(int fn, const cv::Mat& raw)
        : frameNumber(fn), rawFrame(raw.clone()), isValid(true), lastAccessed(QDateTime::currentDateTime()) {}
};

// Thread-safe LRU frame cache
class FrameCache {
public:
    explicit FrameCache(int maxCacheSize = 50);
    ~FrameCache();

    // Cache operations
    void insertFrame(int frameNumber, const cv::Mat& frame);
    bool getFrame(int frameNumber, cv::Mat& outFrame);
    bool hasFrame(int frameNumber) const;
    void clear();
    void setMaxSize(int maxSize);

    // Statistics
    int size() const;
    int maxSize() const;
    double hitRate() const;

private:
    void evictLRU();
    void updateAccessTime(int frameNumber);

    mutable QMutex m_mutex;
    QMap<int, CachedFrame> m_frames;
    int m_maxSize;
    mutable QAtomicInt m_hits;
    mutable QAtomicInt m_requests;
};

// Frame loading request structure
struct FrameLoadRequest {
    int frameNumber;
    int priority; // Higher number = higher priority
    QDateTime requestTime;

    FrameLoadRequest(int fn, int prio = 1)
        : frameNumber(fn), priority(prio), requestTime(QDateTime::currentDateTime()) {}

    bool operator<(const FrameLoadRequest& other) const {
        if (priority != other.priority) return priority < other.priority;
        return requestTime > other.requestTime; // Newer requests first for same priority
    }

    bool operator>(const FrameLoadRequest& other) const {
        if (priority != other.priority) return priority > other.priority;
        return requestTime < other.requestTime; // Older requests last for same priority
    }
};

// Background frame loader worker
class FrameLoader : public QObject {
    Q_OBJECT

public:
    explicit FrameLoader(QObject* parent = nullptr);
    ~FrameLoader();

    void setVideoPath(const QString& path);
    void requestFrames(const QList<int>& frameNumbers, int priority = 1);
    void requestSingleFrame(int frameNumber, int priority = 1);
    void clearRequests();
    void stop();

signals:
    void frameLoaded(int frameNumber, cv::Mat frame);
    void frameLoadError(int frameNumber, QString error);

public slots:
    void processRequests();

private:
    void loadFrame(int frameNumber);

    QString m_videoPath;
    cv::VideoCapture m_videoCapture;
    QQueue<FrameLoadRequest> m_requestQueue;
    mutable QMutex m_queueMutex;
    QWaitCondition m_waitCondition;
    bool m_stopRequested;
    bool m_isProcessing;
};





class VideoLoader : public QWidget {
    Q_OBJECT

public:
    explicit VideoLoader(QWidget *parent = nullptr);
    ~VideoLoader();

    /**
 * @brief Defines the user interaction modes for the VideoLoader widget.
 * User can select one interaction mode at a time.
 */
    enum class InteractionMode {
        PanZoom,        // For panning and zooming the video
        DrawROI,        // For drawing a Region of Interest
        Crop,           // For defining a crop area (uses DrawROI mechanics initially)
        EditBlobs,      // For selecting/clicking blobs on the thresholded image to add to BlobTableModel
        EditTracks      // For interacting with displayed tracks (e.g., selecting, merging - future)
    };
    /**
 * @brief Defines the visual content modes for the VideoLoader widget.
 * User can select one view mode at a time.
 */
    enum class ViewModeOption {
        None      = 0x00,       // No active view overlays/modes
        Threshold = 0x01,       // Show thresholded image as base
        Blobs     = 0x02,       // Overlay blob information
        Tracks    = 0x04        // Overlay track information
        // You could add combined flags like:
        // AllOverlays = Blobs | Tracks,
        // FullDebug = Threshold | Blobs | Tracks
    };
    Q_DECLARE_FLAGS(ViewModeOptions, ViewModeOption) // Creates ViewModeOptions, which is QFlags<ViewModeOption>
    Q_FLAG(ViewModeOptions) // Makes ViewModeOptions usable in Qt's property system, if needed

    // --- Public Methods ---
    void setTrackingDataStorage(TrackingDataStorage* storage);
    bool isVideoLoaded() const;
    int getTotalFrames() const;
    double getFPS() const;
    int getCurrentFrameNumber() const;
    QImage getCurrentQImageFrame() const;
    QSize getVideoFrameSize() const;
    double getZoomFactor() const;
    QRectF getCurrentRoi() const; // The general purpose ROI (e.g., for processing)
    InteractionMode getCurrentInteractionMode() const;
    VideoLoader::ViewModeOptions getActiveViewModes() const;
    double getPlaybackSpeed() const;
    QString getCurrentVideoPath() const;
    QString getDataDirectory() const;

    // Frame cache management
    void setCacheSize(int maxFrames);
    int getCacheSize() const;
    double getCacheHitRate() const;

    // --- Thresholding and Pre-processing Status Getters ---
    // bool isThresholdViewEnabled() const; // This will be controlled by ViewMode::Threshold
    Thresholding::ThresholdSettings getCurrentThresholdSettings() const;
    Thresholding::ThresholdAlgorithm getCurrentThresholdAlgorithm() const;
    int getThresholdValue() const;
    bool getAssumeLightBackground() const;
    int getAdaptiveBlockSize() const;
    double getAdaptiveCValue() const;
    bool isBlurEnabled() const;
    int getBlurKernelSize() const;
    double getBlurSigmaX() const;



public slots:
    // --- Control Slots ---
    bool loadVideo(const QString &filePath);
    void play();
    void pause();
    void seekToFrame(int frameNumber, bool suppressEmit = false);
    void setZoomFactor(double factor);
    void setZoomFactorAtPoint(double factor, const QPointF& widgetPoint);
    void centerOnVideoPoint(const QPointF& videoPoint);

    // --- Mode Setting Slots ---
    void setInteractionMode(InteractionMode mode);
    void setViewModeOption(VideoLoader::ViewModeOption option, bool active);

    void clearRoi(); // Clears the general purpose ROI
    void setPlaybackSpeed(double multiplier);
    // void clearWormSelections(); // Will be obsolete as selections are not stored here temporarily

    // --- Thresholding & Pre-processing Control Slots ---
    // void toggleThresholdView(bool enabled); // Replaced by setViewMode(ViewMode::Threshold)
    void setThresholdAlgorithm(Thresholding::ThresholdAlgorithm algorithm);
    void setThresholdValue(int value);
    void setAssumeLightBackground(bool isLight);
    void setAdaptiveThresholdBlockSize(int blockSize);
    void setAdaptiveThresholdC(double cValue);
    void setEnableBlur(bool enabled);
    void setBlurKernelSize(int kernelSize);
    void setBlurSigmaX(double sigmaX);

    // --- Slots for Data Display from Models ---
    /**
     * @brief Sets or updates the list of TrackedItems (blobs/worms) to be displayed.
     * Called by MainWindow when the BlobTableModel changes.
     * @param items The list of items to display.
     */
    void updateItemsToDisplay(const QList<TableItems::ClickedItem>& items);

    /**
     * @brief Sets the tracks to be displayed.
     * @param tracks The map of track ID to track points.
     */
    void setTracksToDisplay(const Tracking::AllWormTracks& tracks);

    /**
     * @brief Sets which track IDs should be visible.
     * @param visibleTrackIDs Set of IDs for visible tracks.
     */
    void setVisibleTrackIDs(const QSet<int>& visibleTrackIDs);

    void clearDisplayedTracks(); // Clears m_allTracksToDisplay and m_visibleTrackIDs

    // --- Slot for Worm Color Updates (from BlobTableModel, if still needed directly) ---
    void updateWormColor(int wormId, const QColor& color);


signals:
    // --- UI Update Signals ---
    void videoLoaded(const QString& filePath, int totalFrames, double fps, QSize frameSize);
    void videoLoadFailed(const QString& filePath, const QString& errorMessage);
    void videoProcessingStarted(const QString& message);
    void videoProcessingFinished(const QString& message, bool success);
    void frameChanged(int currentFrameNumber, const QImage& currentFrame); // QImage is current frame (raw or thresholded based on ViewMode)
    void playbackStateChanged(bool isPlaying, double currentSpeed);
    void roiDefined(const QRectF &roi); // For the general purpose ROI
    void zoomFactorChanged(double newZoomFactor);

    void interactionModeChanged(InteractionMode newMode);
    void activeViewModesChanged(VideoLoader::ViewModeOptions newModes);

    void thresholdParametersChanged(const Thresholding::ThresholdSettings& newSettings);
    void playbackSpeedChanged(double newSpeedMultiplier);

    /**
     * @brief Emitted when a blob is clicked by the user in EditBlobs mode,
     * suggesting it should be added to the BlobTableModel.
     * @param blobData The data of the clicked blob (centroid, bounding box in video coordinates).
     */
    void blobClickedForAddition(const Tracking::DetectedBlob& blobData);

    void trackPointClicked(int wormId, int frameNumber, QPointF videoPoint); // For interaction in EditTracks mode

    /**
     * @brief Emitted when the data directory is created or changed.
     * @param directoryPath The path to the yawt data directory.
     */
    void dataDirectoryChanged(const QString& directoryPath);


protected:
    // --- Qt Event Handlers ---
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void processNextFrame();

private:
    // --- Helper Methods ---
    bool openVideoFile(const QString &filePath);
    void displayFrame(int frameNumber, bool suppressEmit = false); // Will now consider ViewMode for QImage content
    void convertCvMatToQImage(const cv::Mat &mat, QImage &qimg);
    QRectF calculateTargetRect() const;
    QPointF mapPointToVideo(const QPointF& widgetPoint) const;
    QPointF mapPointFromVideo(const QPointF& videoPoint) const;
    void updateCursorShape();
    void clampPanOffset();
    void handleRoiDefinedForCrop(const QRectF& cropRoiVideoCoords);
    bool performVideoCrop(const QRectF& cropRectVideoCoords, QString& outCroppedFilePath);
    void applyThresholding(); // Applies thresholding to currentCvFrame, stores in m_thresholdedFrame_mono
    void updateTimerInterval();
    void emitThresholdParametersChanged();
    QColor getTrackColor(int trackId) const; // Used for drawing tracks
    QString createDataDirectory(const QString& videoFilePath); // Creates "yawt" directory for data storage

    // Frame caching and loading helpers
    bool getCachedFrame(int frameNumber, cv::Mat& outFrame);
    void preloadAdjacentFrames(int centerFrame, int radius = 5);
    void startFrameLoader();
    void stopFrameLoader();

private slots:
    void onFrameLoaded(int frameNumber, cv::Mat frame);
    void onFrameLoadError(int frameNumber, QString error);

private:
    // --- OpenCV Video Members ---
    cv::VideoCapture videoCapture;
    cv::Mat currentCvFrame;          // Holds the raw/original current frame from video
    cv::Mat m_thresholdedFrame_mono; // Holds the binary thresholded version of currentCvFrame

    // --- Qt Display Members ---
    QImage currentQImageFrame;       // The QImage actually painted (can be raw, thresholded, etc.)
    QTimer *playbackTimer;

    // --- Video Properties ---
    QString currentFilePath;
    int totalFramesCount;
    double framesPerSecond;
    int currentFrameIdx;
    QSize originalFrameSize;

    // --- Playback State ---
    bool m_isPlaying;
    double m_playbackSpeedMultiplier;

    // --- Mode States ---
    InteractionMode m_currentInteractionMode;
    ViewModeOptions m_activeViewModes;
    bool m_isPanning;
    QPointF m_lastMousePos;

    // --- ROI & Crop Selection Members ---
    QRectF m_activeRoiRect;         // The general purpose ROI (e.g., for processing or display)
    QPoint m_roiStartPointWidget;   // For drawing ROI interactively
    QPoint m_roiEndPointWidget;     // For drawing ROI interactively
    bool m_isDefiningRoi;           // True when user is dragging to define an ROI

    // --- Data for Display (received from models) ---
    QList<TableItems::ClickedItem> m_itemsToDisplay; // List of blobs/worms to display (from BlobTableModel)
    Tracking::AllWormTracks m_allTracksToDisplay;  // All tracks data
    QSet<int> m_visibleTrackIDs;         // IDs of tracks that should be currently rendered
    mutable QMap<int, QColor> m_trackColors; // Cache for track/item colors
    TrackingDataStorage* m_storage;      // Pointer to central data storage

    // --- Zoom & Pan Members ---
    double m_zoomFactor;
    QPointF m_panOffset;

    // --- Thresholding & Pre-processing Members ---
    // bool m_showThresholdMask; // This state is now part of m_currentViewMode (ViewMode::Threshold)
    Thresholding::ThresholdAlgorithm m_thresholdAlgorithm;
    int m_thresholdValue;
    bool m_assumeLightBackground;
    int m_adaptiveBlockSize;
    double m_adaptiveC;
    bool m_enableBlur;
    int m_blurKernelSize;
    double m_blurSigmaX;

    // --- Data Directory ---
    QString m_dataDirectory;  // Path to the "yawt" directory for data storage

    // --- Frame Caching and Background Loading ---
    FrameCache* m_frameCache;                  // Thread-safe frame cache
    FrameLoader* m_frameLoader;                // Background frame loader
    QThread* m_frameLoaderThread;              // Thread for background loading
    int m_lastPreloadCenter;                   // Last center frame for preloading
    mutable QAtomicInt m_pendingSeekFrame;     // Frame number for pending seek operation
};

Q_DECLARE_OPERATORS_FOR_FLAGS(VideoLoader::ViewModeOptions)

#endif // VIDEOLOADER_H
