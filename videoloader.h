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
#include <QList> // For storing WormTrackers
#include <QVariantMap> // For passing parameters

// OpenCV Headers
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

// Forward declarations from this project
class WormTracker;
class Worm;

/**
 * @brief Defines the interaction modes for the VideoLoader widget.
 */
enum class InteractionMode {
    PanZoom,
    DrawROI,
    Crop,
    SelectWorms // New mode for selecting initial worm positions
};

/**
 * @brief Defines available thresholding algorithms.
 */
enum class ThresholdAlgorithm {
    Global,
    Otsu,
    AdaptiveMean,
    AdaptiveGaussian
};

class VideoLoader : public QWidget {
    Q_OBJECT

public:
    explicit VideoLoader(QWidget *parent = nullptr);
    ~VideoLoader();

    // --- Public Methods ---
    bool isVideoLoaded() const;
    int getTotalFrames() const;
    double getFPS() const;
    int getCurrentFrameNumber() const;
    QSize getVideoFrameSize() const;
    double getZoomFactor() const;
    QRectF getCurrentRoi() const; // General purpose ROI
    InteractionMode getCurrentInteractionMode() const;

    // Thresholding status
    bool isThresholdViewEnabled() const;
    ThresholdAlgorithm getCurrentThresholdAlgorithm() const;
    int getThresholdValue() const;
    bool getAssumeLightBackground() const;
    int getAdaptiveBlockSize() const;
    double getAdaptiveCValue() const;
    QVariantMap getCurrentThresholdParameters() const; // Get all threshold params

public slots:
    // --- Control Slots ---
    bool loadVideo(const QString &filePath);
    void play();
    void pause();
    void seekToFrame(int frameNumber, bool suppressEmit = false);
    void setZoomFactor(double factor);
    void setZoomFactorAtPoint(double factor, const QPointF& widgetPoint);
    void setInteractionMode(InteractionMode mode);
    void clearRoi(); // Clears m_activeRoiRect

    // --- Thresholding Control Slots ---
    void toggleThresholdView(bool enabled);
    void setThresholdAlgorithm(ThresholdAlgorithm algorithm);
    void setThresholdValue(int value);
    void setAssumeLightBackground(bool isLight);
    void setAdaptiveThresholdBlockSize(int blockSize);
    void setAdaptiveThresholdC(double cValue);

    // --- Worm Tracking Control Slots ---
    void setNumberOfWormsToSelect(int num); // User specifies how many worms
    void startTrackingSelectedWorms();    // Initiates the tracking process for all selected worms
    void stopAllTracking();               // Stops all active worm tracking threads

signals:
    // --- UI Update Signals ---
    void videoLoaded(const QString& filePath, int totalFrames, double fps, QSize frameSize);
    void videoLoadFailed(const QString& filePath, const QString& errorMessage);
    void videoProcessingStarted(const QString& message);
    void videoProcessingFinished(const QString& message, bool success);
    void frameChanged(int currentFrameNumber, const QImage& currentFrame);
    void playbackStateChanged(bool isPlaying);
    void roiDefined(const QRectF &roi); // General purpose ROI
    void zoomFactorChanged(double newZoomFactor);
    void interactionModeChanged(InteractionMode newMode);
    void thresholdParametersChanged(ThresholdAlgorithm algorithm, int value, bool lightBg, int blockSize, double cVal);

    // --- Worm Tracking Signals ---
    void wormSelected(int wormId, const QPointF& position); // When a worm is clicked in SelectWorms mode
    void allWormsSelected(); // When the specified number of worms have been selected
    void trackingUpdateForDisplay(int wormId, TrackingDirection direction, int frameNum, const QImage& cropImage); // For live display of tracking ROIs
    void wormTrackingCompleted(int wormId); // A specific worm's tracking (both directions) is done
    void allTrackingCompleted(); // All worms' tracking is done


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
    // Slots to handle signals from WormTracker instances
    void onWormTrackerFinished(int wormId);
    void onWormTrackerError(int wormId, const QString& message);
    void onWormTrackerDisplayUpdate(int wormId, TrackingDirection direction, int frameNum, const QImage& cropImage);


private:
    // --- Helper Methods ---
    bool openVideoFile(const QString &filePath);
    void displayFrame(int frameNumber, bool suppressEmit = false);
    void convertCvMatToQImage(const cv::Mat &mat, QImage &qimg);
    QRectF calculateTargetRect() const;
    QPointF mapPointToVideo(const QPointF& widgetPoint) const;
    QPointF mapPointFromVideo(const QPointF& videoPoint) const;
    void updateCursorShape();
    void clampPanOffset();
    void handleRoiDefinedForCrop(const QRectF& cropRoiVideoCoords);
    bool performVideoCrop(const QRectF& cropRectVideoCoords, QString& outCroppedFilePath);
    void applyThresholding();
    void handleWormSelection(const QPointF& widgetClickPos); // Logic for SelectWorms mode


    // --- OpenCV Video Members ---
    cv::VideoCapture videoCapture;
    cv::Mat currentCvFrame;
    cv::Mat m_thresholdedFrame_mono;

    // --- Qt Display Members ---
    QImage currentQImageFrame;
    QTimer *playbackTimer;

    // --- Video Properties ---
    QString currentFilePath;
    int totalFramesCount;
    double framesPerSecond;
    int currentFrameIdx; // This is the KEYFRAME for worm selection
    QSize originalFrameSize;

    // --- Playback State ---
    bool m_isPlaying;

    // --- Interaction State ---
    InteractionMode m_currentMode;
    bool m_isPanning;
    QPointF m_lastMousePos;

    // --- ROI & Crop Selection Members ---
    QRectF m_activeRoiRect; // General purpose ROI for DrawROI mode
    QPoint m_roiStartPointWidget;
    QPoint m_roiEndPointWidget;
    bool m_isDefiningRoi; // For DrawROI and Crop modes

    // --- Zoom & Pan Members ---
    double m_zoomFactor;
    QPointF m_panOffset;

    // --- Thresholding Members ---
    bool m_showThresholdMask;
    ThresholdAlgorithm m_thresholdAlgorithm;
    int m_thresholdValue;
    bool m_assumeLightBackground;
    int m_adaptiveBlockSize;
    double m_adaptiveC;

    // --- Worm Tracking Members ---
    QList<WormTracker*> m_wormTrackers;
    QList<Worm*> m_selectedWormsData; // Temporary storage during selection
    int m_numberOfWormsToSelect;
    int m_wormsSelectedCount;
    bool m_allTrackingTasksCompleted; // Flag to check if all trackers are done
};

#endif // VIDEOLOADER_H
