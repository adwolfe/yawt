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
#include <QVariantMap>

// OpenCV Headers
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

// Forward declarations
class WormTracker;
class Worm;
class ThresholdVideoWorker; // New

/**
 * @brief Defines the interaction modes for the VideoLoader widget.
 */
enum class InteractionMode {
    PanZoom,
    DrawROI,
    Crop,
    SelectWorms
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

    // ... (existing public methods like isVideoLoaded, getZoomFactor, etc.) ...
    bool isVideoLoaded() const;
    int getTotalFrames() const;
    double getFPS() const;
    int getCurrentFrameNumber() const;
    QSize getVideoFrameSize() const;
    double getZoomFactor() const;
    QRectF getCurrentRoi() const;
    InteractionMode getCurrentInteractionMode() const;
    bool isThresholdViewEnabled() const;
    ThresholdAlgorithm getCurrentThresholdAlgorithm() const;
    int getThresholdValue() const;
    bool getAssumeLightBackground() const;
    int getAdaptiveBlockSize() const;
    double getAdaptiveCValue() const;
    QVariantMap getCurrentThresholdParameters() const;
    bool isPreThresholdingInProgress() const;


public slots:
    // --- Control Slots ---
    bool loadVideo(const QString &filePath);
    void play();
    void pause();
    void seekToFrame(int frameNumber, bool suppressEmit = false);
    void setZoomFactor(double factor);
    void setZoomFactorAtPoint(double factor, const QPointF& widgetPoint);
    void setInteractionMode(InteractionMode mode);
    void clearRoi();

    // --- Thresholding Control Slots ---
    void toggleThresholdView(bool enabled);
    void setThresholdAlgorithm(ThresholdAlgorithm algorithm);
    void setThresholdValue(int value);
    void setAssumeLightBackground(bool isLight);
    void setAdaptiveThresholdBlockSize(int blockSize);
    void setAdaptiveThresholdC(double cValue);
    void startVideoPreThresholding(); // New slot to initiate pre-thresholding
    void cancelVideoPreThresholding(); // New slot to cancel

    // --- Worm Tracking Control Slots ---
    void setNumberOfWormsToSelect(int num);
    void startTrackingSelectedWorms();
    void stopAllTracking();

signals:
    // --- UI Update Signals ---
    void videoLoaded(const QString& filePath, int totalFrames, double fps, QSize frameSize);
    void videoLoadFailed(const QString& filePath, const QString& errorMessage);
    void videoProcessingStarted(const QString& message); // General processing
    void videoProcessingFinished(const QString& message, bool success); // General processing
    void preThresholdingProgress(int percentage); // Specific for pre-thresholding
    void preThresholdingCompleted(const QString& outputPath, bool success, const QString& message); // Specific
    void frameChanged(int currentFrameNumber, const QImage& currentFrame);
    void playbackStateChanged(bool isPlaying);
    void roiDefined(const QRectF &roi);
    void zoomFactorChanged(double newZoomFactor);
    void interactionModeChanged(InteractionMode newMode);
    void thresholdParametersChanged(ThresholdAlgorithm algorithm, int value, bool lightBg, int blockSize, double cVal);

    // --- Worm Tracking Signals ---
    // ... (existing worm tracking signals) ...
    void wormSelected(int wormId, const QPointF& position);
    void allWormsSelected();
    void trackingUpdateForDisplay(int wormId, TrackingDirection direction, int frameNum, const QImage& cropImage);
    void wormTrackingCompleted(int wormId);
    void allTrackingCompleted();


protected:
    // ... (event handlers) ...
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void processNextFrame();
    void onWormTrackerFinished(int wormId);
    void onWormTrackerError(int wormId, const QString& message);
    void onWormTrackerDisplayUpdate(int wormId, TrackingDirection direction, int frameNum, const QImage& cropImage);
    void onPreThresholdingWorkerFinished(const QString& outputPath, bool success, const QString& message); // Slot for worker

private:
    // ... (helper methods) ...
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
    void handleWormSelection(const QPointF& widgetClickPos);


    // --- OpenCV Video Members ---
    cv::VideoCapture videoCapture;
    cv::Mat currentCvFrame;
    cv::Mat m_thresholdedFrame_mono; // For interactive threshold view

    // --- Qt Display Members ---
    QImage currentQImageFrame;
    QTimer *playbackTimer;

    // --- Video Properties ---
    QString currentFilePath;        // Path to the *original* loaded video
    QString m_preThresholdedVideoPath; // Path to the generated binary mask video for tracking
    int totalFramesCount;
    double framesPerSecond;
    int currentFrameIdx;
    QSize originalFrameSize;

    // --- Playback State ---
    bool m_isPlaying;

    // --- Interaction State ---
    InteractionMode m_currentMode;
    bool m_isPanning;
    QPointF m_lastMousePos;

    // --- ROI & Crop Selection Members ---
    QRectF m_activeRoiRect;
    QPoint m_roiStartPointWidget;
    QPoint m_roiEndPointWidget;
    bool m_isDefiningRoi;

    // --- Zoom & Pan Members ---
    double m_zoomFactor;
    QPointF m_panOffset;

    // --- Thresholding Members ---
    bool m_showThresholdMask;       // For interactive display
    ThresholdAlgorithm m_thresholdAlgorithm;
    int m_thresholdValue;
    bool m_assumeLightBackground;
    int m_adaptiveBlockSize;
    double m_adaptiveC;
    ThresholdVideoWorker* m_thresholdWorker; // Worker thread for pre-thresholding

    // --- Worm Tracking Members ---
    QList<WormTracker*> m_wormTrackers;
    QList<Worm*> m_selectedWormsData;
    int m_numberOfWormsToSelect;
    int m_wormsSelectedCount;
    bool m_allTrackingTasksCompleted;
};

#endif // VIDEOLOADER_H
