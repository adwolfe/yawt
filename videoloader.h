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

// OpenCV Headers
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief Defines the interaction modes for the VideoLoader widget.
 */
enum class InteractionMode {
    PanZoom,
    DrawROI,
    Crop
};

/**
 * @brief Defines available thresholding algorithms.
 */
enum class ThresholdAlgorithm {
    Global,         // Simple global threshold
    Otsu,           // Otsu's binarization (auto global threshold)
    AdaptiveMean,   // Adaptive threshold using mean of neighborhood
    AdaptiveGaussian // Adaptive threshold using Gaussian weighted sum of neighborhood
};

class VideoLoader : public QWidget {
    Q_OBJECT

public:
    explicit VideoLoader(QWidget *parent = nullptr);
    ~VideoLoader();

    // --- Public Methods ---
    bool isVideoLoaded() const;
    int getTotalFrames() const;
    double getFPS() const; // This remains the ORIGINAL video FPS
    int getCurrentFrameNumber() const;
    QSize getVideoFrameSize() const;
    double getZoomFactor() const;
    QRectF getCurrentRoi() const;
    InteractionMode getCurrentInteractionMode() const;
    double getPlaybackSpeed() const; // Getter for current playback speed

    // Thresholding status
    bool isThresholdViewEnabled() const;
    ThresholdAlgorithm getCurrentThresholdAlgorithm() const;
    int getThresholdValue() const;
    bool getAssumeLightBackground() const;
    int getAdaptiveBlockSize() const;
    double getAdaptiveCValue() const;


public slots:
    // --- Control Slots ---
    bool loadVideo(const QString &filePath);
    void play(); // Toggles play/pause
    void pause();
    void seekToFrame(int frameNumber, bool suppressEmit = false);
    void setZoomFactor(double factor);
    void setZoomFactorAtPoint(double factor, const QPointF& widgetPoint);
    void setInteractionMode(InteractionMode mode);
    void clearRoi();
    void setPlaybackSpeed(double multiplier); // New slot to change playback speed

    // --- Thresholding Control Slots ---
    void toggleThresholdView(bool enabled);
    void setThresholdAlgorithm(ThresholdAlgorithm algorithm);
    void setThresholdValue(int value); // For Global threshold
    void setAssumeLightBackground(bool isLight); // True for light bg/dark objects
    void setAdaptiveThresholdBlockSize(int blockSize); // For adaptive algorithms (odd, >=3)
    void setAdaptiveThresholdC(double cValue);      // For adaptive algorithms

signals:
    // --- UI Update Signals ---
    void videoLoaded(const QString& filePath, int totalFrames, double fps, QSize frameSize);
    void videoLoadFailed(const QString& filePath, const QString& errorMessage);
    void videoProcessingStarted(const QString& message);
    void videoProcessingFinished(const QString& message, bool success);
    void frameChanged(int currentFrameNumber, const QImage& currentFrame); // Emits original or thresholded QImage
    void playbackStateChanged(bool isPlaying, double currentSpeed); // Added currentSpeed
    void roiDefined(const QRectF &roi);
    void zoomFactorChanged(double newZoomFactor);
    void interactionModeChanged(InteractionMode newMode);
    void thresholdParametersChanged(ThresholdAlgorithm algorithm, int value, bool lightBg, int blockSize, double cVal);
    void playbackSpeedChanged(double newSpeedMultiplier); // New signal


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
    void displayFrame(int frameNumber, bool suppressEmit = false);
    void convertCvMatToQImage(const cv::Mat &mat, QImage &qimg); // Converts BGR/Gray CV::Mat to QImage
    QRectF calculateTargetRect() const;
    QPointF mapPointToVideo(const QPointF& widgetPoint) const;
    QPointF mapPointFromVideo(const QPointF& videoPoint) const;
    void updateCursorShape();
    void clampPanOffset();
    void handleRoiDefinedForCrop(const QRectF& cropRoiVideoCoords);
    bool performVideoCrop(const QRectF& cropRectVideoCoords, QString& outCroppedFilePath);
    void applyThresholding(); // Applies current threshold settings to currentCvFrame -> m_thresholdedFrame_mono
    void updateTimerInterval(); // Helper to set timer interval based on FPS and speed multiplier


    // --- OpenCV Video Members ---
    cv::VideoCapture videoCapture;
    cv::Mat currentCvFrame;       // Original current frame from OpenCV
    cv::Mat m_thresholdedFrame_mono; // Grayscale thresholded mask

    // --- Qt Display Members ---
    QImage currentQImageFrame;    // QImage to be displayed (either original or thresholded)
    QTimer *playbackTimer;

    // --- Video Properties ---
    QString currentFilePath;
    int totalFramesCount;
    double framesPerSecond;       // ORIGINAL video FPS
    int currentFrameIdx;
    QSize originalFrameSize;

    // --- Playback State ---
    bool m_isPlaying;
    double m_playbackSpeedMultiplier; // New: For controlling playback speed (1.0 is normal)

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
    bool m_showThresholdMask;
    ThresholdAlgorithm m_thresholdAlgorithm;
    int m_thresholdValue;             // For Global/Otsu (Otsu ignores this input if used)
    bool m_assumeLightBackground;     // True: objects are darker than background
    int m_adaptiveBlockSize;          // For adaptive thresholding (e.g., 11, 21, etc.)
    double m_adaptiveC;               // Constant for adaptive thresholding (e.g., 2, 5, etc.)
};

#endif // VIDEOLOADER_H
