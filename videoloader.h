#ifndef VIDEOLOADER_H
#define VIDEOLOADER_H

#include <QWidget>
#include <QString>
#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QRectF> // Use QRectF for precision
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QCursor> // For setting cursors

// OpenCV Headers
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief Defines the interaction modes for the VideoLoader widget.
 */
enum class InteractionMode {
    PanZoom, // Default mode: Left-click drag to pan, Wheel to zoom towards cursor
    DrawROI  // Mode for drawing ROI: Left-click drag defines ROI
};

/**
 * @brief The VideoLoader class handles loading, displaying, and interacting with video files.
 * Supports playback, frame scrubbing, zoom-to-cursor, panning, and ROI selection
 * via different interaction modes.
 */
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
    QRectF getCurrentRoi() const;
    InteractionMode getCurrentInteractionMode() const;

public slots:
    // --- Control Slots ---
    bool loadVideo(const QString &filePath);
    void play();
    void pause();
    void stop();
    void seekToFrame(int frameNumber);
    void setZoomFactor(double factor); // Zooms towards center
    void setZoomFactorAtPoint(double factor, const QPointF& widgetPoint); // Zooms towards point
    void setInteractionMode(InteractionMode mode); // Switch between PanZoom and DrawROI
    void clearRoi();
    // void cropToRoi(); // Placeholder for future crop functionality slot

signals:
    // --- UI Update Signals ---
    void videoLoaded(const QString& filePath, int totalFrames, double fps, QSize frameSize);
    void videoLoadFailed(const QString& filePath, const QString& errorMessage);
    void frameChanged(int currentFrameNumber, const QImage& currentFrame);
    void playbackStateChanged(bool isPlaying);
    void roiDefined(const QRectF &roi); // ROI in original video coordinates
    void zoomFactorChanged(double newZoomFactor);
    void interactionModeChanged(InteractionMode newMode); // Signal when mode changes

protected:
    // --- Qt Event Handlers ---
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void resizeEvent(QResizeEvent *event) override; // Handle widget resize

private slots:
    void processNextFrame();

private:
    // --- Helper Methods ---
    bool openVideoFile(const QString &filePath);
    void displayFrame(int frameNumber);
    void convertCvMatToQImage(const cv::Mat &mat, QImage &qimg);
    QRectF calculateTargetRect() const; // Calculate the drawing rectangle based on zoom/pan
    QPointF mapPointToVideo(const QPointF& widgetPoint) const; // Maps widget coordinates to video coordinates
    QPointF mapPointFromVideo(const QPointF& videoPoint) const; // Maps video coordinates to widget coordinates
    void updateCursorShape(); // Sets cursor based on current mode and state
    void clampPanOffset(); // Keep video view within reasonable bounds


    // --- OpenCV Video Members ---
    cv::VideoCapture videoCapture;
    cv::Mat currentCvFrame;

    // --- Qt Display Members ---
    QImage currentQImageFrame;
    QTimer *playbackTimer;

    // --- Video Properties ---
    QString currentFilePath;
    int totalFramesCount;
    double framesPerSecond;
    int currentFrameIdx;
    QSize originalFrameSize;

    // --- Playback State ---
    bool m_isPlaying;

    // --- Interaction State ---
    InteractionMode m_currentMode;
    bool m_isPanning;             // True if currently dragging to pan
    QPointF m_lastMousePos;       // Last mouse position for panning delta

    // --- ROI Selection Members ---
    // Note: roiSelectionActive flag removed, use m_currentMode == InteractionMode::DrawROI instead
    QRectF currentRoiRect;        // ROI in original video coordinates (QRectF for precision)
    QPoint roiStartPoint;         // Mouse press point for ROI in widget coordinates
    QPoint roiEndPoint;           // Mouse move/release point for ROI in widget coordinates
    bool roiBeingDefined;         // True while mouse is pressed and moving for ROI

    // --- Zoom & Pan Members ---
    double m_zoomFactor;
    QPointF m_panOffset;          // Pan offset: Translation applied *after* scaling around center.
        // Represents shift of the video relative to centered view.
};

#endif // VIDEOLOADER_H
