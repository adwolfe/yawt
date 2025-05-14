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
#include <QMap>  // For AllWormTracks (if it's a QMap typedef)
#include <QSet>  // For visible track IDs
#include <vector> // For std::vector in AllWormTracks

// OpenCV Headers
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include "trackingcommon.h" // Contains ThresholdAlgorithm and ThresholdSettings
#include "trackeditemdata.h"  // Contains AllWormTracks, WormTrackPoint
#include "trackdata.h"

/**
 * @brief Defines the interaction modes for the VideoLoader widget.
 */
enum class InteractionMode {
    PanZoom,
    DrawROI,
    Crop,
    SelectWorms,
    ViewEditTracks // New mode for viewing and interacting with tracks
};

// ThresholdAlgorithm enum is defined in TrackingCommon.h

class VideoLoader : public QWidget {
    Q_OBJECT

public:
    explicit VideoLoader(QWidget *parent = nullptr);
    ~VideoLoader();

    // --- Public Methods (existing) ---
    bool isVideoLoaded() const;
    int getTotalFrames() const;
    double getFPS() const;
    int getCurrentFrameNumber() const;
    QSize getVideoFrameSize() const;
    double getZoomFactor() const;
    QRectF getCurrentRoi() const;
    InteractionMode getCurrentInteractionMode() const;
    double getPlaybackSpeed() const;
    QString getCurrentVideoPath() const; // Added this getter in a previous step

    // --- Thresholding and Pre-processing Status Getters (existing) ---
    bool isThresholdViewEnabled() const;
    ThresholdSettings getCurrentThresholdSettings() const;
    ThresholdAlgorithm getCurrentThresholdAlgorithm() const;
    int getThresholdValue() const;
    bool getAssumeLightBackground() const;
    int getAdaptiveBlockSize() const;
    double getAdaptiveCValue() const;
    bool isBlurEnabled() const;
    int getBlurKernelSize() const;
    double getBlurSigmaX() const;


public slots:
    // --- Control Slots (existing) ---
    bool loadVideo(const QString &filePath);
    void play();
    void pause();
    void seekToFrame(int frameNumber, bool suppressEmit = false);
    void setZoomFactor(double factor);
    void setZoomFactorAtPoint(double factor, const QPointF& widgetPoint);
    void setInteractionMode(InteractionMode mode);
    void clearRoi();
    void setPlaybackSpeed(double multiplier);
    void clearWormSelections();

    // --- Thresholding & Pre-processing Control Slots (existing) ---
    void toggleThresholdView(bool enabled);
    void setThresholdAlgorithm(ThresholdAlgorithm algorithm);
    void setThresholdValue(int value);
    void setAssumeLightBackground(bool isLight);
    void setAdaptiveThresholdBlockSize(int blockSize);
    void setAdaptiveThresholdC(double cValue);
    void setEnableBlur(bool enabled);
    void setBlurKernelSize(int kernelSize);
    void setBlurSigmaX(double sigmaX);

    // --- New Slots for Track Display ---
    /**
     * @brief Sets the complete track data to be available for display.
     * Call this when tracking is finished and results are ready.
     * @param tracks The map of worm IDs to their track points.
     */
    void setTracksToDisplay(const AllWormTracks& tracks);

    /**
     * @brief Updates which tracks should be currently visible.
     * @param visibleTrackIDs A set of worm IDs that should be drawn.
     */
    void setVisibleTrackIDs(const QSet<int>& visibleTrackIDs);

    /**
     * @brief Clears all displayed tracks.
     */
    void clearDisplayedTracks();


signals:
    // --- UI Update Signals (existing) ---
    void videoLoaded(const QString& filePath, int totalFrames, double fps, QSize frameSize);
    void videoLoadFailed(const QString& filePath, const QString& errorMessage);
    void videoProcessingStarted(const QString& message);
    void videoProcessingFinished(const QString& message, bool success);
    void frameChanged(int currentFrameNumber, const QImage& currentFrame);
    void playbackStateChanged(bool isPlaying, double currentSpeed);
    void roiDefined(const QRectF &roi);
    void zoomFactorChanged(double newZoomFactor);
    void interactionModeChanged(InteractionMode newMode);
    void thresholdParametersChanged(const ThresholdSettings& newSettings);
    void playbackSpeedChanged(double newSpeedMultiplier);
    void wormBlobSelected(const QPointF& centroidVideoCoords, const QRectF& boundingRectVideoCoords);

    // --- New Signals for Track Interaction ---
    /**
     * @brief Emitted when a point on a displayed track is clicked.
     * @param wormId The ID of the worm whose track was clicked.
     * @param frameNumber The frame number corresponding to the clicked track point.
     * @param videoPoint The coordinates of the clicked track point in the video's coordinate system.
     */
    void trackPointClicked(int wormId, int frameNumber, QPointF videoPoint);


protected:
    // --- Qt Event Handlers (existing) ---
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override; // May need for draggable marker later
    void mouseReleaseEvent(QMouseEvent *event) override; // May need for draggable marker later
    void wheelEvent(QWheelEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void processNextFrame();

private:
    // --- Helper Methods (existing and new) ---
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
    void updateTimerInterval();
    void emitThresholdParametersChanged();
    QColor getTrackColor(int trackId) const; // Helper to get distinct colors for tracks


    // --- OpenCV Video Members (existing) ---
    cv::VideoCapture videoCapture;
    cv::Mat currentCvFrame;
    cv::Mat m_thresholdedFrame_mono;

    // --- Qt Display Members (existing) ---
    QImage currentQImageFrame;
    QTimer *playbackTimer;

    // --- Video Properties (existing) ---
    QString currentFilePath;
    int totalFramesCount;
    double framesPerSecond;
    int currentFrameIdx;
    QSize originalFrameSize;

    // --- Playback State (existing) ---
    bool m_isPlaying;
    double m_playbackSpeedMultiplier;

    // --- Interaction State (existing) ---
    InteractionMode m_currentMode;
    bool m_isPanning;
    QPointF m_lastMousePos;

    // --- ROI & Crop Selection Members (existing) ---
    QRectF m_activeRoiRect;
    QPoint m_roiStartPointWidget;
    QPoint m_roiEndPointWidget;
    bool m_isDefiningRoi;

    // --- Worm Selection Members (existing for drawing feedback) ---
    QList<QPointF> m_selectedCentroids_temp;
    QList<QRectF> m_selectedBounds_temp;

    // --- Track Display Members (New) ---
    AllWormTracks m_allTracksToDisplay; // Stores all loaded tracks
    QSet<int> m_visibleTrackIDs;        // IDs of tracks currently selected for display
    QMap<int, QColor> m_trackColors;    // Cache for track colors

    // --- Zoom & Pan Members (existing) ---
    double m_zoomFactor;
    QPointF m_panOffset;

    // --- Thresholding & Pre-processing Members (existing) ---
    bool m_showThresholdMask;
    ThresholdAlgorithm m_thresholdAlgorithm;
    int m_thresholdValue;
    bool m_assumeLightBackground;
    int m_adaptiveBlockSize;
    double m_adaptiveC;
    bool m_enableBlur;
    int m_blurKernelSize;
    double m_blurSigmaX;
};

#endif // VIDEOLOADER_H
