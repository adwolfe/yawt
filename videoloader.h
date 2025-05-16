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

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include "trackingcommon.h" // Contains ThresholdAlgorithm and ThresholdSettings


/**
 * @brief Defines the interaction modes for the VideoLoader widget.
 */
enum class InteractionMode {
    PanZoom,
    DrawROI,
    Crop,
    SelectWorms,
    ViewEditTracks
};

class VideoLoader : public QWidget {
    Q_OBJECT

/**
 * @brief VideoLoader opens and displays videos using OpenCV; it also controls display of tracks, worm ROIs, and interactions thereof.
 * @param parent
 */

    // TODO
    // METHOD FOR DELETING ROIs (from tableview)
    //

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
    double getPlaybackSpeed() const;
    QString getCurrentVideoPath() const;

    // --- Thresholding and Pre-processing Status Getters ---
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
    // --- Control Slots ---
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

    // --- Thresholding & Pre-processing Control Slots ---
    void toggleThresholdView(bool enabled);
    void setThresholdAlgorithm(ThresholdAlgorithm algorithm);
    void setThresholdValue(int value);
    void setAssumeLightBackground(bool isLight);
    void setAdaptiveThresholdBlockSize(int blockSize);
    void setAdaptiveThresholdC(double cValue);
    void setEnableBlur(bool enabled);
    void setBlurKernelSize(int kernelSize);
    void setBlurSigmaX(double sigmaX);

    // --- Slots for Track Display ---
    void setTracksToDisplay(const AllWormTracks& tracks);
    void setVisibleTrackIDs(const QSet<int>& visibleTrackIDs);
    void clearDisplayedTracks();


signals:
    // --- UI Update Signals ---
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
    void trackPointClicked(int wormId, int frameNumber, QPointF videoPoint);


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
    QColor getTrackColor(int trackId) const;


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
    int currentFrameIdx;
    QSize originalFrameSize;

    // --- Playback State ---
    bool m_isPlaying;
    double m_playbackSpeedMultiplier;

    // --- Interaction State ---
    InteractionMode m_currentMode;
    bool m_isPanning;
    QPointF m_lastMousePos;

    // --- ROI & Crop Selection Members ---
    QRectF m_activeRoiRect;
    QPoint m_roiStartPointWidget;
    QPoint m_roiEndPointWidget;
    bool m_isDefiningRoi;

    // --- Worm Selection Members ---
    QList<QPointF> m_selectedCentroids_temp;
    QList<QRectF> m_selectedBounds_temp;

    // --- Track Display Members ---
    AllWormTracks m_allTracksToDisplay;
    QSet<int> m_visibleTrackIDs;
    mutable QMap<int, QColor> m_trackColors; // Made mutable to allow modification in const getTrackColor

    // --- Zoom & Pan Members ---
    double m_zoomFactor;
    QPointF m_panOffset;

    // --- Thresholding & Pre-processing Members ---
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
