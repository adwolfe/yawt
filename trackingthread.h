#ifndef TRACKINGTHREAD_H
#define TRACKINGTHREAD_H

#include <QThread>
#include <QString>
#include <QRectF>
#include <QImage> // For emitting cropped worm images
#include <QVariantMap> // For tracking parameters

// Forward declare OpenCV types to avoid including heavy headers here if possible
namespace cv {
class Mat;
class VideoCapture;
}

class Worm; // Forward declaration

/**
 * @brief The TrackingDirection enum defines whether the tracking is forward or backward.
 */
enum class TrackingDirection {
    Forward,
    Backward
};

/**
 * @brief The TrackingThread class performs frame-by-frame tracking for a single worm
 * in a specific direction (forward or backward from a keyframe).
 */
class TrackingThread : public QThread {
    Q_OBJECT
public:
    explicit TrackingThread(const QString& videoPath, int wormId, const QRectF& initialRoi,
                            int keyFrame, TrackingDirection direction,
                            const QVariantMap& trackingParameters, // e.g., threshold settings
                            int totalFrames, double fps, // Pass video properties
                            QObject *parent = nullptr);
    ~TrackingThread();

    void stopTracking(); // Request the thread to stop

signals:
    void frameProcessed(int wormId, TrackingDirection direction, int frameNumber, const QPointF& newCentroid, const QRectF& newRoi);
    void newWormCropImage(int wormId, TrackingDirection direction, int frameNumber, const QImage& cropImage); // For live display
    void trackingCompleted(int wormId, TrackingDirection direction);
    void trackingError(int wormId, TrackingDirection direction, const QString& errorMessage);

protected:
    void run() override;

private:
    // --- Tracking Logic ---
    bool initializeVideoCapture();
    cv::Mat getFrameForProcessing(int frameNumber); // Gets frame and applies necessary pre-processing like thresholding
    QRectF predictNextRoi(const QRectF& currentRoi, const QPointF& currentCentroid, const QPointF& prevCentroid);
    QPointF findWormInRoi(const cv::Mat& processedFrame, const QRectF& localRoi, QRectF& outActualBlobBox); // This is the core CV logic, outputs actual blob box
    void convertCvMatToQImage(const cv::Mat &mat, QImage &qimg); // Local utility

    QString m_videoPath;
    int m_wormId;
    QRectF m_currentRoi;         // Current local ROI in video coordinates
    QPointF m_currentCentroid;   // Current worm centroid in video coordinates
    QPointF m_previousCentroid;  // Previous worm centroid for simple prediction
    int m_keyFrame;
    TrackingDirection m_direction;
    QVariantMap m_trackingParameters; // Holds threshold settings, etc.

    cv::VideoCapture* m_videoCapture; // Each thread gets its own VideoCapture
    bool m_running;
    int m_totalFrames; // Total frames in the video
    double m_fps;      // Video FPS
};

#endif // TRACKINGTHREAD_H
