#include "trackingthread.h"
#include "worm.h" // For m_wormId, not strictly needed in .cpp if only ID is used

#include <QDebug>
#include <QElapsedTimer> // For potential FPS control or performance measurement

// OpenCV includes
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> // For debugging (imshow) if needed, remove for final

// Helper function for applying thresholding (similar to VideoLoader's)
// This could be moved to a shared utility class/namespace.


namespace TrackingUtils {
cv::Mat applyThresholdingToFrame(const cv::Mat& inputFrame, const QVariantMap& params) {
    if (inputFrame.empty()) return cv::Mat();

    cv::Mat grayFrame;
    if (inputFrame.channels() == 3 || inputFrame.channels() == 4) {
        cv::cvtColor(inputFrame, grayFrame, cv::COLOR_BGR2GRAY);
    } else {
        grayFrame = inputFrame.clone();
    }

    // Gaussian blur - kernel size and sigma might be part of params
    int blurKernelSize = params.value("blurKernelSize", 5).toInt();
    if (blurKernelSize < 3) blurKernelSize = 3;
    if (blurKernelSize % 2 == 0) blurKernelSize++; // Must be odd
    cv::GaussianBlur(grayFrame, grayFrame, cv::Size(blurKernelSize, blurKernelSize), 0);

    ThresholdAlgorithm algorithm = static_cast<ThresholdAlgorithm>(params.value("algorithm", static_cast<int>(ThresholdAlgorithm::Global)).toInt());
    int thresholdValue = params.value("value", 127).toInt();
    bool lightBackground = params.value("lightBackground", true).toBool();
    int adaptiveBlockSize = params.value("adaptiveBlockSize", 11).toInt();
    double adaptiveC = params.value("adaptiveC", 2.0).toDouble();

    int cvThresholdType = lightBackground ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;
    cv::Mat thresholdedFrame;

    switch (algorithm) {
    case ThresholdAlgorithm::Global:
        cv::threshold(grayFrame, thresholdedFrame, thresholdValue, 255, cvThresholdType);
        break;
    case ThresholdAlgorithm::Otsu:
        cv::threshold(grayFrame, thresholdedFrame, 0, 255, cvThresholdType | cv::THRESH_OTSU);
        break;
    case ThresholdAlgorithm::AdaptiveMean:
        cv::adaptiveThreshold(grayFrame, thresholdedFrame, 255,
                              cv::ADAPTIVE_THRESH_MEAN_C, cvThresholdType,
                              adaptiveBlockSize, adaptiveC);
        break;
    case ThresholdAlgorithm::AdaptiveGaussian:
        cv::adaptiveThreshold(grayFrame, thresholdedFrame, 255,
                              cv::ADAPTIVE_THRESH_GAUSSIAN_C, cvThresholdType,
                              adaptiveBlockSize, adaptiveC);
        break;
    default:
        cv::threshold(grayFrame, thresholdedFrame, thresholdValue, 255, cvThresholdType);
        break;
    }
    return thresholdedFrame;
}
} // namespace TrackingUtils


TrackingThread::TrackingThread(const QString& videoPath, int wormId, const QRectF& initialRoi,
                               int keyFrame, TrackingDirection direction,
                               const QVariantMap& trackingParameters,
                               int totalFrames, double fps,
                               QObject *parent)
    : QThread(parent),
    m_videoPath(videoPath),
    m_wormId(wormId),
    m_currentRoi(initialRoi), // Initial ROI is the starting point
    m_currentCentroid(initialRoi.center()), // Initial centroid
    m_previousCentroid(initialRoi.center()), // Initialize previous centroid
    m_keyFrame(keyFrame),
    m_direction(direction),
    m_trackingParameters(trackingParameters),
    m_videoCapture(nullptr),
    m_running(false),
    m_totalFrames(totalFrames),
    m_fps(fps > 0 ? fps : 25.0) // Ensure FPS is positive
{
}

TrackingThread::~TrackingThread() {
    stopTracking(); // Ensure thread is stopped
    wait();         // Wait for run() to finish
    if (m_videoCapture) {
        m_videoCapture->release();
        delete m_videoCapture;
        m_videoCapture = nullptr;
    }
    qDebug() << "TrackingThread for worm" << m_wormId << (m_direction == TrackingDirection::Forward ? "Forward" : "Backward") << "destroyed.";
}

void TrackingThread::stopTracking() {
    QMutexLocker locker(&m_mutex); // Assuming you add a QMutex m_mutex; if m_running is accessed from outside run()
    m_running = false;
}

bool TrackingThread::initializeVideoCapture() {
    m_videoCapture = new cv::VideoCapture();
    try {
        if (!m_videoCapture->open(m_videoPath.toStdString())) {
            emit trackingError(m_wormId, m_direction, "Failed to open video file in thread.");
            delete m_videoCapture;
            m_videoCapture = nullptr;
            return false;
        }
    } catch (const cv::Exception& ex) {
        emit trackingError(m_wormId, m_direction, QString("OpenCV Exception opening video: %1").arg(ex.what()));
        delete m_videoCapture;
        m_videoCapture = nullptr;
        return false;
    }
    // m_totalFrames = static_cast<int>(m_videoCapture->get(cv::CAP_PROP_FRAME_COUNT)); // Already passed in
    // m_fps = m_videoCapture->get(cv::CAP_PROP_FPS); // Already passed in
    return true;
}

void TrackingThread::run() {
    { // Scope for mutex lock if used for m_running
        // QMutexLocker locker(&m_mutex);
        m_running = true;
    }

    if (!initializeVideoCapture()) {
        m_running = false; // Ensure m_running is false if init fails
        emit trackingCompleted(m_wormId, m_direction); // Or a specific error signal
        return;
    }

    int currentFrameNumber;
    int endFrame;
    int step;

    if (m_direction == TrackingDirection::Forward) {
        currentFrameNumber = m_keyFrame; // Start from keyframe
        endFrame = m_totalFrames;
        step = 1;
    } else { // Backward
        currentFrameNumber = m_keyFrame; // Start from keyframe
        endFrame = -1; // Loop until frame 0 (or -1 to exit loop)
        step = -1;
    }

    qDebug() << "Worm" << m_wormId << (m_direction == TrackingDirection::Forward ? "Forward" : "Backward")
             << "tracking started. Keyframe:" << m_keyFrame << "Total frames:" << m_totalFrames;

    QElapsedTimer frameTimer; // To control processing speed if necessary
    long long frameDurationMs = static_cast<long long>(1000.0 / m_fps);


    // Seek to the starting frame (keyFrame)
    if (m_keyFrame >= 0 && m_keyFrame < m_totalFrames) {
        if (!m_videoCapture->set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(m_keyFrame))) {
            emit trackingError(m_wormId, m_direction, QString("Failed to seek to keyframe: %1").arg(m_keyFrame));
            m_running = false;
        }
    } else {
        emit trackingError(m_wormId, m_direction, QString("Invalid keyframe: %1").arg(m_keyFrame));
        m_running = false;
    }


    // Process the keyframe itself first to establish initial state if needed,
    // or assume initialRoi and initialCentroid are for the keyframe.
    // For this loop, we start processing *from* the keyframe.

    while (m_running) {
        frameTimer.start();

        if ((step == 1 && currentFrameNumber >= endFrame) || (step == -1 && currentFrameNumber <= endFrame)) {
            break; // Reached end of tracking range
        }

        // For backward tracking, we read the frame *then* decrement.
        // For forward tracking, we read the frame *then* increment.
        // The first frame to process is currentFrameNumber (which is m_keyFrame initially).

        cv::Mat rawFrame;
        // For backward, if currentFrameNumber is m_keyFrame, we want to process it.
        // If it's m_keyFrame - 1, we seek to m_keyFrame - 1.
        // cv::VideoCapture::read advances the position.
        // So for backward, we need to seek to `currentFrameNumber` *before* reading.
        // For forward, if we just processed `currentFrameNumber`, the next is `currentFrameNumber + 1`.

        if (m_direction == TrackingDirection::Backward) {
            if (currentFrameNumber < 0) break; // Safety for backward
            if (!m_videoCapture->set(cv::CAP_PROP_POS_FRAMES, static_cast<double>(currentFrameNumber))) {
                qWarning() << "Worm" << m_wormId << "Bwd: Failed to seek to frame" << currentFrameNumber;
                // Decide: emit error and stop, or try to continue?
                break;
            }
        }
        // For forward, after processing currentFrameNumber, the capture is already at currentFrameNumber+1
        // So, no explicit seek is needed unless there was a jump.
        // However, to be robust, always setting position might be safer if other parts could touch m_videoCapture.
        // For simplicity, let's assume linear reads are fine after initial seek for forward.
        // If currentFrameNumber is m_keyFrame, we read it. Next loop, currentFrameNumber is m_keyFrame+1.

        if (!m_videoCapture->read(rawFrame) || rawFrame.empty()) {
            qWarning() << "Worm" << m_wormId << (m_direction == TrackingDirection::Forward ? "Fwd" : "Bwd")
            << ": Failed to read frame or empty frame at" << currentFrameNumber
            << "(capture pos:" << m_videoCapture->get(cv::CAP_PROP_POS_FRAMES) << ")";
            // This might mean end of video or an error
            break;
        }

        // --- Core Tracking Logic ---
        // 1. Pre-process the raw frame within the current ROI (or slightly larger search ROI)
        //    This typically involves applying the same thresholding as used for selection.
        cv::Mat processedFrameForRoi = TrackingUtils::applyThresholdingToFrame(rawFrame, m_trackingParameters);
        if (processedFrameForRoi.empty()) {
            qWarning() << "Worm" << m_wormId << "Empty processed frame for ROI at frame" << currentFrameNumber;
            currentFrameNumber += step;
            continue;
        }


        // 2. Find the worm in the current ROI
        QRectF actualBlobBox; // To be filled by findWormInRoi
        QPointF newCentroid = findWormInRoi(processedFrameForRoi, m_currentRoi, actualBlobBox);

        if (newCentroid.x() < 0) { // Worm not found or error
            qDebug() << "Worm" << m_wormId << "not found in ROI at frame" << currentFrameNumber << ". ROI:" << m_currentRoi;
            // Handle lost track: e.g., expand ROI, use prediction, or stop this track.
            // For now, we'll just continue with the old ROI or a predicted one.
            // If continuously not found, should probably emit an error or stop.
            // For simplicity, let's assume if not found, we don't update centroid/ROI and just proceed.
            // A more robust solution would try to re-acquire.
        } else {
            m_previousCentroid = m_currentCentroid;
            m_currentCentroid = newCentroid;
            // Update m_currentRoi based on newCentroid or actualBlobBox for the next frame
            // This could be centered on newCentroid with a fixed size, or use actualBlobBox with padding.
            if (!actualBlobBox.isNull() && actualBlobBox.isValid()) {
                m_currentRoi = actualBlobBox.adjusted(-5, -5, 5, 5); // Example: Pad the found blob
            } else {
                m_currentRoi = QRectF(m_currentCentroid - QPointF(m_currentRoi.width()/2, m_currentRoi.height()/2), m_currentRoi.size());
            }
        }


        // 3. Emit results
        emit frameProcessed(m_wormId, m_direction, currentFrameNumber, m_currentCentroid, m_currentRoi);

        // 4. Emit cropped image for display (optional)
        if (!rawFrame.empty() && m_currentRoi.isValid() &&
            m_currentRoi.left() >= 0 && m_currentRoi.top() >= 0 &&
            m_currentRoi.right() < rawFrame.cols && m_currentRoi.bottom() < rawFrame.rows &&
            m_currentRoi.width() > 0 && m_currentRoi.height() > 0)
        {
            cv::Rect cvRoi(static_cast<int>(m_currentRoi.x()), static_cast<int>(m_currentRoi.y()),
                           static_cast<int>(m_currentRoi.width()), static_cast<int>(m_currentRoi.height()));
            cv::Mat croppedCvMat = rawFrame(cvRoi); // Crop from original color frame
            QImage qCropImage;
            convertCvMatToQImage(croppedCvMat, qCropImage);
            if (!qCropImage.isNull()) {
                emit newWormCropImage(m_wormId, m_direction, currentFrameNumber, qCropImage.copy()); // copy() for thread safety
            }
        }

        // 5. Predict ROI for the next frame (optional, can also be done at start of next iteration)
        // m_currentRoi = predictNextRoi(m_currentRoi, m_currentCentroid, m_previousCentroid);
        // For simplicity, current m_currentRoi update is basic (based on found blob or centered on centroid).

        currentFrameNumber += step;

        // Control processing speed (crude FPS limiting)
        long elapsed = frameTimer.elapsed();
        if (elapsed < frameDurationMs) {
            QThread::msleep(frameDurationMs - elapsed);
        }
    }

    qDebug() << "Worm" << m_wormId << (m_direction == TrackingDirection::Forward ? "Forward" : "Backward") << "tracking loop finished.";
    if (m_videoCapture) {
        m_videoCapture->release();
        delete m_videoCapture;
        m_videoCapture = nullptr;
    }
    m_running = false;
    emit trackingCompleted(m_wormId, m_direction);
}


// Placeholder for converting cv::Mat to QImage (can be moved to a utility)
void TrackingThread::convertCvMatToQImage(const cv::Mat &mat, QImage &qimg) {
    if (mat.empty()) { qimg = QImage(); return; }
    if (mat.type() == CV_8UC3) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888).rgbSwapped(); }
    else if (mat.type() == CV_8UC1) { qimg = QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8); }
    else { qimg = QImage(); /* Unsupported */ }
}


// --- Conceptual Tracking Logic (Placeholders) ---
QRectF TrackingThread::predictNextRoi(const QRectF& currentRoi, const QPointF& currentCentroid, const QPointF& prevCentroid) {
    // Simple prediction: assume constant velocity or just center ROI on currentCentroid.
    // More advanced: Kalman filter, etc.
    QPointF delta = currentCentroid - prevCentroid;
    QPointF predictedCentroid = currentCentroid + delta; // Simple linear prediction
    return QRectF(predictedCentroid - QPointF(currentRoi.width()/2, currentRoi.height()/2), currentRoi.size());
}

QPointF TrackingThread::findWormInRoi(const cv::Mat& processedFrame, const QRectF& localRoi, QRectF& outActualBlobBox) {
    // This is the core computer vision part.
    // 1. Ensure localRoi is valid and within processedFrame bounds.
    cv::Rect cvLocalRoi(
        static_cast<int>(qBound(0.0, localRoi.x(), static_cast<double>(processedFrame.cols - 1))),
        static_cast<int>(qBound(0.0, localRoi.y(), static_cast<double>(processedFrame.rows - 1))),
        static_cast<int>(localRoi.width()),
        static_cast<int>(localRoi.height())
        );
    // Adjust width/height if ROI goes out of bounds
    if (cvLocalRoi.x + cvLocalRoi.width > processedFrame.cols) {
        cvLocalRoi.width = processedFrame.cols - cvLocalRoi.x;
    }
    if (cvLocalRoi.y + cvLocalRoi.height > processedFrame.rows) {
        cvLocalRoi.height = processedFrame.rows - cvLocalRoi.y;
    }

    if (cvLocalRoi.width <= 0 || cvLocalRoi.height <= 0) {
        return QPointF(-1, -1); // Invalid ROI
    }

    cv::Mat roiMat = processedFrame(cvLocalRoi); // Extract the ROI from the (already thresholded) frame

    // 2. Find contours in the ROI.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roiMat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return QPointF(-1, -1); // No worm found
    }

    // 3. Select the "best" contour (e.g., largest, closest to previous centroid if available, most worm-like shape).
    // For simplicity, let's pick the largest contour.
    double maxArea = 0;
    int largestContourIdx = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            largestContourIdx = i;
        }
    }

    if (largestContourIdx == -1) {
        return QPointF(-1, -1); // Should not happen if contours is not empty
    }

    const auto& bestContour = contours[largestContourIdx];
    cv::Moments M = cv::moments(bestContour);
    if (M.m00 == 0) return QPointF(-1,-1); // Avoid division by zero

    QPointF centroidInRoi( (M.m10 / M.m00) , (M.m01 / M.m00) );

    // Convert centroid back to full frame coordinates
    QPointF centroidInFullFrame = centroidInRoi + localRoi.topLeft();

    // Get bounding box of the found blob
    cv::Rect cvBlobBox = cv::boundingRect(bestContour);
    outActualBlobBox = QRectF(cvBlobBox.x + localRoi.x(), cvBlobBox.y + localRoi.y(),
                              cvBlobBox.width, cvBlobBox.height);


    // More advanced:
    // - Filter contours by size, aspect ratio.
    // - If multiple "good" contours, use proximity to last known position or a more sophisticated matching.
    // - Handle merged blobs (this is where it gets complex with N worms).

    return centroidInFullFrame;
}
