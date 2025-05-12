#include "wormtracker.h"
#include <QDebug> // For logging

// A reasonable default size for ROI adjustment, can be made configurable
const QSizeF DEFAULT_ROI_SIZE_MULTIPLIER(3.0, 3.0); // ROI will be roughly 3x worm size
const QSizeF MIN_ROI_SIZE(20, 20); // Minimum ROI size in pixels
const QSizeF DEFAULT_WORM_SIZE_GUESS(10, 2); // A very rough guess for worm dimensions (width, height)

WormTracker::WormTracker(int wormId,
                         QRectF initialRoi,
                         TrackingDirection direction,
                         int videoKeyFrameNum,
                         QObject *parent)
    : QObject(parent),
    m_wormId(wormId),
    m_currentRoi(initialRoi),
    m_lastKnownPosition(initialRoi.center().x(), initialRoi.center().y()),
    m_direction(direction),
    m_videoKeyFrameNum(videoKeyFrameNum),
    m_framesToProcess(nullptr),
    m_trackingActive(false),
    m_estimatedWormSize(DEFAULT_WORM_SIZE_GUESS) // Initialize with a guess
{
    qDebug() << "WormTracker created for worm ID:" << m_wormId << "Direction:" << (direction == TrackingDirection::Forward ? "Forward" : "Backward");
}

WormTracker::~WormTracker() {
    qDebug() << "WormTracker destroyed for worm ID:" << m_wormId;
}

void WormTracker::setFrames(const std::vector<cv::Mat>* frames) {
    m_framesToProcess = frames;
}

void WormTracker::startTracking() {
    if (!m_framesToProcess || m_framesToProcess->empty()) {
        emit errorOccurred(m_wormId, "No frames provided to tracker.");
        emit finished();
        return;
    }

    m_trackingActive = true;
    qDebug() << "Worm ID" << m_wormId << "starting tracking. Frames:" << m_framesToProcess->size();

    for (size_t i = 0; i < m_framesToProcess->size() && m_trackingActive; ++i) {
        if (QThread::currentThread()->isInterruptionRequested()) {
            m_trackingActive = false;
            break;
        }

        const cv::Mat& currentFrame = (*m_framesToProcess)[i];
        if (currentFrame.empty()) {
            qWarning() << "Worm ID" << m_wormId << "encountered empty frame at sequence index" << i;
            continue;
        }

        cv::Point2f foundPosition;
        bool success = processSingleFrame(currentFrame, static_cast<int>(i), m_currentRoi, foundPosition);

        int originalFrameNumber;
        if (m_direction == TrackingDirection::Forward) {
            originalFrameNumber = m_videoKeyFrameNum + static_cast<int>(i);
        } else { // Backward
            // For reversed frames, index 0 is keyframe-1, index 1 is keyframe-2, etc.
            originalFrameNumber = m_videoKeyFrameNum - 1 - static_cast<int>(i);
        }

        if (originalFrameNumber < 0) { // Should not happen if keyframe logic is correct
            qWarning() << "Worm ID" << m_wormId << "calculated negative original frame number:" << originalFrameNumber;
            // Potentially stop or handle this error
        }


        if (success) {
            m_lastKnownPosition = foundPosition;
            emit positionUpdated(m_wormId, originalFrameNumber, QPointF(foundPosition.x, foundPosition.y), m_currentRoi);
            // Basic ROI adjustment - could be more sophisticated
            m_currentRoi = adjustRoi(foundPosition, currentFrame.size(), m_estimatedWormSize);
        } else {
            // Lost track or other issue
            emit stateChanged(m_wormId, WormObject::TrackingState::Lost);
            // For now, we stop tracking this worm if lost. Could implement re-detection.
            // m_trackingActive = false; // Option: stop if lost
            // Or, keep the ROI large and hope it reappears
            qDebug() << "Worm ID" << m_wormId << "lost track at original frame" << originalFrameNumber;
        }

        if (i % 10 == 0) { // Emit progress every 10 frames
            emit progress(m_wormId, static_cast<int>((static_cast<double>(i + 1) / m_framesToProcess->size()) * 100.0));
        }
    }

    if (!m_trackingActive) {
        qDebug() << "Worm ID" << m_wormId << "tracking was stopped or interrupted.";
    } else {
        qDebug() << "Worm ID" << m_wormId << "finished processing all frames.";
    }
    emit progress(m_wormId, 100);
    emit finished();
}

void WormTracker::stopTracking() {
    m_trackingActive = false;
}

/**
 * @brief Processes a single frame to find the worm.
 * @param frame The thresholded image (full frame, not ROI yet).
 * @param sequenceFrameIndex The index in the current (forward/reversed) sequence.
 * @param currentRoi The current ROI for this worm (input), will be updated if worm moves significantly.
 * @param foundPosition Output: The found position of the worm.
 * @return True if worm found, false otherwise.
 *
 * This is a placeholder implementation. You'll need to replace this with
 * your actual blob detection and tracking logic within the ROI.
 */
bool WormTracker::processSingleFrame(const cv::Mat& fullFrame, int /*sequenceFrameIndex*/, QRectF& roiRectF, cv::Point2f& foundPosition) {
    if (fullFrame.empty()) return false;

    // Ensure ROI is within frame boundaries
    cv::Rect roiCv(static_cast<int>(roiRectF.x()), static_cast<int>(roiRectF.y()),
                   static_cast<int>(roiRectF.width()), static_cast<int>(roiRectF.height()));
    roiCv = roiCv & cv::Rect(0, 0, fullFrame.cols, fullFrame.rows);

    if (roiCv.width <= 0 || roiCv.height <= 0) {
        // ROI is invalid or outside frame, try to reset or use last known position
        // For now, signal as lost.
        qWarning() << "Worm ID" << m_wormId << "ROI is invalid:" << roiRectF.x() << roiRectF.y() << roiRectF.width() << roiRectF.height();
        return false;
    }

    cv::Mat roiImage = fullFrame(roiCv);
    if (roiImage.empty()) return false;

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roiImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return false; // No contours found in ROI
    }

    // Find the largest contour (simplistic approach, assumes worm is largest blob in ROI)
    double maxArea = 0;
    int largestContourIdx = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            largestContourIdx = static_cast<int>(i);
        }
    }

    if (largestContourIdx != -1 && maxArea > 5) { // Min area threshold
        cv::Moments mu = cv::moments(contours[largestContourIdx]);
        if (mu.m00 > 0) { // Avoid division by zero
            // Position relative to ROI top-left
            cv::Point2f roiRelativePosition(static_cast<float>(mu.m10 / mu.m00),
                                            static_cast<float>(mu.m01 / mu.m00));
            // Convert to full frame coordinates
            foundPosition.x = static_cast<float>(roiCv.x + roiRelativePosition.x);
            foundPosition.y = static_cast<float>(roiCv.y + roiRelativePosition.y);

            // Basic update of estimated worm size from bounding box of contour
            cv::Rect br = cv::boundingRect(contours[largestContourIdx]);
            m_estimatedWormSize.setWidth(br.width);
            m_estimatedWormSize.setHeight(br.height);

            return true;
        }
    }
    return false;
}

/**
 * @brief Adjusts the ROI based on the worm's current position and estimated size.
 * @param wormCenter Center of the worm in full frame coordinates.
 * @param frameSize Size of the full video frame.
 * @param wormSizeGuess A QSizeF representing the estimated width and height of the worm.
 * @return A new QRectF for the ROI.
 */
QRectF WormTracker::adjustRoi(const cv::Point2f& wormCenter, const cv::Size& frameSize, const QSizeF& wormSizeGuess) {
    // Make ROI larger than the worm itself to allow for movement and prevent clipping
    qreal roiWidth = qMax(MIN_ROI_SIZE.width(), wormSizeGuess.width() * DEFAULT_ROI_SIZE_MULTIPLIER.width());
    qreal roiHeight = qMax(MIN_ROI_SIZE.height(), wormSizeGuess.height() * DEFAULT_ROI_SIZE_MULTIPLIER.height());

    // Center the ROI around the worm
    qreal roiX = wormCenter.x - roiWidth / 2.0;
    qreal roiY = wormCenter.y - roiHeight / 2.0;

    // Clamp ROI to frame boundaries
    roiX = qMax(0.0, qMin(roiX, static_cast<qreal>(frameSize.width) - roiWidth));
    roiY = qMax(0.0, qMin(roiY, static_cast<qreal>(frameSize.height) - roiHeight));

    // Ensure width/height are not extending beyond frame boundaries if clamped at 0,0
    roiWidth = qMin(roiWidth, static_cast<qreal>(frameSize.width) - roiX);
    roiHeight = qMin(roiHeight, static_cast<qreal>(frameSize.height) - roiY);


    return QRectF(roiX, roiY, roiWidth, roiHeight);
}

