// WormTracker.cpp
#include "wormtracker.h" // Lowercase include
#include <QDebug>
#include <QtMath> // For qSqrt, qPow

// Default parameters for blob plausibility (these should ideally be configurable)
const double DEFAULT_MIN_BLOB_AREA = 10.0;
const double DEFAULT_MAX_BLOB_AREA = 500.0; // Adjust based on expected worm size and magnification
const double DEFAULT_MIN_ASPECT_RATIO = 0.1; // height/width or width/height
const double DEFAULT_MAX_ASPECT_RATIO = 10.0;

const QSizeF DEFAULT_ROI_SIZE_MULTIPLIER_TRACK(2.5, 2.5); // ROI will be roughly 2.5x worm size during tracking
const QSizeF MIN_ROI_SIZE_TRACK(30, 30); // Minimum ROI size in pixels during tracking

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
    m_currentState(TrackerState::Idle),
    m_estimatedWormSize(initialRoi.size()), // Initial guess from selection ROI
    m_minBlobArea(DEFAULT_MIN_BLOB_AREA),
    m_maxBlobArea(DEFAULT_MAX_BLOB_AREA),
    m_minAspectRatio(DEFAULT_MIN_ASPECT_RATIO),
    m_maxAspectRatio(DEFAULT_MAX_ASPECT_RATIO)
{
    qDebug() << "WormTracker created for worm ID:" << m_wormId << "Direction:" << (direction == TrackingDirection::Forward ? "Forward" : "Backward");
    m_lastPrimaryBlob.isValid = false;
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
    m_currentState = TrackerState::TrackingSingle; // Start by assuming we are tracking a single entity
    qDebug() << "Worm ID" << m_wormId << "starting tracking. Frames:" << m_framesToProcess->size() << "Initial ROI:" << m_currentRoi;

    for (size_t i = 0; i < m_framesToProcess->size() && m_trackingActive; ++i) {
        if (QThread::currentThread()->isInterruptionRequested()) {
            qDebug() << "Worm ID" << m_wormId << "tracking interrupted.";
            m_trackingActive = false;
            break;
        }

        // If paused by split detection, loop here until resumed or stopped
        while(m_currentState == TrackerState::PausedAwaitingSplitDecision && m_trackingActive) {
            QThread::msleep(50); // Sleep briefly to avoid busy-waiting
            if (QThread::currentThread()->isInterruptionRequested()) {
                m_trackingActive = false;
                break;
            }
        }
        if (!m_trackingActive) break;


        const cv::Mat& currentFrame = (*m_framesToProcess)[i];
        if (currentFrame.empty()) {
            qWarning() << "Worm ID" << m_wormId << "encountered empty frame at sequence index" << i;
            continue;
        }

        cv::Point2f foundPosition; // This will be the primary target's position
        // processSingleFrame will now handle more complex logic and update m_currentState
        bool success = processSingleFrame(currentFrame, static_cast<int>(i), m_currentRoi, foundPosition);

        int originalFrameNumber;
        if (m_direction == TrackingDirection::Forward) {
            originalFrameNumber = m_videoKeyFrameNum + static_cast<int>(i);
        } else {
            originalFrameNumber = m_videoKeyFrameNum - 1 - static_cast<int>(i);
        }
        if (originalFrameNumber < 0 && totalFramesCount > 0) { // totalFramesCount from VideoLoader (passed if needed)
            qWarning() << "Worm ID" << m_wormId << "calculated negative original frame number:" << originalFrameNumber << "SeqIdx:" << i << "Keyframe:" << m_videoKeyFrameNum;
            // This can happen if keyframe is 0 and we are going backward.
            // For now, let it proceed, but the frame number needs to be handled carefully by manager.
        }


        if (success && m_currentState != TrackerState::PausedAwaitingSplitDecision) {
            // Position and ROI are updated within processSingleFrame or by resumeTrackingWithNewTarget
            // The positionUpdated signal is also emitted from within processSingleFrame now.
        } else if (m_currentState != TrackerState::PausedAwaitingSplitDecision) { // Not successful and not paused
            qDebug() << "Worm ID" << m_wormId << "lost track at original frame" << originalFrameNumber << "(seq index" << i << ")";
            emit stateChanged(m_wormId, WormObject::TrackingState::Lost); // Update WormObject state via TrackingManager
            // Decide if we should stop tracking this worm entirely if lost for too long
            // For now, it might continue with a wider ROI or stop if ROI becomes invalid
        }

        if (i % 20 == 0 || i == m_framesToProcess->size() -1 ) { // Emit progress periodically
            emit progress(m_wormId, static_cast<int>((static_cast<double>(i + 1) / m_framesToProcess->size()) * 100.0));
        }
    }

    qDebug() << "Worm ID" << m_wormId << "finished processing loop. Active:" << m_trackingActive << "State:" << static_cast<int>(m_currentState);
    emit progress(m_wormId, 100);
    emit finished(); // Signal that this tracker instance's work is done
}

void WormTracker::stopTracking() {
    qDebug() << "Worm ID" << m_wormId << "stopTracking() called.";
    m_trackingActive = false;
    // If paused, this will break the wait loop in startTracking()
}

void WormTracker::resumeTrackingWithNewTarget(const TrackingHelper::DetectedBlob& targetBlob) {
    qDebug() << "Worm ID" << m_wormId << "resuming tracking with new target. Centroid:" << targetBlob.centroid;
    if (!targetBlob.isValid) {
        qWarning() << "Worm ID" << m_wormId << "resumeTrackingWithNewTarget called with invalid blob.";
        m_currentState = TrackerState::TrackingSingle; // Or try to re-acquire
        m_trackingActive = false; // Could stop if no valid target
        return;
    }
    m_lastKnownPosition = cv::Point2f(static_cast<float>(targetBlob.centroid.x()), static_cast<float>(targetBlob.centroid.y()));
    m_currentRoi = targetBlob.boundingBox; // Or adjustRoi based on it
    m_estimatedWormSize = targetBlob.boundingBox.size();
    m_lastPrimaryBlob = targetBlob; // Store this as the new primary
    m_currentState = TrackerState::TrackingSingle; // Resume normal tracking
    // No need to set m_trackingActive = true here, the main loop in startTracking continues if it was true.
}

void WormTracker::confirmTargetIsMerged(int mergedEntityID, const QPointF& mergedBlobCentroid, const QRectF& mergedBlobRoi) {
    qDebug() << "Worm ID" << m_wormId << "confirmed as part of merged entity" << mergedEntityID;
    m_currentState = TrackerState::TrackingMerged;
    m_lastKnownPosition = cv::Point2f(static_cast<float>(mergedBlobCentroid.x()), static_cast<float>(mergedBlobCentroid.y()));
    m_currentRoi = mergedBlobRoi; // Manager dictates the ROI for the merged blob
    m_estimatedWormSize = mergedBlobRoi.size(); // Update estimated size to the merged blob
    // The tracker will now use this larger ROI and expect a single large blob.
}


QList<TrackingHelper::DetectedBlob> WormTracker::findPlausibleBlobsInRoi(
    const cv::Mat& fullFrame, const QRectF& roi,
    double minArea, double maxArea,
    double minAspectRatio, double maxAspectRatio) {

    QList<TrackingHelper::DetectedBlob> plausibleBlobs;
    if (fullFrame.empty() || roi.isEmpty() || roi.width() <=0 || roi.height() <=0 ) {
        return plausibleBlobs;
    }

    cv::Rect roiCv(static_cast<int>(roi.x()), static_cast<int>(roi.y()),
                   static_cast<int>(roi.width()), static_cast<int>(roi.height()));
    roiCv &= cv::Rect(0, 0, fullFrame.cols, fullFrame.rows); // Clamp to frame

    if (roiCv.width <= 0 || roiCv.height <= 0) return plausibleBlobs;

    cv::Mat roiImage = fullFrame(roiCv);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roiImage.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // Use clone as findContours can modify

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < minArea || area > maxArea) continue;

        cv::Rect br = cv::boundingRect(contour);
        if (br.width == 0 || br.height == 0) continue;

        double aspectRatio = static_cast<double>(br.width) / br.height;
        if (aspectRatio < 1.0) aspectRatio = 1.0 / aspectRatio; // Ensure aspect ratio is >= 1

        if (aspectRatio < minAspectRatio || aspectRatio > maxAspectRatio) continue;

        cv::Moments mu = cv::moments(contour);
        if (mu.m00 > 0) {
            TrackingHelper::DetectedBlob blob;
            blob.centroid = QPointF(roiCv.x + mu.m10 / mu.m00, roiCv.y + mu.m01 / mu.m00);
            blob.boundingBox = QRectF(roiCv.x + br.x, roiCv.y + br.y, br.width, br.height);
            blob.area = area;
            blob.contourPoints = contour; // Store contour points relative to ROI origin, need to offset if used globally
            for(cv::Point& pt : blob.contourPoints) { // Offset points to be in full frame coordinates
                pt.x += roiCv.x;
                pt.y += roiCv.y;
            }
            blob.isValid = true;
            plausibleBlobs.append(blob);
        }
    }
    return plausibleBlobs;
}


bool WormTracker::processSingleFrame(const cv::Mat& frame, int sequenceFrameIndex, QRectF& roiInOut, cv::Point2f& foundPositionOut) {
    Q_UNUSED(sequenceFrameIndex); // May not be needed directly here

    QList<TrackingHelper::DetectedBlob> blobs = findPlausibleBlobsInRoi(frame, roiInOut, m_minBlobArea, m_maxBlobArea, m_minAspectRatio, m_maxAspectRatio);
    int plausibleBlobsFound = blobs.count();
    TrackingHelper::DetectedBlob primaryTarget;
    primaryTarget.isValid = false;

    int originalFrameNumber;
    if (m_direction == TrackingDirection::Forward) {
        originalFrameNumber = m_videoKeyFrameNum + static_cast<int>(sequenceFrameIndex);
    } else {
        originalFrameNumber = m_videoKeyFrameNum - 1 - static_cast<int>(sequenceFrameIndex);
    }

    if (plausibleBlobsFound == 0) {
        m_currentState = TrackerState::TrackingSingle; // Or a "Lost" state?
        m_lastPrimaryBlob.isValid = false;
        emit positionUpdated(m_wormId, originalFrameNumber, QPointF(m_lastKnownPosition.x, m_lastKnownPosition.y), roiInOut, 0, 0.0);
        return false; // No blob found
    } else if (plausibleBlobsFound == 1) {
        primaryTarget = blobs.first();
        m_currentState = TrackerState::TrackingSingle; // Confidently tracking one
    } else { // plausibleBlobsFound > 1
        // Multiple blobs found. Decide if it's a split or just ambiguity.
        if (m_currentState == TrackerState::TrackingMerged || (m_lastPrimaryBlob.isValid && m_lastPrimaryBlob.area > m_maxBlobArea * 0.8) ) { // If previously tracking a large (merged) blob
            qDebug() << "Worm ID" << m_wormId << "was tracking merged/large, now sees" << plausibleBlobsFound << "blobs. Potential split.";
            m_currentState = TrackerState::PausedAwaitingSplitDecision;
            emit splitDetectedAndPaused(m_wormId, originalFrameNumber, blobs);
            m_lastPrimaryBlob.isValid = false; // Reset last primary as we are paused
            return false; // Paused, no primary target chosen by this tracker
        } else {
            // Ambiguity: multiple small blobs. Pick the closest to the last known position.
            m_currentState = TrackerState::PotentialMergeOrSplit;
            double minDistanceSq = std::numeric_limits<double>::max();
            for (const auto& blob : blobs) {
                double dx = blob.centroid.x() - m_lastKnownPosition.x;
                double dy = blob.centroid.y() - m_lastKnownPosition.y;
                double distSq = dx * dx + dy * dy;
                if (distSq < minDistanceSq) {
                    minDistanceSq = distSq;
                    primaryTarget = blob;
                }
            }
        }
    }

    if (primaryTarget.isValid) {
        m_lastKnownPosition = cv::Point2f(static_cast<float>(primaryTarget.centroid.x()), static_cast<float>(primaryTarget.centroid.y()));
        foundPositionOut = m_lastKnownPosition;
        m_estimatedWormSize = primaryTarget.boundingBox.size(); // Update estimated size
        roiInOut = adjustRoi(m_lastKnownPosition, frame.size(), m_estimatedWormSize); // Adjust ROI for next frame
        m_lastPrimaryBlob = primaryTarget;

        emit positionUpdated(m_wormId, originalFrameNumber, primaryTarget.centroid, roiInOut, plausibleBlobsFound, primaryTarget.area);
        return true;
    }

    // If no primary target was chosen (e.g. in PausedAwaitingSplitDecision or if all blobs were filtered out by some other logic)
    m_lastPrimaryBlob.isValid = false;
    emit positionUpdated(m_wormId, originalFrameNumber, QPointF(m_lastKnownPosition.x, m_lastKnownPosition.y), roiInOut, plausibleBlobsFound, 0.0); // Report last known if nothing better
    return false;
}


QRectF WormTracker::adjustRoi(const cv::Point2f& wormCenter, const cv::Size& frameSize, const QSizeF& wormSizeGuess) {
    qreal roiWidth = qMax(MIN_ROI_SIZE_TRACK.width(), wormSizeGuess.width() * DEFAULT_ROI_SIZE_MULTIPLIER_TRACK.width());
    qreal roiHeight = qMax(MIN_ROI_SIZE_TRACK.height(), wormSizeGuess.height() * DEFAULT_ROI_SIZE_MULTIPLIER_TRACK.height());
    qreal roiX = wormCenter.x - roiWidth / 2.0;
    qreal roiY = wormCenter.y - roiHeight / 2.0;
    roiX = qMax(0.0, qMin(roiX, static_cast<qreal>(frameSize.width) - roiWidth));
    roiY = qMax(0.0, qMin(roiY, static_cast<qreal>(frameSize.height) - roiHeight));
    roiWidth = qMin(roiWidth, static_cast<qreal>(frameSize.width) - roiX);
    roiHeight = qMin(roiHeight, static_cast<qreal>(frameSize.height) - roiY);
    return QRectF(roiX, roiY, roiWidth, roiHeight);
}
