// wormtracker.cpp
#include "wormtracker.h" // Lowercase include
#include <QDebug>
#include <QtMath>       // For qSqrt, qPow, qAbs
#include <algorithm>    // For std::sort, std::min_element etc. if needed
#include <limits>       // For std::numeric_limits
#include <QLineF>


// ROI adjustment parameters
const QSizeF DEFAULT_ROI_SIZE_MULTIPLIER_WORMTRACKER(2.5, 2.5); // ROI will be roughly X times worm size
const QSizeF MIN_ROI_SIZE_WORMTRACKER(30, 30);                  // Minimum ROI size in pixels

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
    m_estimatedWormSize(initialRoi.size().isValid() ? initialRoi.size() : QSizeF(20,5)), // Initial guess from selection ROI
    m_minBlobArea(TrackingConstants::DEFAULT_MIN_WORM_AREA),
    m_maxBlobArea(TrackingConstants::DEFAULT_MAX_WORM_AREA),
    m_minAspectRatio(TrackingConstants::DEFAULT_MIN_ASPECT_RATIO),
    m_maxAspectRatio(TrackingConstants::DEFAULT_MAX_ASPECT_RATIO)
{
    qDebug() << "WormTracker (" << this << ") created for worm ID:" << m_wormId
             << "Direction:" << (direction == TrackingDirection::Forward ? "Forward" : "Backward")
             << "Initial ROI:" << initialRoi;
    m_lastPrimaryBlob.isValid = false; // Initialize last tracked blob as invalid
}

WormTracker::~WormTracker() {
    qDebug() << "WormTracker (" << this << ") destroyed for worm ID:" << m_wormId;
}

void WormTracker::setFrames(const std::vector<cv::Mat>* frames) {
    m_framesToProcess = frames;
}

void WormTracker::startTracking() {
    if (!m_framesToProcess || m_framesToProcess->empty()) {
        qWarning() << "WormTracker ID" << m_wormId << ": No frames provided to tracker.";
        emit errorOccurred(m_wormId, "No frames provided to tracker.");
        emit finished();
        return;
    }

    m_trackingActive = true;
    m_currentState = TrackerState::TrackingSingle; // Assume starting by tracking a single target
    m_lastPrimaryBlob.isValid = false; // Reset for new tracking session
    qDebug() << "WormTracker ID" << m_wormId << ": Starting tracking. Frames:" << m_framesToProcess->size()
             << "Initial ROI:" << m_currentRoi;

    for (size_t i = 0; i < m_framesToProcess->size() && m_trackingActive; ++i) {
        if (QThread::currentThread()->isInterruptionRequested()) {
            qDebug() << "WormTracker ID" << m_wormId << ": Tracking interrupted by thread request.";
            m_trackingActive = false; // Ensure loop terminates
            break;
        }

        // If paused by split detection, loop here until resumed or stopped
        while(m_currentState == TrackerState::PausedAwaitingSplitDecision && m_trackingActive) {
            QThread::msleep(30); // Sleep briefly to yield execution
            if (QThread::currentThread()->isInterruptionRequested()) {
                m_trackingActive = false; // Check interruption again
            }
        }
        if (!m_trackingActive) break; // Exit outer loop if stopped during pause


        const cv::Mat& currentFrame = (*m_framesToProcess)[i];
        if (currentFrame.empty()) {
            qWarning() << "WormTracker ID" << m_wormId << ": Encountered empty frame at sequence index" << i;
            continue;
        }

        cv::Point2f primaryTargetPosition;
        // processSingleFrame handles internal state changes and signal emissions
        bool foundTargetThisFrame = processSingleFrame(currentFrame, static_cast<int>(i), m_currentRoi, primaryTargetPosition);

        // Note: positionUpdated and splitDetectedAndPaused are emitted from within processSingleFrame.
        // State changes to WormObject (like Lost) are also signaled from there or by TrackingManager.

        if (!foundTargetThisFrame && m_currentState != TrackerState::PausedAwaitingSplitDecision) {
            // If no target was found and we are not paused waiting for a split decision
            // (e.g., truly lost, not just ambiguous)
            // The positionUpdated signal would have been emitted with 0 plausible blobs.
            // TrackingManager can then decide if this means the WormObject state is "Lost".
            qDebug() << "WormTracker ID" << m_wormId << ": Target lost at sequence index" << i;
            // emit stateChanged(m_wormId, WormObject::TrackingState::Lost); // Let TrackingManager decide this based on patterns
        }

        // Emit progress
        if (i % 10 == 0 || i == m_framesToProcess->size() - 1) {
            emit progress(m_wormId, static_cast<int>((static_cast<double>(i + 1) / m_framesToProcess->size()) * 100.0));
        }
    } // End of frame processing loop

    qDebug() << "WormTracker ID" << m_wormId << ": Finished processing loop. Final state:" << static_cast<int>(m_currentState)
             << "TrackingActive:" << m_trackingActive;
    if (m_trackingActive) { // If loop completed naturally
        emit progress(m_wormId, 100);
    }
    emit finished(); // Signal that this tracker instance's work is done
}

void WormTracker::stopTracking() {
    qDebug() << "WormTracker ID" << m_wormId << ": stopTracking() called. Current state:" << static_cast<int>(m_currentState);
    m_trackingActive = false;
    // If paused, setting m_trackingActive to false will break the wait loop in startTracking()
}

void WormTracker::resumeTrackingWithNewTarget(const TrackingHelper::DetectedBlob& targetBlob) {
    qDebug() << "WormTracker ID" << m_wormId << ": resumeTrackingWithNewTarget called. Centroid:" << targetBlob.centroid;
    if (!targetBlob.isValid) {
        qWarning() << "WormTracker ID" << m_wormId << ": resumeTrackingWithNewTarget called with invalid blob. May stop tracking.";
        m_currentState = TrackerState::Idle; // Or some error state
        m_trackingActive = false; // Stop if no valid target
        return;
    }
    m_lastKnownPosition = cv::Point2f(static_cast<float>(targetBlob.centroid.x()), static_cast<float>(targetBlob.centroid.y()));
    m_currentRoi = targetBlob.boundingBox; // Or adjustRoi based on it for next frame
    m_estimatedWormSize = targetBlob.boundingBox.size().isValid() ? targetBlob.boundingBox.size() : QSizeF(20,5);
    m_lastPrimaryBlob = targetBlob; // This is our new primary target
    m_currentState = TrackerState::TrackingSingle; // Resume normal tracking state
    qDebug() << "WormTracker ID" << m_wormId << ": Resumed. New ROI:" << m_currentRoi;
    // m_trackingActive should already be true if we were paused; otherwise, startTracking loop handles it.
}

void WormTracker::confirmTargetIsMerged(int mergedEntityID, const QPointF& mergedBlobCentroid, const QRectF& mergedBlobRoi) {
    qDebug() << "WormTracker ID" << m_wormId << ": confirmTargetIsMerged. EntityID:" << mergedEntityID << "Centroid:" << mergedBlobCentroid;
    m_currentState = TrackerState::TrackingMerged;
    m_lastKnownPosition = cv::Point2f(static_cast<float>(mergedBlobCentroid.x()), static_cast<float>(mergedBlobCentroid.y()));
    m_currentRoi = mergedBlobRoi; // Manager dictates the ROI for the merged blob
    m_estimatedWormSize = mergedBlobRoi.size().isValid() ? mergedBlobRoi.size() : QSizeF(30,10); // Update estimated size
    m_lastPrimaryBlob.isValid = true; // Assume the merged blob is now the primary
    m_lastPrimaryBlob.centroid = mergedBlobCentroid;
    m_lastPrimaryBlob.boundingBox = mergedBlobRoi;
    // Area might be tricky here, could be passed or roughly estimated
    m_lastPrimaryBlob.area = mergedBlobRoi.width() * mergedBlobRoi.height();
}


QList<TrackingHelper::DetectedBlob> WormTracker::findPlausibleBlobsInRoi(const cv::Mat& fullFrame, const QRectF& roi) {
    // This function now directly uses the TrackingHelper function
    // It passes the tracker's specific plausibility parameters.
    return TrackingHelper::findAllPlausibleBlobsInRoi(fullFrame, roi,
                                                      m_minBlobArea, m_maxBlobArea,
                                                      m_minAspectRatio, m_maxAspectRatio);
}


bool WormTracker::processSingleFrame(const cv::Mat& frame, int sequenceFrameIndex, QRectF& roiInOut, cv::Point2f& foundPositionOut) {
    QList<TrackingHelper::DetectedBlob> blobs = findPlausibleBlobsInRoi(frame, roiInOut);
    int plausibleBlobsFound = blobs.count();
    TrackingHelper::DetectedBlob currentPrimaryTarget; // The blob we decide to follow this frame
    currentPrimaryTarget.isValid = false;

    int originalFrameNumber;
    if (m_direction == TrackingDirection::Forward) {
        originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else {
        originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex;
    }

    if (plausibleBlobsFound == 0) {
        // No plausible blobs found in the current ROI
        if (m_currentState != TrackerState::TrackingMerged) { // If not expecting a large merged blob that might have temporarily vanished
            m_currentState = TrackerState::TrackingSingle; // Revert to single, but it's effectively lost
        }
        m_lastPrimaryBlob.isValid = false;
        emit positionUpdated(m_wormId, originalFrameNumber, QPointF(m_lastKnownPosition.x, m_lastKnownPosition.y), roiInOut, 0, 0.0);
        // Try to expand ROI slightly to re-acquire next frame? For now, just report 0.
        // roiInOut = adjustRoi(m_lastKnownPosition, frame.size(), m_estimatedWormSize * 1.5); // Example expansion
        return false;
    }

    // Select the primary target from the plausible blobs
    if (plausibleBlobsFound == 1) {
        currentPrimaryTarget = blobs.first();
        if (m_currentState == TrackerState::TrackingMerged && currentPrimaryTarget.area < m_maxBlobArea * 0.75) {
            // Was tracking a merged blob, now see a single, smaller blob. Potential split.
            qDebug() << "WormTracker ID" << m_wormId << ": Was TrackingMerged, now sees 1 smaller blob. Potential split.";
            m_currentState = TrackerState::PausedAwaitingSplitDecision;
            emit splitDetectedAndPaused(m_wormId, originalFrameNumber, blobs); // Send this single blob as a split candidate
            m_lastPrimaryBlob.isValid = false;
            return false; // Paused
        }
        m_currentState = TrackerState::TrackingSingle; // Confidently tracking one
    } else { // plausibleBlobsFound > 1
        if (m_currentState == TrackerState::TrackingMerged) {
            // Was tracking a merged blob, now sees multiple smaller blobs. Clear split.
            qDebug() << "WormTracker ID" << m_wormId << ": Was TrackingMerged, now sees" << plausibleBlobsFound << "blobs. Clear split.";
            m_currentState = TrackerState::PausedAwaitingSplitDecision;
            emit splitDetectedAndPaused(m_wormId, originalFrameNumber, blobs);
            m_lastPrimaryBlob.isValid = false;
            return false; // Paused
        } else { // Not TrackingMerged, but seeing multiple blobs (TrackingSingle or PotentialMergeOrSplit)
            // This could be a single worm fragmenting, or other worms entering ROI.
            // Try to pick the "best" candidate to continue following.
            // Heuristic: if a previous primary blob existed, pick the new blob closest to it.
            // Otherwise, pick the largest or closest to ROI center.
            double minDistanceSq = std::numeric_limits<double>::max();
            QPointF referencePoint = m_lastPrimaryBlob.isValid ? m_lastPrimaryBlob.centroid : roiInOut.center();

            for (const auto& blob : blobs) {
                double dx = blob.centroid.x() - referencePoint.x();
                double dy = blob.centroid.y() - referencePoint.y();
                double distSq = dx * dx + dy * dy;
                if (distSq < minDistanceSq) {
                    minDistanceSq = distSq;
                    currentPrimaryTarget = blob;
                }
            }
            // If the chosen primary target is significantly different from the last (e.g. much smaller, different position)
            // AND there were other strong candidates, this could be a split of a single worm.
            if (m_lastPrimaryBlob.isValid && currentPrimaryTarget.isValid &&
                (currentPrimaryTarget.area < m_lastPrimaryBlob.area * 0.6 || QLineF(currentPrimaryTarget.centroid, m_lastPrimaryBlob.centroid).length() > qMax(m_estimatedWormSize.width(), m_estimatedWormSize.height())) &&
                plausibleBlobsFound > 1) {
                qDebug() << "WormTracker ID" << m_wormId << ": Single target may have split/fragmented. Pausing.";
                m_currentState = TrackerState::PausedAwaitingSplitDecision;
                emit splitDetectedAndPaused(m_wormId, originalFrameNumber, blobs);
                m_lastPrimaryBlob.isValid = false;
                return false; // Paused
            }
            m_currentState = TrackerState::PotentialMergeOrSplit; // Still ambiguous
        }
    }

    // If a primary target was identified and we are not paused
    if (currentPrimaryTarget.isValid) {
        m_lastKnownPosition = cv::Point2f(static_cast<float>(currentPrimaryTarget.centroid.x()), static_cast<float>(currentPrimaryTarget.centroid.y()));
        foundPositionOut = m_lastKnownPosition; // Output the found position
        m_estimatedWormSize = currentPrimaryTarget.boundingBox.size().isValid() ? currentPrimaryTarget.boundingBox.size() : m_estimatedWormSize;
        roiInOut = adjustRoi(m_lastKnownPosition, frame.size(), m_estimatedWormSize); // Adjust ROI for the next frame
        m_lastPrimaryBlob = currentPrimaryTarget; // Update the last primary blob

        emit positionUpdated(m_wormId, originalFrameNumber, currentPrimaryTarget.centroid, roiInOut, plausibleBlobsFound, currentPrimaryTarget.area);
        return true;
    }

    // Fallback: if no valid primary target chosen (should be rare if blobs were found)
    m_lastPrimaryBlob.isValid = false;
    emit positionUpdated(m_wormId, originalFrameNumber, QPointF(m_lastKnownPosition.x, m_lastKnownPosition.y), roiInOut, plausibleBlobsFound, 0.0);
    return false;
}


QRectF WormTracker::adjustRoi(const cv::Point2f& wormCenter, const cv::Size& frameSize, const QSizeF& wormSizeGuess) {
    QSizeF currentWormSize = wormSizeGuess.isValid() ? wormSizeGuess : QSizeF(20,5); // Use default if guess invalid

    qreal roiWidth = qMax(MIN_ROI_SIZE_WORMTRACKER.width(), currentWormSize.width() * DEFAULT_ROI_SIZE_MULTIPLIER_WORMTRACKER.width());
    qreal roiHeight = qMax(MIN_ROI_SIZE_WORMTRACKER.height(), currentWormSize.height() * DEFAULT_ROI_SIZE_MULTIPLIER_WORMTRACKER.height());
    qreal roiX = wormCenter.x - roiWidth / 2.0;
    qreal roiY = wormCenter.y - roiHeight / 2.0;

    // Clamp ROI to frame boundaries
    roiX = qMax(0.0, qMin(roiX, static_cast<qreal>(frameSize.width) - roiWidth));
    roiY = qMax(0.0, qMin(roiY, static_cast<qreal>(frameSize.height) - roiHeight));
    // Ensure width/height are also clamped if they were initially larger than the frame
    roiWidth = qMin(roiWidth, static_cast<qreal>(frameSize.width) - roiX);
    roiHeight = qMin(roiHeight, static_cast<qreal>(frameSize.height) - roiY);

    return QRectF(roiX, roiY, roiWidth, roiHeight);
}
