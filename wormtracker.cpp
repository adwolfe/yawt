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

    continueTracking();
}

void WormTracker::continueTracking() {
    // STEP 1: check thread status
    if (QThread::currentThread()->isInterruptionRequested()) {
        qDebug() << "WormTracker ID" << m_wormId << ": Tracking interrupted by thread request.";
        m_trackingActive = false; // We're done
    }
    if (m_trackingActive && m_currFrameNum < m_framesToProcess->size()){
        // if we're still tracking, next check tracker state
        const cv::Mat& currentFrame = (*m_framesToProcess)[m_currFrameNum];  // get frame to process
        if (currentFrame.empty()) {
            // whoops, better not process an empty frame. skip the rest
            qWarning() << "WormTracker ID" << m_wormId << ": Encountered empty frame at sequence index" << m_currFrameNum;
        } else {
            // process the frame, based on the state
            if (m_currentState != TrackerState::PausedAwaitingSplitDecision)
            {
                // whether we're tracking a single worm or two that are merged doesn't matter; continue tracking
                cv::Point2f primaryTargetPosition;
                // processSingleFrame handles internal state changes and signal emissions
                //qDebug() << "WormTracker ID" << m_wormId << ": seeking blobs";
                bool foundTargetThisFrame = processSingleFrame(currentFrame, m_currFrameNum, m_currentRoi, primaryTargetPosition);
                if (!foundTargetThisFrame) {
                    // If no target was found and we are not paused waiting for a split decision
                    // (e.g., truly lost, not just ambiguous)
                    // The positionUpdated signal would have been emitted with 0 plausible blobs.
                    // This can happen with inconsistent video lighting.
                    // Need to make sure trackingmanager handles this!
                    // emit stateChanged(m_wormId, WormObject::TrackingState::Lost);
                    qDebug() << "WormTracker ID" << m_wormId << ": Target lost at sequence index" << m_currFrameNum;
                    // Currently continues, hoping the worm pops back in.
                }
                if (m_currFrameNum % 10 == 0 || m_currFrameNum == static_cast<int>(m_framesToProcess->size()) - 1) {
                    emit progress(m_wormId, static_cast<int>((static_cast<double>(m_currFrameNum + 1) / m_framesToProcess->size()) * 100.0));
                }
                m_currFrameNum ++;
                QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
            } else // if m_currentState == TrackerState::PausedAwaitingSplitDecision
            {
                // We're still waiting for TrackManager to tell us what to do...
            }

        }
    } else { // if tracking is done or canceled
        if (m_trackingActive) { // If loop completed naturally
            qDebug() << "WormTracker ID" << m_wormId << ": Loop completed naturally";
            emit progress(m_wormId, 100);
            m_trackingActive = false;
        }
        qDebug() << "WormTracker ID" << m_wormId << ": Finished processing loop. Final state:" << static_cast<int>(m_currentState);
        emit finished(); // Signal that this tracker instance's work is done
    }
}

void WormTracker::stopTracking() {
    qDebug() << "WormTracker ID" << m_wormId << ": stopTracking() called. Current state:" << static_cast<int>(m_currentState);
    m_trackingActive = false;
    QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
    // If paused, setting m_trackingActive to false will close out the "continueTracking" cycle
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
    QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
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
    QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
}

bool WormTracker::processSingleFrame(const cv::Mat& frame, int sequenceFrameIndex, QRectF& roiInOut, cv::Point2f& foundPositionOut) {
    // This is where the magic happens. This function detects blobs within a reasonable ROI of the last known worm position.
    // It then counts how many blobs were found and delivers those to TrackingManager. If there are >1 blobs, TrackingManager needs to
    // sort out which is which.

    int originalFrameNumber;
    if (m_direction == TrackingDirection::Forward) {
        originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else { // Backward
        originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex;
    }

    QList<TrackingHelper::DetectedBlob> blobs = findPlausibleBlobsInRoi(frame, roiInOut);
    int plausibleBlobsFound = blobs.count();
    TrackingHelper::DetectedBlob currentPrimaryTarget;
    currentPrimaryTarget.isValid = false;

    qDebug() << "WormTracker ID" << m_wormId << (m_direction == TrackingDirection::Forward ? "Fwd" : "Bwd")
             << "Frame" << originalFrameNumber << "(Seq:" << sequenceFrameIndex << "): Initial search in ROI" << roiInOut
             << "found" << plausibleBlobsFound << "blobs.";

    if (plausibleBlobsFound == 0) {
        m_lastPrimaryBlob.isValid = false; // Target lost in current ROI
        emit positionUpdated(m_wormId, originalFrameNumber, QPointF(m_lastKnownPosition.x, m_lastKnownPosition.y), roiInOut, 0, 0.0);
        return false; // No target found
    }
    else if (plausibleBlobsFound == 1) {
        currentPrimaryTarget = blobs.first();

        if (currentPrimaryTarget.touchesROIboundary) {
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                     << ": Single blob found touches ROI boundary. Initial ROI:" << roiInOut
                     << "Blob BBox:" << currentPrimaryTarget.boundingBox;

            QRectF expandedRoi = roiInOut;
            const QRectF originalSearchRoiForThisFrame = roiInOut;
            const int MAX_EXPANSION_ITERATIONS = 3;
            const double EXPANSION_FACTOR = 1.25;
            const QSizeF MAX_ROI_SIZE_ABSOLUTE(frame.cols * 0.8, frame.rows * 0.8);

            for (int iter = 0; iter < MAX_EXPANSION_ITERATIONS; ++iter) {
                qreal newWidth = qMin(expandedRoi.width() * EXPANSION_FACTOR, MAX_ROI_SIZE_ABSOLUTE.width());
                qreal newHeight = qMin(expandedRoi.height() * EXPANSION_FACTOR, MAX_ROI_SIZE_ABSOLUTE.height());
                QPointF center = currentPrimaryTarget.isValid ? currentPrimaryTarget.centroid : expandedRoi.center();
                expandedRoi.setX(center.x() - newWidth / 2.0);
                expandedRoi.setY(center.y() - newHeight / 2.0);
                expandedRoi.setWidth(newWidth);
                expandedRoi.setHeight(newHeight);

                expandedRoi.setX(qMax(0.0, expandedRoi.x()));
                expandedRoi.setY(qMax(0.0, expandedRoi.y()));
                if (expandedRoi.right() > frame.cols) expandedRoi.setRight(frame.cols);
                if (expandedRoi.bottom() > frame.rows) expandedRoi.setBottom(frame.rows);

                if (expandedRoi.width() < MIN_ROI_SIZE_WORMTRACKER.width() || expandedRoi.height() < MIN_ROI_SIZE_WORMTRACKER.height()) {
                    qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Expanded ROI became too small after clamping. Stopping expansion.";
                    break;
                }

                qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                         << ": Expansion iter" << iter + 1 << ". New expanded ROI:" << expandedRoi;

                QList<TrackingHelper::DetectedBlob> newBlobs = findPlausibleBlobsInRoi(frame, expandedRoi);
                int newPlausibleBlobsFound = newBlobs.count();
                qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                         << ": Found" << newPlausibleBlobsFound << "blobs in expanded ROI.";

                if (newPlausibleBlobsFound == 0) {
                    qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                             << ": Expansion led to 0 blobs. Reverting to pre-expansion blob.";
                    roiInOut = originalSearchRoiForThisFrame;
                    break;
                }
                else if (newPlausibleBlobsFound == 1) {
                    currentPrimaryTarget = newBlobs.first();
                    roiInOut = expandedRoi;
                    if (!currentPrimaryTarget.touchesROIboundary) {
                        qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                                 << ": Blob now contained after expansion. ROI:" << roiInOut
                                 << "Blob BBox:" << currentPrimaryTarget.boundingBox;
                        break;
                    }
                    qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                             << ": Blob still touches boundary of expanded ROI:" << roiInOut
                             << "Blob BBox:" << currentPrimaryTarget.boundingBox;
                }
                else { // newPlausibleBlobsFound > 1
                    qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                             << ": Expansion revealed" << newPlausibleBlobsFound << "blobs. Pausing for split decision.";
                    m_currentState = TrackerState::PausedAwaitingSplitDecision;
                    emit splitDetectedAndPaused(m_wormId, originalFrameNumber, newBlobs);
                    roiInOut = expandedRoi;
                    return false;
                }

                if (iter == MAX_EXPANSION_ITERATIONS - 1 && currentPrimaryTarget.touchesROIboundary) {
                    qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                             << ": Max expansion iterations reached, blob still touches boundary. Using current blob.";
                }
                if (expandedRoi.size().width() >= MAX_ROI_SIZE_ABSOLUTE.width() || expandedRoi.size().height() >= MAX_ROI_SIZE_ABSOLUTE.height()){
                    qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                             << ": ROI reached max absolute size, blob still touches boundary. Using current blob.";
                    break;
                }
            }
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                     << ": After expansion loop, primary target centroid:" << currentPrimaryTarget.centroid
                     << "Area:" << currentPrimaryTarget.area << "TouchesBoundary:" << currentPrimaryTarget.touchesROIboundary
                     << "Final ROI for this frame:" << roiInOut;
        }

        // --- Logic after expansion (or if it never touched boundary) ---
        if (!currentPrimaryTarget.isValid) {
            qWarning() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": currentPrimaryTarget is invalid after plausibleBlobsFound == 1 logic.";
            m_lastPrimaryBlob.isValid = false;
            emit positionUpdated(m_wormId, originalFrameNumber, QPointF(m_lastKnownPosition.x, m_lastKnownPosition.y), roiInOut, plausibleBlobsFound, 0.0);
            return false;
        }

        // Check for potential merge based on area increase
        bool potentialMerge = false;
        const double MERGE_AREA_INCREASE_FACTOR = 1.6; // e.g., 60% larger than last blob
        const double ABSOLUTE_MERGE_AREA_THRESHOLD = m_maxBlobArea * 1.4; // e.g., 40% larger than typical max single worm area

        if (m_lastPrimaryBlob.isValid && currentPrimaryTarget.area > m_lastPrimaryBlob.area * MERGE_AREA_INCREASE_FACTOR && currentPrimaryTarget.area > m_minBlobArea) {
            potentialMerge = true;
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Potential merge by relative area increase. Current:" << currentPrimaryTarget.area << "Last:" << m_lastPrimaryBlob.area;
        } else if (!m_lastPrimaryBlob.isValid && currentPrimaryTarget.area > ABSOLUTE_MERGE_AREA_THRESHOLD) {
            // No last blob, but current is very large
            potentialMerge = true;
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Potential merge by absolute area. Current:" << currentPrimaryTarget.area;
        }


        if (potentialMerge && m_lastPrimaryBlob.isValid && !m_lastPrimaryBlob.contourPoints.empty() && !currentPrimaryTarget.contourPoints.empty()) {
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Attempting blob difference analysis for merge.";

            cv::Mat prevBlobMask = cv::Mat::zeros(frame.size(), CV_8UC1);
            cv::Mat currBlobMask = cv::Mat::zeros(frame.size(), CV_8UC1);

            std::vector<std::vector<cv::Point>> prevContoursVec = {m_lastPrimaryBlob.contourPoints};
            cv::drawContours(prevBlobMask, prevContoursVec, 0, cv::Scalar(255), cv::FILLED);

            std::vector<std::vector<cv::Point>> currContoursVec = {currentPrimaryTarget.contourPoints};
            cv::drawContours(currBlobMask, currContoursVec, 0, cv::Scalar(255), cv::FILLED);

            cv::Mat newGrowthMask, oldOverlapMask, prevBlobMaskNot;
            cv::bitwise_not(prevBlobMask, prevBlobMaskNot);
            cv::bitwise_and(currBlobMask, prevBlobMaskNot, newGrowthMask);               // Current AND (NOT Previous)
            cv::bitwise_and(currBlobMask, prevBlobMask, oldOverlapMask);                 // Current AND Previous

            TrackingHelper::DetectedBlob newGrowthComponent = findLargestBlobComponentInMask(newGrowthMask, "NewGrowth");
            TrackingHelper::DetectedBlob oldOverlapComponent = findLargestBlobComponentInMask(oldOverlapMask, "OldOverlap");

            QPointF chosenCentroid = currentPrimaryTarget.centroid;
            TrackingHelper::DetectedBlob componentForNextRoi = currentPrimaryTarget;

            if (oldOverlapComponent.isValid && oldOverlapComponent.area > m_minBlobArea * 0.3) {
                chosenCentroid = oldOverlapComponent.centroid;
                componentForNextRoi = oldOverlapComponent;
                qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Merge resolved: Following 'old overlap' component. Centroid:" << chosenCentroid;
            } else if (newGrowthComponent.isValid && oldOverlapComponent.area <= m_minBlobArea * 0.3 && newGrowthComponent.area > m_minBlobArea * 0.3) {
                chosenCentroid = newGrowthComponent.centroid;
                componentForNextRoi = newGrowthComponent;
                qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Merge resolved: Following 'new growth' component (old part small/gone). Centroid:" << chosenCentroid;
            } else {
                qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Merge difference analysis inconclusive or components too small. Following centroid of whole merged blob.";
            }

            m_lastKnownPosition = cv::Point2f(static_cast<float>(chosenCentroid.x()), static_cast<float>(chosenCentroid.y()));
            foundPositionOut = m_lastKnownPosition;
            m_estimatedWormSize = componentForNextRoi.boundingBox.size().isValid() ? componentForNextRoi.boundingBox.size() : m_estimatedWormSize;
            QRectF nextFrameRoi = adjustRoi(m_lastKnownPosition, frame.size(), m_estimatedWormSize);
            m_lastPrimaryBlob = currentPrimaryTarget;
            m_currentState = TrackerState::TrackingSingle;

            emit positionUpdated(m_wormId, originalFrameNumber, chosenCentroid, roiInOut, 1, currentPrimaryTarget.area);
            roiInOut = nextFrameRoi;
            return true;

        } else {
            // Not a suspected merge, or cannot perform difference analysis: Standard update
            m_lastKnownPosition = cv::Point2f(static_cast<float>(currentPrimaryTarget.centroid.x()), static_cast<float>(currentPrimaryTarget.centroid.y()));
            foundPositionOut = m_lastKnownPosition;
            m_estimatedWormSize = currentPrimaryTarget.boundingBox.size().isValid() ? currentPrimaryTarget.boundingBox.size() : m_estimatedWormSize;
            QRectF nextFrameRoi = adjustRoi(m_lastKnownPosition, frame.size(), m_estimatedWormSize);
            m_lastPrimaryBlob = currentPrimaryTarget;
            m_currentState = TrackerState::TrackingSingle;
            emit positionUpdated(m_wormId, originalFrameNumber, currentPrimaryTarget.centroid, roiInOut, plausibleBlobsFound, currentPrimaryTarget.area);
            roiInOut = nextFrameRoi;
            return true;
        }
    }
    else { // plausibleBlobsFound > 1
        qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                 << ": Multiple (" << plausibleBlobsFound << ") blobs found initially. Pausing for split decision.";
        m_currentState = TrackerState::PausedAwaitingSplitDecision;
        emit splitDetectedAndPaused(m_wormId, originalFrameNumber, blobs);
        return false; // Paused
    }
    return false;
}

/*bool WormTracker::processSingleFrame(const cv::Mat& frame, int sequenceFrameIndex, QRectF& roiInOut, cv::Point2f& foundPositionOut) {
    // This is where the magic happens. This function detects blobs within a reasonable ROI of the last known worm position.
    // It then counts how many blobs were found and delivers those to TrackingManager. If there are >1 blobs, TrackingManager needs to
    // sort out which is which.
    // First, find all the blobs in the frame and ROI of interest.
    QList<TrackingHelper::DetectedBlob> blobs = findPlausibleBlobsInRoi(frame, roiInOut);
    int plausibleBlobsFound = blobs.count();
    TrackingHelper::DetectedBlob currentPrimaryTarget; // This is the blob we decide to follow this frame
    currentPrimaryTarget.isValid = false; // I mean, we don't know yet so

    int originalFrameNumber;
    if (m_direction == TrackingDirection::Forward) {
        originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else {
        originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex;
    }
    qDebug() << "WormTracker ID" << m_wormId << m_direction << "found" << plausibleBlobsFound << "blobs in frame" << originalFrameNumber;

    if (plausibleBlobsFound == 0) {
        // Shit, no plausible blobs found in the current ROI
        // This might happen if the worm pops out of existence or the lighting is bad
        m_lastPrimaryBlob.isValid = false;
        emit positionUpdated(m_wormId, originalFrameNumber, QPointF(m_lastKnownPosition.x, m_lastKnownPosition.y), roiInOut, 0, 0.0);
        return false;
    }
    else if (plausibleBlobsFound == 1) {
        currentPrimaryTarget = blobs.first();
        if (currentPrimaryTarget.touchesROIboundary) {
            while (currentPrimaryTarget.touchesROIboundary)
            {
                // We are working under the assumption that the speed of the worm is not going to suddenly change dramatically
                // The FPS of the video should have smooth worm travel, it isn't going to jump out of the ROI
                // If the blob doesn't fit in the ROI, then the most likely scenario is two worms have merged
                // or the worm has passed over some other unmoving blob; also possible, but effect is the same.
                // We will expand the ROI until it covers the whole blob.

                // First, check that the ROI is not already larger than the initial ROI (which is fixed).
                // If it is larger, then check whether we are not already in AmbiguousMode -- ensure we are if not.
                // If it is not, then emit AmbiguousMode and redraw ROI by 25% larger.
                // Redo the check. Repeat this process until the largest blob is contained within the ROI.
            }
            // Then, we will calculate the centroid of the blob
            // Then, we will split the blob in twain at the centroid (to create two equal sized shapes -- is this easy to do?)
            // Then, we will determine the centroids of the two sub-blobs
            // Then, we will determine whether either centroid, or the original centroid, is closest to the previously known position
            // Then, we will use whichever is closest of these three as the updated position.

        }


    }
    // ORIGINAL LOGIC BELOW HERE


    else if (plausibleBlobsFound == 1) {
        // If there is just 1 blob in the ROI, it's either one worm or two worms that are now one.

        if (m_currentState == TrackerState::TrackingMerged && currentPrimaryTarget.area < m_maxBlobArea * 0.75) {
            // Was tracking a merged blob, now see a single, smaller blob. Potential split.
            // May want to remove the area thing
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
           // m_currentState = TrackerState::PotentialMergeOrSplit; // Still ambiguous
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
*/

TrackingHelper::DetectedBlob WormTracker::findLargestBlobComponentInMask(const cv::Mat& mask, const QString& debugContextName) {
    TrackingHelper::DetectedBlob resultBlob;
    resultBlob.isValid = false;
    if (mask.empty() || cv::countNonZero(mask) == 0) {
        return resultBlob;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return resultBlob;
    }

    double maxArea = 0;
    int largestContourIdx = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            largestContourIdx = static_cast<int>(i);
        }
    }

    if (largestContourIdx != -1 && maxArea > (m_minBlobArea * 0.2)) {
        const auto& largestContour = contours[largestContourIdx];
        cv::Moments mu = cv::moments(largestContour);
        if (mu.m00 > 0) {
            resultBlob.centroid = QPointF(mu.m10 / mu.m00, mu.m01 / mu.m00);

            // Corrected line:
            cv::Rect cvBox = cv::boundingRect(largestContour);
            resultBlob.boundingBox = QRectF(static_cast<qreal>(cvBox.x),
                                            static_cast<qreal>(cvBox.y),
                                            static_cast<qreal>(cvBox.width),
                                            static_cast<qreal>(cvBox.height));

            resultBlob.area = maxArea;
            resultBlob.contourPoints = largestContour;
            resultBlob.isValid = true;
            resultBlob.touchesROIboundary = false;

            qDebug() << "WormTracker ID" << m_wormId << ":" << debugContextName
                     << "component found. Centroid:" << resultBlob.centroid << "Area:" << resultBlob.area;
        }
    } else {
        qDebug() << "WormTracker ID" << m_wormId << ":" << debugContextName
                 << "component found no significant blob. Max area found:" << maxArea;
    }
    return resultBlob;
}


QList<TrackingHelper::DetectedBlob> WormTracker::findPlausibleBlobsInRoi(const cv::Mat& fullFrame, const QRectF& roi) {
    // This function now directly uses the TrackingHelper function
    // It passes the tracker's specific plausibility parameters.
    return TrackingHelper::findAllPlausibleBlobsInRoi(fullFrame, roi,
                                                      m_minBlobArea, m_maxBlobArea,
                                                      m_minAspectRatio, m_maxAspectRatio);
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
