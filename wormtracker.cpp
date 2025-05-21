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
    m_direction(direction),
    m_framesToProcess(nullptr),
    m_initialRoiEdge(initialRoi.width()),   // It's a square
    m_minBlobArea(TrackingConstants::DEFAULT_MIN_WORM_AREA),
    m_maxBlobArea(TrackingConstants::DEFAULT_MAX_WORM_AREA),
    m_currentRoi(initialRoi),
    m_lastKnownPosition(initialRoi.center().x(), initialRoi.center().y()),
    m_videoKeyFrameNum(videoKeyFrameNum),
    m_trackingActive(false),
    m_currentState(TrackerState::Idle),
    //DEFUNCT
    m_minAspectRatio(TrackingConstants::DEFAULT_MIN_ASPECT_RATIO),
    m_maxAspectRatio(TrackingConstants::DEFAULT_MAX_ASPECT_RATIO),
    m_estimatedWormSize(initialRoi.size().isValid() ? initialRoi.size() : QSizeF(20,5)) // Initial guess from selection ROI
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

void WormTracker::continueTracking()
{
    if (QThread::currentThread()->isInterruptionRequested())
    {   // STEP 1: check thread status:
        qDebug() << "WormTracker ID" << m_wormId << ": Tracking interrupted by thread request.";
        m_trackingActive = false; // We're done
    }
    if (m_trackingActive && m_currFrameNum < static_cast<int>(m_framesToProcess->size()))
    {   // STEP 2: If we're actively tracking, and we haven't reached the video end:
        const cv::Mat& currentFrame = (*m_framesToProcess)[m_currFrameNum];  // get frame to process

        if (currentFrame.empty())
        { // whoops, better not process an empty frame. skip the rest
            qWarning() << "WormTracker ID" << m_wormId << ": Encountered empty frame at sequence index" << m_currFrameNum;
        }

        else
        {   // process the frame, based on the state
            bool foundTargetThisFrame = false;  // Unless a split is inferred, this should come back true

            if (m_currentState == TrackerState::TrackingSingle || m_currentState == TrackerState::AmbiguouslySingle)
            {   // The simplest scenario. Worms should all be starting as single worms; at the very least, every track starts with this function
                // We might know the tracker has detected a second ROI but it hasn't affected us yet
                foundTargetThisFrame = processFrameAsSingleWorm(currentFrame, m_currFrameNum, m_currentRoi);
            }

            else if (m_currentState == TrackerState::TrackingMerged || m_currentState == TrackerState::AmbiguouslyMerged)
            {   // Wormtracker knows it is tracking two blobs that merged (confirmed by TrackingManager)
                foundTargetThisFrame = processFrameAsMergedWorms(currentFrame, m_currFrameNum, m_currentRoi);
            }
            // At this point if we haven't found a blob, we're either waiting or it's missing
            if (!foundTargetThisFrame) qDebug() << "WormTracker ID" << m_wormId << ": Target search at sequence index" << m_currFrameNum << "was skipped or unsuccessful; continuing";

            if (m_currentState != TrackerState::PausedForSplit)
            {   // Emit progress and move to next frame (if we didn't find anything, maybe it'll return next frame)
                if (m_currFrameNum % 10 == 0 || m_currFrameNum == static_cast<int>(m_framesToProcess->size()) - 1) {
                    emit progress(m_wormId, static_cast<int>((static_cast<double>(m_currFrameNum + 1) / m_framesToProcess->size()) * 100.0));
                }
                m_currFrameNum ++;
            }
            else
            {
                // We're still waiting for TrackerManager to tell us which blob to follow, so call it again.
            }
        }
        QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);

    } else { // e.g. if tracking is complete, or it's active but has no frames left, or its canceled
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


bool WormTracker::processFrameAsSingleWorm(const cv::Mat& frame, int sequenceFrameIndex, QRectF& roiIn)
{
    // This is where the magic happens. This function detects blobs within a fixed ROI of the last known worm position. If the largest blob fits within the ROI,
    // it assumes we're still seeing our worm and continues tracking that. If that blob no longer fits within the ROI, then it assumes we're in a merging state
    // and alerts TrackingManager; it then attempts to discern our worm within the merged entity.
    // Notably, in this case, a second blob is really no concern. It's only if we're already tracking a merged entity that two blobs might represent a split.

    int originalFrameNumber;
    if (m_direction == TrackingDirection::Forward) {
        originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else { // Backwards movement
        originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex;
    }
    // First, gather all the blobs in our fixed ROI
    QList<TrackingHelper::DetectedBlob> blobs = findPlausibleBlobsInRoi(frame, roiIn);
    int plausibleBlobsFound = blobs.count();
    TrackingHelper::DetectedBlob currentPrimaryTarget;
    currentPrimaryTarget.isValid = false;

    qDebug() << "WormTracker ID" << m_wormId << (m_direction == TrackingDirection::Forward ? "Fwd" : "Bwd")
             << "Frame" << originalFrameNumber << "(Seq:" << sequenceFrameIndex << "): Initial search in ROI" << roiIn
             << "found" << plausibleBlobsFound << "blobs.";

    if (plausibleBlobsFound == 0)
    {   // Worst case scenario! Where did it go?
        m_lastPrimaryBlob.isValid = false;  // Marking that last blob point for failure
        emit positionUpdated(m_wormId, originalFrameNumber, QPointF(m_lastKnownPosition.x, m_lastKnownPosition.y), roiIn, 0, 0.0);
        return false; // No target found. We'll continue tracking the next frame with a slightly larger ROI (happens in continueTracking)
    }
    else  //if (plausibleBlobsFound >= 1)
    {   // Whether we have 1 or more blobs, so long as our biggest blob is valid and contained within the ROI, we should be fine.
        currentPrimaryTarget = blobs.first();
        if (currentPrimaryTarget.touchesROIboundary)
        {   // This shouldn't happen if you have set your ROI size properly! It depends upon speed and FPS too.
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                     << ": Single blob found touches ROI boundary. Initial ROI:" << roiIn
                     << "Blob BBox:" << currentPrimaryTarget.boundingBox;
            // Handle this edge case
            return false;
        }
        else
        {   // Our main blob fits within the ROI, but let's make sure it's worm-sized.
            if (!currentPrimaryTarget.isValid)
            {   // This might happen if for some reason the blob that is detected is too small.
                qWarning() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": single currentPrimaryTarget is invalid after plausibleBlobsFound >= 1 logic.";
                m_lastPrimaryBlob.isValid = false;
                emit positionUpdated(m_wormId, originalFrameNumber, QPointF(m_lastKnownPosition.x, m_lastKnownPosition.y), roiIn, plausibleBlobsFound, 0.0);
                return false;
            }
            else
            {   // We're good to process this frame as normal.
                // Get the centroid for our blob,
                m_lastKnownPosition = cv::Point2f(static_cast<float>(currentPrimaryTarget.centroid.x()), static_cast<float>(currentPrimaryTarget.centroid.y()));
                //m_estimatedWormSize = currentPrimaryTarget.boundingBox.size().isValid() ? currentPrimaryTarget.boundingBox.size() : m_estimatedWormSize;
                QRectF nextFrameRoi = adjustRoiPos(m_lastKnownPosition, frame.size());
                m_lastPrimaryBlob = currentPrimaryTarget;
                m_currentState = TrackerState::TrackingSingle;
                emit positionUpdated(m_wormId, originalFrameNumber, currentPrimaryTarget.centroid, roiIn, plausibleBlobsFound, currentPrimaryTarget.area);
                m_currentRoi = nextFrameRoi;
                // don't return yet, let the conditional finish
            }
        }
        if (plausibleBlobsFound > 1 && m_currentState != TrackerState::AmbiguouslySingle)
        {   // Even though we've processed it, we should tell the TrackingManager we've seen something else in our ROI.
            m_currentState = TrackerState::AmbiguouslySingle;
            emit stateChanged(m_wormId, m_currentState);
        }
        else {
            // Tell the TM if things are normal again.
            if (m_currentState != TrackerState::TrackingSingle) emit stateChanged(m_wormId, TrackerState::TrackingSingle);
            m_currentState = TrackerState::TrackingSingle;
        }
        // if we made it here we found something and progressed the ROI so lets tell continueTracking as such
        return true;
    }
}

bool WormTracker::processFrameAsMergedWorms(const cv::Mat& frame, int sequenceFrameIndex, QRectF& roiIn)
{
    int originalFrameNumber;
    if (m_direction == TrackingDirection::Forward) {
        originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else { // Backwards movement
        originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex;
    }
    // First, gather all the blobs in our fixed ROI
    QList<TrackingHelper::DetectedBlob> blobs = findPlausibleBlobsInRoi(frame, roiIn);
    int plausibleBlobsFound = blobs.count();
    TrackingHelper::DetectedBlob currentPrimaryTarget;
    currentPrimaryTarget.isValid = false;

    qDebug() << "WormTracker ID" << m_wormId << (m_direction == TrackingDirection::Forward ? "Fwd" : "Bwd")
             << "Frame" << originalFrameNumber << "(Seq:" << sequenceFrameIndex << "): Initial search in ROI" << roiIn
             << "found" << plausibleBlobsFound << "blobs.";
}

/*
            // If a blob is larger than the initial ROI, expand the ROI up to three times to see if we can get the combined blob in entirety.
            QRectF expandedRoi = roiInOut;
            const QRectF originalSearchRoiForThisFrame = roiInOut;
            const int MAX_EXPANSION_ITERATIONS = 3;
            const double EXPANSION_FACTOR = 1.25;
            const QSizeF MAX_ROI_SIZE_ABSOLUTE(frame.cols * 0.8, frame.rows * 0.8);

            for (int iter = 0; iter < MAX_EXPANSION_ITERATIONS; ++iter) {
                // Expand but keep it within the frame boundaries.
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
                    // This should never happen but good to check for I guess
                    qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Expanded ROI was too small after clamping. Stopping expansion.";
                    break;
                }

                qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                         << ": Expansion #" << iter + 1 << ". New expanded ROI:" << expandedRoi;

                QList<TrackingHelper::DetectedBlob> newBlobs = findPlausibleBlobsInRoi(frame, expandedRoi);
                int newPlausibleBlobsFound = newBlobs.count();
                qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                         << ": Found" << newPlausibleBlobsFound << "blobs in expanded ROI.";

                if (newPlausibleBlobsFound == 0) {
                    // This shouldn't happen but just in case
                    qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                             << ": Expansion somehow led to 0 blobs. Reverting to pre-expansion blob.";
                    roiInOut = originalSearchRoiForThisFrame;
                    break;
                }
                else  { //(newPlausibleBlobsFound >= 1)
                    // At this point we're operating under the assumption that other blobs are not our worms of interest
                    // The biggest blob should still be our worms
                    currentPrimaryTarget = newBlobs.first();

                    // Important! We want to maintain focus on our portion of the worm, so don't pass the expanded roi to the next frame
                    //roiInOut = expandedRoi;
                    if (!currentPrimaryTarget.touchesROIboundary) {
                        qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                                 << ": Blob now contained after expansion. ROI:" << expandedRoi
                                 << "Blob BBox:" << currentPrimaryTarget.boundingBox;
                        break;
                    }
                    qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                             << ": Blob still touches boundary of expanded ROI:" << roiInOut
                             << "Blob BBox:" << currentPrimaryTarget.boundingBox;
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

        }
    }
    else { // plausibleBlobsFound > 1

        // This is the trickiest state. If we have two or more worms merged already, and now we are getting an additional blob,
        // it could be either due to a split, or because a NEW worm is joining the party
        // Will have to really think about the most efficient way to do this

        if (m_currentState == TrackerState::TrackingMerged) {
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                     << ": Multiple (" << plausibleBlobsFound << ") blobs found initially. Pausing for split decision.";
            m_currentState = TrackerState::PausedForSplit;
            emit splitDetectedAndPaused(m_wormId, originalFrameNumber, blobs);
            return false; // Paused
        } else{

        }
    }
    return false;
}

*/

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

TrackingHelper::DetectedBlob WormTracker::findLargestBlobComponentInMask(const cv::Mat& mask, const QString& debugContextName) {
    // This might be redundant with TrackingCommon -- consider refactoring
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


QRectF WormTracker::adjustRoiSize(const QRectF roiIn, const cv::Size& frameSize) {
    // Takes in an ROI and returns the same ROI expanded by some static factor
    // Uses same center. Truncates the ROI if we're at the frame edge.

    qreal roiWidth = roiIn.width() * EXPANSION_FACTOR;
    qreal roiHeight = roiIn.height() * EXPANSION_FACTOR;

    qreal roiX = roiIn.center().x() - roiWidth / 2.0;
    qreal roiY = roiIn.center().y() - roiHeight / 2.0;

    // Clamp ROI to frame boundaries
    roiX = qMax(0.0, qMin(roiX, static_cast<qreal>(frameSize.width) - roiWidth));
    roiY = qMax(0.0, qMin(roiY, static_cast<qreal>(frameSize.height) - roiHeight));
    // Ensure width/height are also clamped if they were initially larger than the frame
    roiWidth = qMin(roiWidth, static_cast<qreal>(frameSize.width) - roiX);
    roiHeight = qMin(roiHeight, static_cast<qreal>(frameSize.height) - roiY);

    return QRectF(roiX, roiY, roiWidth, roiHeight);
}


QRectF WormTracker::adjustRoiPos(const cv::Point2f& wormCenter, const cv::Size& frameSize) {
    // Takes in an ROI and adjusts it to a new center. Keeps the same size, unless it's at the boundary.

    qreal roiWidth = m_initialRoiEdge;
    qreal roiHeight = m_initialRoiEdge;
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
