// wormtracker.cpp
#include "wormtracker.h" // Lowercase include
#include <QDebug>
#include <QtMath>       // For qSqrt, qPow, qAbs
#include <algorithm>    // For std::sort, std::min_element etc. if needed
#include <limits>       // For std::numeric_limits
#include <QLineF>


// ROI adjustment parameters (These might be defunct if ROI size is strictly fixed from BlobTableModel)
// const QSizeF DEFAULT_ROI_SIZE_MULTIPLIER_WORMTRACKER(2.5, 2.5);
// const QSizeF MIN_ROI_SIZE_WORMTRACKER(30, 30);

WormTracker::WormTracker(int wormId,
                         QRectF initialRoi, // This is the fixed-size ROI
                         TrackingDirection direction,
                         int videoKeyFrameNum,
                         QObject *parent)
    : QObject(parent),
    m_wormId(wormId),
    m_direction(direction),
    m_framesToProcess(nullptr),
    // Ensure m_initialRoiEdge takes the width, assuming square ROI from BlobTableModel
    m_initialRoiEdge(initialRoi.isValid() ? initialRoi.width() : 20.0), // Default if invalid
    m_minBlobArea(TrackingConstants::DEFAULT_MIN_WORM_AREA), // Using constants from trackingcommon.h
    m_maxBlobArea(TrackingConstants::DEFAULT_MAX_WORM_AREA),
    m_currentSearchRoi(initialRoi),
    m_lastKnownPosition(initialRoi.center().x(), initialRoi.center().y()),
    m_videoKeyFrameNum(videoKeyFrameNum),
    m_trackingActive(false),
    m_currentState(TrackerState::Idle)
// m_minAspectRatio, m_maxAspectRatio, m_estimatedWormSize are not initialized as their roles are changing.
{
    qDebug() << "WormTracker (" << this << ") created for worm ID:" << m_wormId
             << "Direction:" << (direction == TrackingDirection::Forward ? "Forward" : "Backward")
             << "Initial Fixed ROI:" << initialRoi << "Edge:" << m_initialRoiEdge;
    m_lastPrimaryBlob.isValid = false;
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
    m_currentState = TrackerState::TrackingSingle;
    m_lastPrimaryBlob.isValid = false;
    qDebug() << "WormTracker ID" << m_wormId << ": Starting tracking. Frames:" << m_framesToProcess->size()
             << "Initial Search ROI:" << m_currentSearchRoi;

    continueTracking();
}

void WormTracker::continueTracking()
{
    if (QThread::currentThread()->isInterruptionRequested())
    {
        qDebug() << "WormTracker ID" << m_wormId << ": Tracking interrupted by thread request.";
        m_trackingActive = false;
    }
    if (m_trackingActive && m_currFrameNum < static_cast<int>(m_framesToProcess->size()))
    {
        const cv::Mat& currentFrame = (*m_framesToProcess)[m_currFrameNum];

        if (currentFrame.empty())
        {
            qWarning() << "WormTracker ID" << m_wormId << ": Encountered empty frame at sequence index" << m_currFrameNum;
        }
        else
        {
            bool foundTargetThisFrame = false;
            // Ensure m_currentSearchRoi is valid before processing
            if (!m_currentSearchRoi.isValid() || m_currentSearchRoi.isEmpty()) {
                qWarning() << "WormTracker ID" << m_wormId << "Frame" << (m_direction == TrackingDirection::Forward ? m_videoKeyFrameNum + m_currFrameNum : m_videoKeyFrameNum - 1 - m_currFrameNum)
                << ": Invalid m_currentSearchRoi before processing. Resetting based on last known position.";
                m_currentSearchRoi = adjustRoiPos(m_lastKnownPosition, currentFrame.size());
            }


            if (m_currentState == TrackerState::TrackingSingle || m_currentState == TrackerState::AmbiguouslySingle)
            {
                foundTargetThisFrame = processFrameAsSingleWorm(currentFrame, m_currFrameNum, m_currentSearchRoi);
            }
            else if (m_currentState == TrackerState::TrackingMerged || m_currentState == TrackerState::AmbiguouslyMerged)
            {
                foundTargetThisFrame = processFrameAsMergedWorms(currentFrame, m_currFrameNum, m_currentSearchRoi);
            }

            if (!foundTargetThisFrame && m_currentState != TrackerState::PausedForSplit) {
                qDebug() << "WormTracker ID" << m_wormId << ": Target search at sequence index" << m_currFrameNum << "was skipped or unsuccessful; ROI remains" << m_currentSearchRoi;
            }


            if (m_currentState != TrackerState::PausedForSplit)
            {
                if (m_currFrameNum % 10 == 0 || m_currFrameNum == static_cast<int>(m_framesToProcess->size()) - 1) {
                    emit progress(m_wormId, static_cast<int>((static_cast<double>(m_currFrameNum + 1) / m_framesToProcess->size()) * 100.0));
                }
                m_currFrameNum++;
            }
        }
        QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);

    } else {
        if (m_trackingActive) {
            qDebug() << "WormTracker ID" << m_wormId << ": Loop completed naturally";
            emit progress(m_wormId, 100);
            m_trackingActive = false;
        }
        qDebug() << "WormTracker ID" << m_wormId << ": Finished processing loop. Final state:" << static_cast<int>(m_currentState);
        emit finished();
    }
}

void WormTracker::stopTracking() {
    qDebug() << "WormTracker ID" << m_wormId << ": stopTracking() called. Current state:" << static_cast<int>(m_currentState);
    m_trackingActive = false;
    // If called from a different thread, ensure continueTracking is invoked in this tracker's thread
    // to properly exit the loop and emit finished.
    QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
}

TrackingHelper::DetectedBlob WormTracker::findPersistingComponent(
    const TrackingHelper::DetectedBlob& previousFrameBlob,
    const TrackingHelper::DetectedBlob& currentFrameBlob,
    const cv::Size& frameSize,
    int originalFrameNumberForDebug)
{
    TrackingHelper::DetectedBlob persistingComponent; // Default invalid
    persistingComponent.isValid = false;

    if (!previousFrameBlob.isValid || previousFrameBlob.contourPoints.empty() ||
        !currentFrameBlob.isValid || currentFrameBlob.contourPoints.empty()) {
        qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                 << ": findPersistingComponent called with invalid input blobs.";
        return persistingComponent; // Return invalid
    }

    cv::Mat prevBlobMask = cv::Mat::zeros(frameSize, CV_8UC1);
    cv::Mat currBlobMask = cv::Mat::zeros(frameSize, CV_8UC1);

    std::vector<std::vector<cv::Point>> prevContoursVec = {previousFrameBlob.contourPoints};
    cv::drawContours(prevBlobMask, prevContoursVec, 0, cv::Scalar(255), cv::FILLED);

    std::vector<std::vector<cv::Point>> currContoursVec = {currentFrameBlob.contourPoints};
    cv::drawContours(currBlobMask, currContoursVec, 0, cv::Scalar(255), cv::FILLED);

    cv::Mat newGrowthMask, oldOverlapMask, prevBlobMaskNot;
    cv::bitwise_not(prevBlobMask, prevBlobMaskNot);
    cv::bitwise_and(currBlobMask, prevBlobMaskNot, newGrowthMask);    // Component in current not in previous
    cv::bitwise_and(currBlobMask, prevBlobMask, oldOverlapMask);      // Component in current AND in previous

    TrackingHelper::DetectedBlob newGrowthComponentDetails = findLargestBlobComponentInMask(newGrowthMask, "NewGrowthAnalysis");
    TrackingHelper::DetectedBlob oldOverlapComponentDetails = findLargestBlobComponentInMask(oldOverlapMask, "OldOverlapAnalysis");

    // Heuristic to decide: Prioritize the overlapping part if it's substantial.
    // The 0.3 factor is a threshold to consider a component "significant" relative to min worm area.
    // These thresholds might need tuning.
    const double significanceThresholdFactor = 0.3;

    if (oldOverlapComponentDetails.isValid && oldOverlapComponentDetails.area > (m_minBlobArea * significanceThresholdFactor)) {
        qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                 << ": Persisting component identified as 'old overlap'. Centroid:" << oldOverlapComponentDetails.centroid
                 << "Area:" << oldOverlapComponentDetails.area;
        persistingComponent = oldOverlapComponentDetails;
    } else if (newGrowthComponentDetails.isValid && newGrowthComponentDetails.area > (m_minBlobArea * significanceThresholdFactor)) {
        // This case means the old overlap was tiny or non-existent, but there's new growth.
        // This might happen if the worm moved quickly, and the "new growth" is actually the worm in a new spot
        // with little overlap from the exact previous contour. Or it's a new blob entirely.
        // For now, if old overlap is insignificant, we consider new growth.
        qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                 << ": Persisting component identified as 'new growth' (old overlap was small/invalid). Centroid:" << newGrowthComponentDetails.centroid
                 << "Area:" << newGrowthComponentDetails.area;
        persistingComponent = newGrowthComponentDetails;
    } else {
        // If neither component is significant, the analysis is inconclusive.
        // Fallback to the centroid of the whole currentFrameBlob, but mark it as potentially less reliable.
        // Or, for stricter tracking, one might consider this a "lost" or "uncertain" state.
        qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                 << ": Persisting component analysis inconclusive or components too small. Using full current blob.";
        persistingComponent = currentFrameBlob; // Use the whole current blob
        // Optionally, add a flag to 'persistingComponent' if it's a fallback:
        // persistingComponent.analysisUncertain = true; (requires adding to DetectedBlob struct)
    }
    return persistingComponent;
}


bool WormTracker::processFrameAsSingleWorm(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentFixedSearchRoiRef)
{
    int originalFrameNumber;
    if (m_direction == TrackingDirection::Forward) {
        originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else {
        originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex;
    }

    QList<TrackingHelper::DetectedBlob> blobsInRoi = findPlausibleBlobsInRoi(frame, currentFixedSearchRoiRef);
    int plausibleBlobsFound = blobsInRoi.count();

    qDebug() << "WormTracker ID" << m_wormId << (m_direction == TrackingDirection::Forward ? "Fwd" : "Bwd")
             << "Frame" << originalFrameNumber << "(Seq:" << sequenceFrameIndex << "): Searching in fixed ROI" << currentFixedSearchRoiRef
             << "found" << plausibleBlobsFound << "plausible blobs.";

    if (plausibleBlobsFound == 0) {
        m_lastPrimaryBlob.isValid = false;
        emit positionUpdated(m_wormId, originalFrameNumber, QPointF(m_lastKnownPosition.x, m_lastKnownPosition.y), currentFixedSearchRoiRef, 0, 0.0);
        // ROI does not change position if nothing is found, it will search same area next frame.
        // Or, could slightly expand search or enter a "lost" state strategy. For now, keeps ROI.
        return false;
    }

    TrackingHelper::DetectedBlob currentFrameBestBlob = blobsInRoi.first(); // Largest is first due to sort in findAllPlausibleBlobsInRoi

    // Check if the largest blob touches the boundary of our *fixed search ROI*.
    if (currentFrameBestBlob.touchesROIboundary ||
        !currentFixedSearchRoiRef.contains(currentFrameBestBlob.boundingBox.center()) || // Check if center is contained
        !currentFixedSearchRoiRef.contains(currentFrameBestBlob.boundingBox) // Check if whole bbox is contained
        ) {
        qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                 << ": Largest blob touches boundary or not fully contained in fixed search ROI:" << currentFixedSearchRoiRef
                 << "Blob BBox:" << currentFrameBestBlob.boundingBox;

        // This is a critical event. The worm is at the edge of its expected search area.
        // This could be a merge, or it's moving fast.
        // We use the findPersistingComponent logic to get a better centroid.
        if (m_lastPrimaryBlob.isValid) {
            TrackingHelper::DetectedBlob persistentPart = findPersistingComponent(m_lastPrimaryBlob, currentFrameBestBlob, frame.size(), originalFrameNumber);
            if (persistentPart.isValid) {
                currentFrameBestBlob = persistentPart; // Use the resolved component
                qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                         << ": Used persisting component. New best blob centroid:" << currentFrameBestBlob.centroid;
            } else {
                qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                         << ": Persisting component analysis failed for boundary case. Using original largest blob.";
            }
        }
        // Even after analysis, if it's still on boundary, TM needs to know.
        // The state might change to AmbiguouslySingle or trigger merge checks in TM.
        // For now, we track this (potentially resolved) blob.
    }

    // Heuristics for potential merge (simplified from your snippet, to be used before calling findPersistingComponent if needed)
    // This can be refined. For now, processFrameAsSingleWorm assumes it's mostly a single worm.
    // More complex merge detection would involve TrackingManager.
    const double MERGE_AREA_INCREASE_FACTOR = 1.6;
    bool potentialMergeSuspected = false;
    if (m_lastPrimaryBlob.isValid && currentFrameBestBlob.area > m_lastPrimaryBlob.area * MERGE_AREA_INCREASE_FACTOR && currentFrameBestBlob.area > m_minBlobArea) {
        potentialMergeSuspected = true;
    } else if (!m_lastPrimaryBlob.isValid && currentFrameBestBlob.area > m_maxBlobArea * 1.4) { // Max typical area
        potentialMergeSuspected = true;
    }

    if (potentialMergeSuspected && m_lastPrimaryBlob.isValid) {
        qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Potential merge suspected. Current area:" << currentFrameBestBlob.area << "Last area:" << m_lastPrimaryBlob.area;
        TrackingHelper::DetectedBlob resolvedBlob = findPersistingComponent(m_lastPrimaryBlob, currentFrameBestBlob, frame.size(), originalFrameNumber);
        if(resolvedBlob.isValid) {
            currentFrameBestBlob = resolvedBlob;
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Merge resolved locally to centroid:" << currentFrameBestBlob.centroid;
        }
    }


    m_lastKnownPosition = cv::Point2f(static_cast<float>(currentFrameBestBlob.centroid.x()), static_cast<float>(currentFrameBestBlob.centroid.y()));
    QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, frame.size()); // Calculate next ROI based on new position

    m_lastPrimaryBlob = currentFrameBestBlob; // Update last primary blob
    m_currentState = TrackerState::TrackingSingle; // Assume single unless other logic changes it

    // Emit position using the *current* fixed search ROI that was used for finding the blob
    emit positionUpdated(m_wormId, originalFrameNumber, currentFrameBestBlob.centroid, currentFixedSearchRoiRef, plausibleBlobsFound, currentFrameBestBlob.area);

    currentFixedSearchRoiRef = nextFrameSearchRoi; // Update the ROI for the next iteration

    if (plausibleBlobsFound > 1 && m_currentState != TrackerState::AmbiguouslySingle) {
        m_currentState = TrackerState::AmbiguouslySingle;
        emit stateChanged(m_wormId, m_currentState);
        qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Multiple blobs (" << plausibleBlobsFound << ") in ROI. State -> AmbiguouslySingle.";
    } else if (plausibleBlobsFound == 1 && m_currentState == TrackerState::AmbiguouslySingle) {
        // Back to normal
        m_currentState = TrackerState::TrackingSingle;
        emit stateChanged(m_wormId, m_currentState);
    }
    return true;
}

bool WormTracker::processFrameAsMergedWorms(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentFixedSearchRoiRef)
{
    // This function needs to be fully implemented.
    // It will likely use findPersistingComponent to try and track its specific worm
    // within a larger merged blob whose ROI is dictated by TrackingManager.
    // If the merged entity splits, it should detect this and emit splitDetectedAndPaused.

    int originalFrameNumber;
    if (m_direction == TrackingDirection::Forward) {
        originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else {
        originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex;
    }

    qDebug() << "WormTracker ID" << m_wormId << (m_direction == TrackingDirection::Forward ? "Fwd" : "Bwd")
             << "Frame" << originalFrameNumber << "(Seq:" << sequenceFrameIndex << "): In processFrameAsMergedWorms. ROI:" << currentFixedSearchRoiRef;

    QList<TrackingHelper::DetectedBlob> blobsInRoi = findPlausibleBlobsInRoi(frame, currentFixedSearchRoiRef);
    int plausibleBlobsFound = blobsInRoi.count();

    if (plausibleBlobsFound == 0) {
        m_lastPrimaryBlob.isValid = false;
        emit positionUpdated(m_wormId, originalFrameNumber, QPointF(m_lastKnownPosition.x, m_lastKnownPosition.y), currentFixedSearchRoiRef, 0, 0.0);
        return false;
    }

    // Assuming currentFixedSearchRoiRef is large enough to contain the merged entity.
    // We need to find *our* worm within it.
    TrackingHelper::DetectedBlob currentMergedMass = blobsInRoi.first(); // The largest blob is likely the merged mass.

    if (m_lastPrimaryBlob.isValid) {
        TrackingHelper::DetectedBlob ourComponent = findPersistingComponent(m_lastPrimaryBlob, currentMergedMass, frame.size(), originalFrameNumber);

        if (ourComponent.isValid) {
            m_lastKnownPosition = cv::Point2f(static_cast<float>(ourComponent.centroid.x()), static_cast<float>(ourComponent.centroid.y()));
            QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, frame.size()); // Re-center our fixed-size ROI

            m_lastPrimaryBlob = ourComponent; // Our part of the merge
            // State remains TrackingMerged until TM confirms a split or separation.

            emit positionUpdated(m_wormId, originalFrameNumber, ourComponent.centroid, currentFixedSearchRoiRef, plausibleBlobsFound, ourComponent.area);
            currentFixedSearchRoiRef = nextFrameSearchRoi; // Update our tracker's personal search ROI for next frame.

            // Now, check if the *original* mergedMass (blobsInRoi.first()) shows signs of splitting
            // This is a simplified split detection. More robust would be needed.
            // If currentMergedMass is significantly smaller than expected, or if multiple comparable blobs appear.
            if (plausibleBlobsFound > 1) { // If findPlausibleBlobsInRoi gave more than one significant blob
                bool potentialSplit = false;
                // Example: if the 'ourComponent' is much smaller than 'currentMergedMass'
                // and there are other significant blobs.
                if (ourComponent.area < currentMergedMass.area * 0.7 && blobsInRoi.count() > 1) {
                    // Check if the second blob is also significant
                    if (blobsInRoi.at(1).area > m_minBlobArea) {
                        potentialSplit = true;
                    }
                }

                if (potentialSplit) {
                    qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber
                             << ": Potential SPLIT detected from merged state. Pausing.";
                    m_currentState = TrackerState::PausedForSplit;
                    emit splitDetectedAndPaused(m_wormId, originalFrameNumber, blobsInRoi); // Send all found blobs
                    return false; // Paused, don't advance frame counter here
                }
            }
            return true;
        } else {
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": Failed to find persisting component in merged mass.";
            // Fallback: track center of whole merged mass, or a more sophisticated "lost" procedure
            m_lastKnownPosition = cv::Point2f(static_cast<float>(currentMergedMass.centroid.x()), static_cast<float>(currentMergedMass.centroid.y()));
            QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, frame.size());
            m_lastPrimaryBlob = currentMergedMass; // Less ideal, but better than nothing.
            emit positionUpdated(m_wormId, originalFrameNumber, currentMergedMass.centroid, currentFixedSearchRoiRef, plausibleBlobsFound, currentMergedMass.area);
            currentFixedSearchRoiRef = nextFrameSearchRoi;
            return true; // Still "found" something.
        }
    } else {
        qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumber << ": In merged state but no valid m_lastPrimaryBlob. Tracking center of current mass.";
        m_lastKnownPosition = cv::Point2f(static_cast<float>(currentMergedMass.centroid.x()), static_cast<float>(currentMergedMass.centroid.y()));
        QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, frame.size());
        m_lastPrimaryBlob = currentMergedMass;
        emit positionUpdated(m_wormId, originalFrameNumber, currentMergedMass.centroid, currentFixedSearchRoiRef, plausibleBlobsFound, currentMergedMass.area);
        currentFixedSearchRoiRef = nextFrameSearchRoi;
        return true;
    }
}


void WormTracker::resumeTrackingWithNewTarget(const TrackingHelper::DetectedBlob& targetBlob) {
    qDebug() << "WormTracker ID" << m_wormId << ": resumeTrackingWithNewTarget called. Centroid:" << targetBlob.centroid;
    if (!targetBlob.isValid) {
        qWarning() << "WormTracker ID" << m_wormId << ": resumeTrackingWithNewTarget called with invalid blob. May stop tracking.";
        m_currentState = TrackerState::Idle;
        m_trackingActive = false;
        // No queued call to continueTracking here, as m_trackingActive = false will terminate the loop.
        return;
    }
    m_lastKnownPosition = cv::Point2f(static_cast<float>(targetBlob.centroid.x()), static_cast<float>(targetBlob.centroid.y()));
    // The ROI for the next search should be our standard fixed-size ROI, centered on the new target.
    m_currentSearchRoi = adjustRoiPos(m_lastKnownPosition, m_framesToProcess->at(m_currFrameNum).size()); // Use current frame for size
    m_lastPrimaryBlob = targetBlob;
    m_currentState = TrackerState::TrackingSingle;
    qDebug() << "WormTracker ID" << m_wormId << ": Resumed. New Search ROI:" << m_currentSearchRoi;
    // Advance frame counter because we've processed the decision for the current m_currFrameNum
    m_currFrameNum++;
    QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
}

void WormTracker::confirmTargetIsMerged(int mergedEntityID, const QPointF& mergedBlobCentroid, const QRectF& mergedBlobRoi) {
    Q_UNUSED(mergedEntityID);
    qDebug() << "WormTracker ID" << m_wormId << ": confirmTargetIsMerged. New Centroid:" << mergedBlobCentroid << "New ROI for search:" << mergedBlobRoi;
    m_currentState = TrackerState::TrackingMerged;
    m_lastKnownPosition = cv::Point2f(static_cast<float>(mergedBlobCentroid.x()), static_cast<float>(mergedBlobCentroid.y()));

    // When confirmed merged, the WormTracker's m_currentSearchRoi should be updated
    // to the ROI encompassing the whole merged blob, as dictated by TrackingManager.
    // However, for its *own* next search, it should still ideally use its fixed size ROI
    // centered on its best guess of its own position within the merge.
    // For now, let's assume mergedBlobRoi IS the new search ROI for the merged mass.
    // This implies TrackingManager provides a sensible ROI for the whole merge.
    m_currentSearchRoi = mergedBlobRoi;

    m_lastPrimaryBlob.isValid = true;
    m_lastPrimaryBlob.centroid = mergedBlobCentroid;
    m_lastPrimaryBlob.boundingBox = mergedBlobRoi;
    m_lastPrimaryBlob.area = mergedBlobRoi.width() * mergedBlobRoi.height(); // Approximate area

    // Don't advance frame counter here, let continueTracking pick up from current frame in new state.
    // QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection); // Not needed if already in loop
}


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

    // Consider a component significant only if its area is somewhat reasonable
    // This threshold (m_minBlobArea * 0.1) is arbitrary and might need tuning.
    if (largestContourIdx != -1 && maxArea > (m_minBlobArea * 0.1)) {
        const auto& largestContour = contours[largestContourIdx];
        cv::Moments mu = cv::moments(largestContour);
        if (mu.m00 > 0) { // Check for valid moments (non-zero area)
            resultBlob.centroid = QPointF(static_cast<qreal>(mu.m10 / mu.m00), static_cast<qreal>(mu.m01 / mu.m00));
            cv::Rect cvBox = cv::boundingRect(largestContour);
            resultBlob.boundingBox = QRectF(cvBox.x, cvBox.y, cvBox.width, cvBox.height);
            resultBlob.area = maxArea;
            resultBlob.contourPoints = largestContour; // Store the actual contour
            resultBlob.isValid = true;
            // resultBlob.touchesROIboundary is not applicable here as this is from a mask.

            qDebug() << "WormTracker ID" << m_wormId << ":" << debugContextName
                     << "component found. Centroid:" << resultBlob.centroid << "Area:" << resultBlob.area;
        }
    } else {
        qDebug() << "WormTracker ID" << m_wormId << ":" << debugContextName
                 << "found no significant blob. Max area found:" << maxArea << "(Threshold: >" << (m_minBlobArea * 0.1) << ")";
    }
    return resultBlob;
}


QList<TrackingHelper::DetectedBlob> WormTracker::findPlausibleBlobsInRoi(const cv::Mat& fullFrame, const QRectF& roi) {
    // This function now directly uses the TrackingHelper function
    // It passes the tracker's specific plausibility parameters.
    // Note: m_minAspectRatio and m_maxAspectRatio are currently defunct in WormTracker.
    // TrackingHelper::findAllPlausibleBlobsInRoi uses its own defaults or passed-in values.
    // If WormTracker needs to enforce specific aspect ratios, these members should be revived
    // and passed to the TrackingHelper function.
    return TrackingHelper::findAllPlausibleBlobsInRoi(fullFrame, roi,
                                                      m_minBlobArea, m_maxBlobArea,
                                                      TrackingConstants::DEFAULT_MIN_ASPECT_RATIO, // Using defaults from common
                                                      TrackingConstants::DEFAULT_MAX_ASPECT_RATIO);
}

QRectF WormTracker::adjustRoiPos(const cv::Point2f& wormCenter, const cv::Size& frameSize) {
    // Takes in a wormCenter and adjusts the fixed-size ROI (m_initialRoiEdge)
    // to be centered on this new position, clamped to frame boundaries.

    qreal roiWidth = m_initialRoiEdge;
    qreal roiHeight = m_initialRoiEdge; // Assuming square ROI
    qreal roiX = static_cast<qreal>(wormCenter.x) - roiWidth / 2.0;
    qreal roiY = static_cast<qreal>(wormCenter.y) - roiHeight / 2.0;

    // Clamp ROI's top-left to frame boundaries
    roiX = qMax(0.0, roiX);
    roiY = qMax(0.0, roiY);

    // Ensure ROI does not extend beyond frame boundaries from the right/bottom
    if (roiX + roiWidth > frameSize.width) {
        roiX = static_cast<qreal>(frameSize.width) - roiWidth;
    }
    if (roiY + roiHeight > frameSize.height) {
        roiY = static_cast<qreal>(frameSize.height) - roiHeight;
    }
    // Re-clamp after potential adjustment from right/bottom
    roiX = qMax(0.0, roiX);
    roiY = qMax(0.0, roiY);


    // Final check on width/height if frame is smaller than ROI (should not happen if ROI is reasonably sized)
    roiWidth = qMin(roiWidth, static_cast<qreal>(frameSize.width));
    roiHeight = qMin(roiHeight, static_cast<qreal>(frameSize.height));


    return QRectF(roiX, roiY, roiWidth, roiHeight);
}
