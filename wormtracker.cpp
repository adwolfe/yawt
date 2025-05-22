// wormtracker.cpp
#include "wormtracker.h" // Lowercase include
#include <QDebug>
#include <QtMath>       // For qSqrt, qPow, qAbs
#include <algorithm>    // For std::sort, std::min_element etc. if needed
#include <limits>       // For std::numeric_limits
#include <QLineF>


// Constants for boundary expansion logic
const int MAX_EXPANSION_ITERATIONS_BOUNDARY = 3;
const double BOUNDARY_ROI_EXPANSION_FACTOR = 1.25;
const double MERGE_CONFIRM_RELATIVE_AREA_FACTOR = 1.5; // For confirming merge after expansion
const double MERGE_CONFIRM_ABSOLUTE_AREA_FACTOR = 1.3; // For confirming merge against max typical area


WormTracker::WormTracker(int wormId,
                         QRectF initialRoi, // This is the fixed-size ROI
                         TrackingDirection direction,
                         int videoKeyFrameNum,
                         QObject *parent)
    : QObject(parent),
    m_wormId(wormId),
    m_direction(direction),
    m_framesToProcess(nullptr),
    m_initialRoiEdge(initialRoi.isValid() ? initialRoi.width() : 20.0), // Default if invalid
    m_minBlobArea(TrackingConstants::DEFAULT_MIN_WORM_AREA),
    m_maxBlobArea(TrackingConstants::DEFAULT_MAX_WORM_AREA),
    m_currentSearchRoi(initialRoi),
    m_lastKnownPosition(initialRoi.center().x(), initialRoi.center().y()),
    m_videoKeyFrameNum(videoKeyFrameNum),
    m_currFrameNum(0), // Initialize m_currFrameNum
    m_trackingActive(false),
    m_currentState(TrackerState::Idle)
{
    qDebug() << "WormTracker (" << this << ") created for worm ID:" << m_wormId
             << "Direction:" << (direction == TrackingDirection::Forward ? "Forward" : "Backward")
             << "Initial Fixed ROI:" << initialRoi << "Edge:" << m_initialRoiEdge;
    m_lastPrimaryBlob.isValid = false; // Ensure lastPrimaryBlob starts as invalid
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
    m_currentState = TrackerState::TrackingSingle; // Initial state when tracking starts
    m_lastPrimaryBlob.isValid = false; // Reset for a new tracking session
    m_currFrameNum = 0; // Reset frame counter for new tracking session

    qDebug() << "WormTracker ID" << m_wormId << ": Starting tracking. Frames:" << m_framesToProcess->size()
             << "Initial Search ROI:" << m_currentSearchRoi;

    // Start the processing loop
    QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
}

void WormTracker::continueTracking()
{
    if (QThread::currentThread()->isInterruptionRequested())
    {
        qDebug() << "WormTracker ID" << m_wormId << ": Tracking interrupted by thread request.";
        m_trackingActive = false; // Stop active tracking
    }

    if (m_trackingActive && m_currFrameNum < static_cast<int>(m_framesToProcess->size()))
    {
        const cv::Mat& currentFrame = (*m_framesToProcess)[m_currFrameNum];
        QRectF searchRoiForThisFrame = m_currentSearchRoi; // Capture the ROI used for *this* frame's search

        if (currentFrame.empty())
        {
            qWarning() << "WormTracker ID" << m_wormId << ": Encountered empty frame at sequence index" << m_currFrameNum;
            // Optionally emit an error or handle as lost
        }
        else
        {
            bool foundTargetThisFrame = false;
            if (!m_currentSearchRoi.isValid() || m_currentSearchRoi.isEmpty()) {
                qWarning() << "WormTracker ID" << m_wormId << "Frame" << (m_direction == TrackingDirection::Forward ? m_videoKeyFrameNum + m_currFrameNum : m_videoKeyFrameNum - 1 - m_currFrameNum)
                << ": Invalid m_currentSearchRoi before processing. Resetting based on last known position.";
                m_currentSearchRoi = adjustRoiPos(m_lastKnownPosition, currentFrame.size());
                searchRoiForThisFrame = m_currentSearchRoi; // Update captured ROI if it was reset
            }

            // Process based on state
            if (m_currentState == TrackerState::TrackingSingle || m_currentState == TrackerState::AmbiguouslySingle)
            {
                // Pass searchRoiForThisFrame by reference to be updated for the *next* frame's search
                foundTargetThisFrame = processFrameAsSingleWorm(currentFrame, m_currFrameNum, m_currentSearchRoi);
            }
            else if (m_currentState == TrackerState::TrackingMerged || m_currentState == TrackerState::AmbiguouslyMerged)
            {
                foundTargetThisFrame = processFrameAsMergedWorms(currentFrame, m_currFrameNum, m_currentSearchRoi);
            }
            // If PausedForSplit, foundTargetThisFrame will remain false, and we don't advance m_currFrameNum below

            if (!foundTargetThisFrame && m_currentState != TrackerState::PausedForSplit) {
                qDebug() << "WormTracker ID" << m_wormId << ": Target search at sequence index" << m_currFrameNum << "was skipped or unsuccessful; ROI for next frame remains" << m_currentSearchRoi;
                // If target not found, and not paused, we still need to emit positionUpdated with an invalid blob
                // This is handled within processFrameAsSingleWorm/MergedWorms if they determine no valid blob.
            }
        }

        // Advance frame and emit progress only if not paused
        if (m_currentState != TrackerState::PausedForSplit)
        {
            if (m_currFrameNum % 10 == 0 || m_currFrameNum == static_cast<int>(m_framesToProcess->size()) - 1) {
                // Ensure progress is emitted even for the last frame
                emit progress(m_wormId, static_cast<int>((static_cast<double>(m_currFrameNum + 1) / m_framesToProcess->size()) * 100.0));
            }
            m_currFrameNum++;
        }

        // Queue the next iteration if still active and not paused (or if paused, it will re-queue upon resume)
        if (m_trackingActive && (m_currentState != TrackerState::PausedForSplit || m_currFrameNum >= static_cast<int>(m_framesToProcess->size())) ) {
            // If paused but also at end of frames, let it finish the loop to emit 'finished'.
            QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
        } else if (!m_trackingActive) {
            // If tracking became inactive (e.g., due to interruption or stopTracking),
            // ensure the loop terminates and emits 'finished'.
            qDebug() << "WormTracker ID" << m_wormId << ": Tracking no longer active. Emitting finished.";
            emit progress(m_wormId, 100); // Ensure 100% progress if stopped early
            emit finished();
        }
        // If paused and not at end of frames, the loop effectively waits here until resumed.

    } else { // End of frames or tracking not active
        if (m_trackingActive) { // Loop completed naturally
            qDebug() << "WormTracker ID" << m_wormId << ": Frame processing loop completed naturally.";
            emit progress(m_wormId, 100);
            m_trackingActive = false; // Mark as no longer active
        }
        qDebug() << "WormTracker ID" << m_wormId << ": Finished processing. Final state:" << static_cast<int>(m_currentState);
        emit finished();
    }
}

void WormTracker::stopTracking() {
    qDebug() << "WormTracker ID" << m_wormId << ": stopTracking() called. Current state:" << static_cast<int>(m_currentState);
    m_trackingActive = false;
    // If the tracker is in a PausedForSplit state, calling stopTracking should also lead to emitting 'finished'.
    // The check `if (!m_trackingActive)` in continueTracking will handle this.
    // If it's in the middle of processing, the `isInterruptionRequested()` or `m_trackingActive` check will stop it.
    // We can invoke continueTracking to ensure the loop logic properly exits and emits 'finished'.
    QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
}

Tracking::DetectedBlob WormTracker::findPersistingComponent(
    const Tracking::DetectedBlob& previousFrameBlob,
    const Tracking::DetectedBlob& currentFrameBlob,
    const cv::Size& frameSize,
    int originalFrameNumberForDebug)
{
    Tracking::DetectedBlob persistingComponent;
    persistingComponent.isValid = false;

    if (!previousFrameBlob.isValid || previousFrameBlob.contourPoints.empty() ||
        !currentFrameBlob.isValid || currentFrameBlob.contourPoints.empty()) {
        if (originalFrameNumberForDebug != -1) { // Only log if debug frame number is provided
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                     << ": findPersistingComponent called with invalid input blobs.";
        }
        return persistingComponent;
    }

    cv::Mat prevBlobMask = cv::Mat::zeros(frameSize, CV_8UC1);
    cv::Mat currBlobMask = cv::Mat::zeros(frameSize, CV_8UC1);

    std::vector<std::vector<cv::Point>> prevContoursVec = {previousFrameBlob.contourPoints};
    cv::drawContours(prevBlobMask, prevContoursVec, 0, cv::Scalar(255), cv::FILLED);

    std::vector<std::vector<cv::Point>> currContoursVec = {currentFrameBlob.contourPoints};
    cv::drawContours(currBlobMask, currContoursVec, 0, cv::Scalar(255), cv::FILLED);

    cv::Mat newGrowthMask, oldOverlapMask, prevBlobMaskNot;
    cv::bitwise_not(prevBlobMask, prevBlobMaskNot);
    cv::bitwise_and(currBlobMask, prevBlobMaskNot, newGrowthMask);
    cv::bitwise_and(currBlobMask, prevBlobMask, oldOverlapMask);

    Tracking::DetectedBlob newGrowthComponentDetails = findLargestBlobComponentInMask(newGrowthMask, "NewGrowthAnalysis");
    Tracking::DetectedBlob oldOverlapComponentDetails = findLargestBlobComponentInMask(oldOverlapMask, "OldOverlapAnalysis");

    const double significanceThresholdFactor = 0.3;

    if (oldOverlapComponentDetails.isValid && oldOverlapComponentDetails.area > (m_minBlobArea * significanceThresholdFactor)) {
        if (originalFrameNumberForDebug != -1)
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                     << ": Persisting component identified as 'old overlap'. Centroid:" << oldOverlapComponentDetails.centroid
                     << "Area:" << oldOverlapComponentDetails.area;
        persistingComponent = oldOverlapComponentDetails;
    } else if (newGrowthComponentDetails.isValid && newGrowthComponentDetails.area > (m_minBlobArea * significanceThresholdFactor)) {
        if (originalFrameNumberForDebug != -1)
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                     << ": Persisting component identified as 'new growth' (old overlap was small/invalid). Centroid:" << newGrowthComponentDetails.centroid
                     << "Area:" << newGrowthComponentDetails.area;
        persistingComponent = newGrowthComponentDetails;
    } else {
        if (originalFrameNumberForDebug != -1)
            qDebug() << "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                     << ": Persisting component analysis inconclusive or components too small. Using full current blob.";
        persistingComponent = currentFrameBlob;
    }
    return persistingComponent;
}


bool WormTracker::processFrameAsSingleWorm(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentFixedSearchRoiRef_InOut)
{
    int originalFrameNumber;
    if (m_direction == TrackingDirection::Forward) {
        originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else {
        originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex; // KF is 0-indexed, so KF-1 is last frame for rev
    }

    // Capture the search ROI *used for this specific frame's detection* before it's modified for the next frame.
    QRectF searchRoiUsedForThisFrame = currentFixedSearchRoiRef_InOut;

    qDebug() << "WT" << m_wormId << (m_direction == TrackingDirection::Forward ? "Fwd" : "Bwd")
             << "Frame" << originalFrameNumber << "(Seq:" << sequenceFrameIndex << "): STATE_SINGLE|AMBIGUOUS - Searching in fixed ROI" << searchRoiUsedForThisFrame;

    QList<Tracking::DetectedBlob> blobsInFixedRoi = findPlausibleBlobsInRoi(frame, searchRoiUsedForThisFrame);
    int plausibleBlobsInFixedRoi = blobsInFixedRoi.count();

    Tracking::DetectedBlob bestBlobForThisFrame;
    bestBlobForThisFrame.isValid = false;
    TrackerState nextState = m_currentState;

    if (plausibleBlobsInFixedRoi == 0) {
        qDebug() << "  No plausible blobs found in fixed ROI " << searchRoiUsedForThisFrame;
        m_lastPrimaryBlob.isValid = false; // Update m_lastPrimaryBlob as no valid blob was found
        // Emit positionUpdated with an invalid blob
        Tracking::DetectedBlob invalidBlob; // Default constructor makes it invalid
        emit positionUpdated(m_wormId, originalFrameNumber, invalidBlob, searchRoiUsedForThisFrame, 0);
        // currentFixedSearchRoiRef_InOut (for next frame) remains unchanged if no blob found
        return false;
    }

    if (plausibleBlobsInFixedRoi == 1) {
        Tracking::DetectedBlob singleBlob = blobsInFixedRoi.first();
        qDebug() << "  WT" << m_wormId << "Frame" << originalFrameNumber <<"Found 1 plausible blob. BBox:" << singleBlob.boundingBox << "Area:" << singleBlob.area;

        bool touchesBoundary = singleBlob.touchesROIboundary ||
                               !searchRoiUsedForThisFrame.contains(singleBlob.boundingBox);

        if (!touchesBoundary) {
            qDebug() << "  Single blob fully contained. Treating as TrackingSingle.";
            bestBlobForThisFrame = singleBlob;
            nextState = TrackerState::TrackingSingle;
        } else {
            qDebug() << "  Single blob touches ROI boundary. Initiating expansion analysis.";
            QRectF analysisRoi = searchRoiUsedForThisFrame;
            Tracking::DetectedBlob candidateBlobAfterExpansion = singleBlob;

            for (int i = 0; i < MAX_EXPANSION_ITERATIONS_BOUNDARY; ++i) {
                qDebug() << "    Expansion iter #" << i + 1 << " Current analysis ROI:" << analysisRoi;
                qreal newWidth = analysisRoi.width() * BOUNDARY_ROI_EXPANSION_FACTOR;
                qreal newHeight = analysisRoi.height() * BOUNDARY_ROI_EXPANSION_FACTOR;
                QPointF center = candidateBlobAfterExpansion.isValid ? candidateBlobAfterExpansion.centroid : analysisRoi.center();
                analysisRoi.setSize(QSizeF(newWidth, newHeight));
                analysisRoi.moveCenter(center);
                analysisRoi.setX(qMax(0.0, analysisRoi.x()));
                analysisRoi.setY(qMax(0.0, analysisRoi.y()));
                if (analysisRoi.right() > frame.cols) analysisRoi.setRight(frame.cols);
                if (analysisRoi.bottom() > frame.rows) analysisRoi.setBottom(frame.rows);
                analysisRoi.setWidth(qMin(analysisRoi.width(), static_cast<qreal>(frame.cols)));
                analysisRoi.setHeight(qMin(analysisRoi.height(), static_cast<qreal>(frame.rows)));

                QList<Tracking::DetectedBlob> blobsInExpanded = findPlausibleBlobsInRoi(frame, analysisRoi);
                qDebug() << "    Found" << blobsInExpanded.count() << "blobs in expanded ROI:" << analysisRoi;

                if (blobsInExpanded.isEmpty()) {
                    qDebug() << "    No blobs in expanded ROI. Aborting expansion.";
                    break;
                }
                Tracking::DetectedBlob largestInExpanded = blobsInExpanded.first();
                if (m_lastPrimaryBlob.isValid) {
                    Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, largestInExpanded, frame.size(), originalFrameNumber);
                    candidateBlobAfterExpansion = persisted.isValid ? persisted : largestInExpanded;
                } else {
                    candidateBlobAfterExpansion = largestInExpanded;
                }
                qDebug() << "    After persistence/largest: candidate BBox:" << candidateBlobAfterExpansion.boundingBox << "Area:" << candidateBlobAfterExpansion.area;

                if (i == MAX_EXPANSION_ITERATIONS_BOUNDARY - 1 || analysisRoi.contains(candidateBlobAfterExpansion.boundingBox)) {
                    if (!analysisRoi.contains(candidateBlobAfterExpansion.boundingBox))
                        qDebug() << "    Max expansion iterations reached. Using current candidate.";
                    else
                        qDebug() << "    Candidate blob contained in expanded ROI. Stopping expansion.";
                } else continue; // continue if not contained and not max iterations

                if (blobsInExpanded.count() > 1) {
                    qDebug() << "    Multiple blobs (" << blobsInExpanded.count() << ") in final expansion. AmbiguouslyMerged.";
                    nextState = TrackerState::AmbiguouslyMerged; // Or TrackingMerged if area is large
                }
                break; // Break after decision or max iterations
            }
            bestBlobForThisFrame = candidateBlobAfterExpansion;

            if (nextState != TrackerState::AmbiguouslyMerged) {
                bool confirmedMerge = false;
                if (m_lastPrimaryBlob.isValid && bestBlobForThisFrame.isValid && bestBlobForThisFrame.area > m_lastPrimaryBlob.area * MERGE_CONFIRM_RELATIVE_AREA_FACTOR) {
                    confirmedMerge = true;
                } else if (bestBlobForThisFrame.isValid && bestBlobForThisFrame.area > m_maxBlobArea * MERGE_CONFIRM_ABSOLUTE_AREA_FACTOR) {
                    confirmedMerge = true;
                }
                if (confirmedMerge) {
                    qDebug() << "  Boundary touch + expansion + area heuristics CONFIRM MERGE. Area:" << bestBlobForThisFrame.area;
                    nextState = TrackerState::TrackingMerged;
                } else {
                    qDebug() << "  Boundary touch + expansion, NO merge by area. Area:" << bestBlobForThisFrame.area << ". TrackingSingle.";
                    nextState = TrackerState::TrackingSingle;
                }
            }
        }
    } else { // plausibleBlobsInFixedRoi > 1
        Tracking::DetectedBlob largestInFixedRoi = blobsInFixedRoi.first();
        qDebug() << "  Found >1 (" << plausibleBlobsInFixedRoi << ") plausible blobs. Largest BBox:" << largestInFixedRoi.boundingBox;

        bool largestTouchesBoundary = largestInFixedRoi.touchesROIboundary ||
                                      !searchRoiUsedForThisFrame.contains(largestInFixedRoi.boundingBox);

        if (!largestTouchesBoundary) {
            qDebug() << "  Largest blob fully contained. AmbiguouslySingle.";
            bestBlobForThisFrame = largestInFixedRoi; // Choose largest if multiple and contained
            nextState = TrackerState::AmbiguouslySingle;
        } else {
            qDebug() << "  Largest blob touches ROI boundary (with >1 blobs initially). Expansion for largest.";
            QRectF analysisRoi = searchRoiUsedForThisFrame;
            Tracking::DetectedBlob candidateBlobAfterExpansion = largestInFixedRoi;
            // Similar expansion logic as above for single touching blob
            for (int i = 0; i < MAX_EXPANSION_ITERATIONS_BOUNDARY; ++i) {
                // ... (Expansion logic as in the plausibleBlobsInFixedRoi == 1 case) ...
                // For brevity, assuming similar expansion logic here which updates candidateBlobAfterExpansion and nextState
                // This part would be a copy of the expansion loop from the case above,
                // ensuring candidateBlobAfterExpansion is the best guess for *our* worm.
                // The key difference is the initial context (already >1 blob).
                // After expansion, if candidateBlobAfterExpansion is chosen:
                qreal newWidth = analysisRoi.width() * BOUNDARY_ROI_EXPANSION_FACTOR;
                qreal newHeight = analysisRoi.height() * BOUNDARY_ROI_EXPANSION_FACTOR;
                QPointF center = candidateBlobAfterExpansion.isValid ? candidateBlobAfterExpansion.centroid : analysisRoi.center();
                analysisRoi.setSize(QSizeF(newWidth, newHeight));
                analysisRoi.moveCenter(center);
                // Clamp (same clamping as above)
                analysisRoi.setX(qMax(0.0, analysisRoi.x()));
                analysisRoi.setY(qMax(0.0, analysisRoi.y()));
                if (analysisRoi.right() > frame.cols) analysisRoi.setRight(frame.cols);
                if (analysisRoi.bottom() > frame.rows) analysisRoi.setBottom(frame.rows);
                analysisRoi.setWidth(qMin(analysisRoi.width(), static_cast<qreal>(frame.cols)));
                analysisRoi.setHeight(qMin(analysisRoi.height(), static_cast<qreal>(frame.rows)));


                QList<Tracking::DetectedBlob> blobsInExpanded = findPlausibleBlobsInRoi(frame, analysisRoi);
                if (blobsInExpanded.isEmpty()) { break; }
                Tracking::DetectedBlob currentLargestInExpanded = blobsInExpanded.first();
                if (m_lastPrimaryBlob.isValid) {
                    Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, currentLargestInExpanded, frame.size(), originalFrameNumber);
                    candidateBlobAfterExpansion = persisted.isValid ? persisted : currentLargestInExpanded;
                } else {
                    candidateBlobAfterExpansion = currentLargestInExpanded;
                }

                if (i == MAX_EXPANSION_ITERATIONS_BOUNDARY - 1 || analysisRoi.contains(candidateBlobAfterExpansion.boundingBox)) {
                    if (blobsInExpanded.count() > 1) { nextState = TrackerState::AmbiguouslyMerged; }
                    break;
                }
            }
            bestBlobForThisFrame = candidateBlobAfterExpansion;

            if (nextState != TrackerState::AmbiguouslyMerged) { // If not already set by expansion
                bool confirmedMerge = false;
                if (m_lastPrimaryBlob.isValid && bestBlobForThisFrame.isValid && bestBlobForThisFrame.area > m_lastPrimaryBlob.area * MERGE_CONFIRM_RELATIVE_AREA_FACTOR) {
                    confirmedMerge = true;
                } else if (bestBlobForThisFrame.isValid && bestBlobForThisFrame.area > m_maxBlobArea * MERGE_CONFIRM_ABSOLUTE_AREA_FACTOR) {
                    confirmedMerge = true;
                }
                if (confirmedMerge) {
                    nextState = TrackerState::TrackingMerged;
                } else {
                    nextState = TrackerState::AmbiguouslySingle; // Still ambiguous due to initial >1
                }
            }
        }
    }

    if (bestBlobForThisFrame.isValid) {
        if (m_currentState != nextState) {
            m_currentState = nextState;
            emit stateChanged(m_wormId, m_currentState);
            qDebug() << "  WT" << m_wormId << "Frame" << originalFrameNumber <<"State changed to:" << static_cast<int>(m_currentState);
        } else if ( (m_currentState == TrackerState::TrackingSingle && plausibleBlobsInFixedRoi > 1 && nextState == TrackerState::TrackingSingle && searchRoiUsedForThisFrame.contains(bestBlobForThisFrame.boundingBox)) ) {
            // If was single, but now >1 blobs in fixed ROI and chosen blob is contained (no boundary issue forced single)
            // then it should become AmbiguouslySingle
            m_currentState = TrackerState::AmbiguouslySingle;
            emit stateChanged(m_wormId, m_currentState);
            qDebug() << "  WT" << m_wormId << "Frame" << originalFrameNumber <<"State overridden to AmbiguouslySingle due to >1 initial blobs fully contained.";
        }


        m_lastKnownPosition = cv::Point2f(static_cast<float>(bestBlobForThisFrame.centroid.x()), static_cast<float>(bestBlobForThisFrame.centroid.y()));
        QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, frame.size());
        m_lastPrimaryBlob = bestBlobForThisFrame; // Update for next frame's persistence check

        emit positionUpdated(m_wormId, originalFrameNumber, bestBlobForThisFrame, searchRoiUsedForThisFrame, plausibleBlobsInFixedRoi);
        qDebug() << "  WT" << m_wormId << "Frame" << originalFrameNumber <<"Position updated. Blob BBox:" << bestBlobForThisFrame.boundingBox
                 << "Search ROI used:" << searchRoiUsedForThisFrame << "Next Search ROI:" << nextFrameSearchRoi;

        currentFixedSearchRoiRef_InOut = nextFrameSearchRoi; // Update ROI for the next frame
        return true;
    } else {
        qDebug() << "  No valid bestBlobForThisFrame. This is unexpected if plausibleBlobsInFixedRoi > 0.";
        m_lastPrimaryBlob.isValid = false;
        Tracking::DetectedBlob invalidBlob;
        emit positionUpdated(m_wormId, originalFrameNumber, invalidBlob, searchRoiUsedForThisFrame, plausibleBlobsInFixedRoi);
        // currentFixedSearchRoiRef_InOut remains unchanged
        return false;
    }
}

bool WormTracker::processFrameAsMergedWorms(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentFixedSearchRoiRef_InOut)
{
    int originalFrameNumber;
    if (m_direction == TrackingDirection::Forward) {
        originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else {
        originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex;
    }

    QRectF searchRoiUsedForThisFrame = currentFixedSearchRoiRef_InOut;

    qDebug() << "WT" << m_wormId << (m_direction == TrackingDirection::Forward ? "Fwd" : "Bwd")
             << "Frame" << originalFrameNumber << "(Seq:" << sequenceFrameIndex << "): STATE_MERGED - Searching in ROI" << searchRoiUsedForThisFrame;

    QList<Tracking::DetectedBlob> blobsInRoi = findPlausibleBlobsInRoi(frame, searchRoiUsedForThisFrame);
    int plausibleBlobsFound = blobsInRoi.count();

    if (plausibleBlobsFound == 0) {
        qDebug() << "  No plausible blobs found in MERGED ROI " << searchRoiUsedForThisFrame;
        m_lastPrimaryBlob.isValid = false;
        Tracking::DetectedBlob invalidBlob;
        emit positionUpdated(m_wormId, originalFrameNumber, invalidBlob, searchRoiUsedForThisFrame, 0);
        return false;
    }

    // The largest blob in the current (potentially large, manager-defined) ROI is assumed to be the merged mass.
    Tracking::DetectedBlob currentMergedMass = blobsInRoi.first();
    Tracking::DetectedBlob ourComponentInMerge; // This will be our worm's part

    if (m_lastPrimaryBlob.isValid) {
        // Try to find our worm's component within the current merged mass based on the last frame's blob
        ourComponentInMerge = findPersistingComponent(m_lastPrimaryBlob, currentMergedMass, frame.size(), originalFrameNumber);
        if (!ourComponentInMerge.isValid) { // Fallback if persistence fails
            qDebug() << "  WT" << m_wormId << "Frame" << originalFrameNumber << ": Failed to find persisting component in merged mass. Using centroid of whole mass.";
            // Create a blob representing the whole mass if persistence fails
            // This is a simplification; ideally, we'd still try to estimate our part.
            // For now, if findPersistingComponent returns invalid, we might consider the whole currentMergedMass
            // as what we are "following", but this is less precise.
            // A better fallback might be to use the centroid of currentMergedMass but with a smaller, typical worm area/bbox.
            // For now, let's assume findPersistingComponent returns currentMergedMass if it can't find a distinct part.
            ourComponentInMerge = currentMergedMass; // Or a more refined fallback.
        }
    } else {
        qDebug() << "  WT" << m_wormId << "Frame" << originalFrameNumber << ": In merged state but no valid m_lastPrimaryBlob. Using largest blob in ROI.";
        // If no prior blob, our best guess is the largest component in the current ROI,
        // assuming TrackingManager set this ROI to cover the merge.
        ourComponentInMerge = currentMergedMass;
    }


    if (ourComponentInMerge.isValid) {
        m_lastKnownPosition = cv::Point2f(static_cast<float>(ourComponentInMerge.centroid.x()), static_cast<float>(ourComponentInMerge.centroid.y()));
        QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, frame.size()); // Re-center our fixed-size ROI for next personal search
        m_lastPrimaryBlob = ourComponentInMerge; // Update with our component

        emit positionUpdated(m_wormId, originalFrameNumber, ourComponentInMerge, searchRoiUsedForThisFrame, plausibleBlobsFound);
        currentFixedSearchRoiRef_InOut = nextFrameSearchRoi; // Update our tracker's personal search ROI

        // Split detection: If multiple significant blobs are now present in the *original search ROI*
        // where we previously expected one merged mass.
        if (plausibleBlobsFound > 1) {
            // Check if the main "merged mass" (currentMergedMass) is still significantly larger than our component,
            // AND if other blobs in blobsInRoi are also significant.
            // This indicates the larger mass may have split.
            bool potentialSplit = false;
            if (ourComponentInMerge.area < currentMergedMass.area * 0.75 && blobsInRoi.count() > 1) { // Our part is smaller than the whole
                // Check if other blobs are also substantial (not just tiny noise)
                int significantOtherBlobs = 0;
                for(int i = 0; i < blobsInRoi.count(); ++i) {
                    // Don't count ourComponentInMerge itself if it's one of the blobsInRoi directly
                    // This check is tricky if ourComponentInMerge is a sub-part.
                    // A simpler check: are there at least two blobs in blobsInRoi that are of plausible worm size?
                    if (blobsInRoi.at(i).area > m_minBlobArea) {
                        significantOtherBlobs++;
                    }
                }
                if (significantOtherBlobs >= 2) { // At least two "worm-sized" things appeared
                    potentialSplit = true;
                }
            }


            if (potentialSplit) {
                qDebug() << "  WT" << m_wormId << "Frame" << originalFrameNumber
                         << ": Potential SPLIT detected from merged state. Pausing. Found" << plausibleBlobsFound << "blobs in ROI.";
                m_currentState = TrackerState::PausedForSplit;
                emit stateChanged(m_wormId, m_currentState);
                emit splitDetectedAndPaused(m_wormId, originalFrameNumber, blobsInRoi); // Send all blobs found in searchRoiUsedForThisFrame
                return false; // Paused, don't advance frame counter in continueTracking
            }
        }
        return true;
    } else {
        qDebug() << "  WT" << m_wormId << "Frame" << originalFrameNumber << ": In merged state, but no valid component determined. This is unexpected.";
        m_lastPrimaryBlob.isValid = false;
        Tracking::DetectedBlob invalidBlob;
        emit positionUpdated(m_wormId, originalFrameNumber, invalidBlob, searchRoiUsedForThisFrame, plausibleBlobsFound);
        return false;
    }
}


void WormTracker::resumeTrackingWithNewTarget(const Tracking::DetectedBlob& targetBlob) {
    qDebug() << "WormTracker ID" << m_wormId << ": resumeTrackingWithNewTarget called. Blob Centroid:" << targetBlob.centroid << "Area:" << targetBlob.area;
    if (!targetBlob.isValid) {
        qWarning() << "WormTracker ID" << m_wormId << ": resumeTrackingWithNewTarget called with invalid blob. Stopping tracking.";
        m_currentState = TrackerState::Idle; // Or a 'Lost' state
        m_trackingActive = false;
        // No queued call to continueTracking here, as m_trackingActive = false will terminate the loop via stopTracking's invoke.
        // Or, directly emit finished if appropriate.
        // Let stopTracking handle the proper shutdown.
        QMetaObject::invokeMethod(this, &WormTracker::stopTracking, Qt::QueuedConnection);
        return;
    }

    m_lastKnownPosition = cv::Point2f(static_cast<float>(targetBlob.centroid.x()), static_cast<float>(targetBlob.centroid.y()));
    // The ROI for the next search should be our standard fixed-size ROI, centered on the new target.
    // Ensure m_framesToProcess is valid and m_currFrameNum is within bounds before accessing.
    cv::Size currentFrameSize;
    if (m_framesToProcess && m_currFrameNum < static_cast<int>(m_framesToProcess->size()) && m_currFrameNum >=0 ) {
        currentFrameSize = m_framesToProcess->at(m_currFrameNum).size();
    } else if (m_framesToProcess && !m_framesToProcess->empty()) {
        currentFrameSize = m_framesToProcess->at(0).size(); // Fallback to first frame size
        qWarning() << "WormTracker ID" << m_wormId << ": m_currFrameNum invalid in resume. Using first frame size for ROI.";
    } else {
        qWarning() << "WormTracker ID" << m_wormId << ": No frames available in resume. Cannot set ROI size. Tracking may fail.";
        // Default to a reasonable size if frame info is totally missing, though this is bad.
        currentFrameSize = cv::Size(640,480); // Arbitrary fallback
    }

    m_currentSearchRoi = adjustRoiPos(m_lastKnownPosition, currentFrameSize);
    m_lastPrimaryBlob = targetBlob; // This is now the confirmed target
    m_currentState = TrackerState::TrackingSingle; // Resume as tracking a single, confirmed target
    emit stateChanged(m_wormId, m_currentState);

    qDebug() << "WormTracker ID" << m_wormId << ": Resumed. New Search ROI:" << m_currentSearchRoi << "State:" << static_cast<int>(m_currentState);

    // Important: The tracker was paused at m_currFrameNum because a split was detected *in that frame*.
    // Now that it's resuming, it should process the *next* frame.
    m_currFrameNum++;

    // Re-activate the tracking loop
    if (m_trackingActive) { // Only if stopTracking wasn't called due to invalid blob
        QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
    }
}

void WormTracker::confirmTargetIsMerged(int mergedEntityID, const QPointF& mergedBlobCentroid, const QRectF& mergedBlobRoiFromManager) {
    Q_UNUSED(mergedEntityID); // mergedEntityID is for manager's context, tracker uses its own m_wormId
    qDebug() << "WormTracker ID" << m_wormId << ": confirmTargetIsMerged by TrackingManager. New Centroid (approx):" << mergedBlobCentroid
             << "New Search ROI for merged mass:" << mergedBlobRoiFromManager;

    m_currentState = TrackerState::TrackingMerged;
    emit stateChanged(m_wormId, m_currentState, mergedEntityID); // Pass manager's representative ID

    m_lastKnownPosition = cv::Point2f(static_cast<float>(mergedBlobCentroid.x()), static_cast<float>(mergedBlobCentroid.y()));

    // When TrackingManager confirms a merge, it might provide a larger ROI that encompasses the whole merged entity.
    // The WormTracker should use this manager-provided ROI for its searches while in the merged state.
    if (mergedBlobRoiFromManager.isValid()) {
        m_currentSearchRoi = mergedBlobRoiFromManager;
    } else {
        qWarning() << "WormTracker ID" << m_wormId << ": TrackingManager provided invalid ROI for merge. Using self-adjusted ROI.";
        // Fallback: center its own fixed-size ROI on the provided centroid.
        // This might be too small if the merged entity is large.
        // It's better if TM always provides a valid, encompassing ROI.
        if (m_framesToProcess && m_currFrameNum < static_cast<int>(m_framesToProcess->size()) && m_currFrameNum >= 0) {
            m_currentSearchRoi = adjustRoiPos(m_lastKnownPosition, m_framesToProcess->at(m_currFrameNum).size());
        } else {
            qWarning() << "WormTracker ID" << m_wormId << ": Cannot adjust ROI in confirmTargetIsMerged due to invalid frame data.";
        }
    }


    // Update m_lastPrimaryBlob to represent the (approximate) merged entity.
    // This is tricky because we don't have the *exact* contour of our part yet.
    // We use the manager-provided info as the best guess for the overall merged blob.
    m_lastPrimaryBlob.isValid = true;
    m_lastPrimaryBlob.centroid = mergedBlobCentroid;
    m_lastPrimaryBlob.boundingBox = mergedBlobRoiFromManager.isValid() ? mergedBlobRoiFromManager : m_currentSearchRoi; // Use the ROI as bbox
    m_lastPrimaryBlob.area = m_lastPrimaryBlob.boundingBox.width() * m_lastPrimaryBlob.boundingBox.height(); // Approximate area
    m_lastPrimaryBlob.contourPoints.clear(); // Contour is unknown for the whole merge from tracker's perspective

    // Don't advance frame counter here. The continueTracking loop will process the current m_currFrameNum
    // with the new TrackingMerged state and the updated m_currentSearchRoi.
}


Tracking::DetectedBlob WormTracker::findLargestBlobComponentInMask(const cv::Mat& mask, const QString& debugContextName) {
    Tracking::DetectedBlob resultBlob;
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

    if (largestContourIdx != -1 && maxArea > (m_minBlobArea * 0.1)) { // Threshold for significance
        const auto& largestContour = contours[largestContourIdx];
        cv::Moments mu = cv::moments(largestContour);
        if (mu.m00 > std::numeric_limits<double>::epsilon()) { // Check for valid moments (non-zero area)
            resultBlob.centroid = QPointF(static_cast<qreal>(mu.m10 / mu.m00), static_cast<qreal>(mu.m01 / mu.m00));
            cv::Rect cvBox = cv::boundingRect(largestContour);
            resultBlob.boundingBox = QRectF(cvBox.x, cvBox.y, cvBox.width, cvBox.height);
            resultBlob.area = maxArea;
            resultBlob.contourPoints = largestContour;
            resultBlob.isValid = true;
            // touchesROIboundary is not applicable here as this is from a mask, not a search ROI.

            // qDebug() << "WormTracker ID" << m_wormId << ":" << debugContextName
            //          << "component found. Centroid:" << resultBlob.centroid << "Area:" << resultBlob.area;
        }
    } else {
        // qDebug() << "WormTracker ID" << m_wormId << ":" << debugContextName
        //          << "found no significant blob. Max area found:" << maxArea;
    }
    return resultBlob;
}


QList<Tracking::DetectedBlob> WormTracker::findPlausibleBlobsInRoi(const cv::Mat& fullFrame, const QRectF& roi) {
    return Tracking::findAllPlausibleBlobsInRoi(fullFrame, roi,
                                                m_minBlobArea, m_maxBlobArea,
                                                TrackingConstants::DEFAULT_MIN_ASPECT_RATIO,
                                                TrackingConstants::DEFAULT_MAX_ASPECT_RATIO);
}

QRectF WormTracker::adjustRoiPos(const cv::Point2f& wormCenter, const cv::Size& frameSize) {
    qreal roiWidth = m_initialRoiEdge;
    qreal roiHeight = m_initialRoiEdge;
    qreal roiX = static_cast<qreal>(wormCenter.x) - roiWidth / 2.0;
    qreal roiY = static_cast<qreal>(wormCenter.y) - roiHeight / 2.0;

    roiX = qMax(0.0, roiX);
    roiY = qMax(0.0, roiY);

    if (roiX + roiWidth > frameSize.width) {
        roiX = static_cast<qreal>(frameSize.width) - roiWidth;
    }
    if (roiY + roiHeight > frameSize.height) {
        roiY = static_cast<qreal>(frameSize.height) - roiHeight;
    }
    roiX = qMax(0.0, roiX); // Re-clamp after right/bottom adjustment
    roiY = qMax(0.0, roiY);

    // Ensure ROI width/height are not larger than frame itself (e.g. if frame is very small)
    roiWidth = qMin(roiWidth, static_cast<qreal>(frameSize.width));
    roiHeight = qMin(roiHeight, static_cast<qreal>(frameSize.height));

    return QRectF(roiX, roiY, roiWidth, roiHeight);
}

