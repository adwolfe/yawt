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
const double MERGE_CONFIRM_RELATIVE_AREA_FACTOR = 1.3; // For confirming merge after expansion
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
    m_currentState(Tracking::TrackerState::Idle)
{
    qDebug().noquote()<< "WormTracker (" << this << ") created for worm ID:" << m_wormId
             << "Direction:" << (direction == TrackingDirection::Forward ? "Forward" : "Backward")
             << "Initial Fixed ROI:" << initialRoi << "Edge:" << m_initialRoiEdge;
    m_lastPrimaryBlob.isValid = false; // Ensure lastPrimaryBlob starts as invalid
}

WormTracker::~WormTracker() {
    qDebug().noquote()<< "WormTracker (" << this << ") destroyed for worm ID:" << m_wormId;
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
    m_currentState = Tracking::TrackerState::TrackingSingle; // Initial state when tracking starts
    m_lastPrimaryBlob.isValid = false; // Reset for a new tracking session
    m_currFrameNum = 0; // Reset frame counter for new tracking session

    qDebug().noquote()<< "WormTracker ID" << m_wormId << ": Starting tracking. Frames:" << m_framesToProcess->size()
             << "Initial Search ROI:" << m_currentSearchRoi;

    // Start the processing loop
    QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
}

void WormTracker::continueTracking()
{
    if (QThread::currentThread()->isInterruptionRequested())
    {
        qDebug().noquote()<< "WormTracker ID" << m_wormId << ": Tracking interrupted by thread request.";
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
            if (m_currentState == Tracking::TrackerState::TrackingSingle || m_currentState == Tracking::TrackerState::TrackingMerged)
            {
                // Pass searchRoiForThisFrame by reference to be updated for the *next* frame's search
                foundTargetThisFrame = processFrame(currentFrame, m_currFrameNum, m_currentSearchRoi);
            }
            // If PausedForSplit, foundTargetThisFrame will remain false, and we don't advance m_currFrameNum below

            if (!foundTargetThisFrame && m_currentState != Tracking::TrackerState::PausedForSplit) {
                qDebug().noquote()<< "WormTracker ID" << m_wormId << ": Target search at sequence index" << m_currFrameNum << "was skipped or unsuccessful; ROI for next frame remains" << m_currentSearchRoi;
                // If target not found, and not paused, we still need to emit positionUpdated with an invalid blob
                // This is handled within processFrameAsSingleWorm/MergedWorms if they determine no valid blob.
            }
        }

        // Advance frame and emit progress only if not paused
        if (m_currentState != Tracking::TrackerState::PausedForSplit)
        {
            if (m_currFrameNum % 10 == 0 || m_currFrameNum == static_cast<int>(m_framesToProcess->size()) - 1) {
                // Ensure progress is emitted even for the last frame
                emit progress(m_wormId, static_cast<int>((static_cast<double>(m_currFrameNum + 1) / m_framesToProcess->size()) * 100.0));
            }
            m_currFrameNum++;
        }

        // Queue the next iteration if still active and not paused (or if paused, it will re-queue upon resume)
        if (m_trackingActive && (m_currentState != Tracking::TrackerState::PausedForSplit || m_currFrameNum >= static_cast<int>(m_framesToProcess->size())) ) {
            // If paused but also at end of frames, let it finish the loop to emit 'finished'.
            QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
        } else if (!m_trackingActive) {
            // If tracking became inactive (e.g., due to interruption or stopTracking),
            // ensure the loop terminates and emits 'finished'.
            qDebug().noquote()<< "WormTracker ID" << m_wormId << ": Tracking no longer active. Emitting finished.";
            emit progress(m_wormId, 100); // Ensure 100% progress if stopped early
            emit finished();
        }
        // If paused and not at end of frames, the loop effectively waits here until resumed.

    } else { // End of frames or tracking not active
        if (m_trackingActive) { // Loop completed naturally
            qDebug().noquote()<< "WormTracker ID" << m_wormId << ": Frame processing loop completed naturally.";
            emit progress(m_wormId, 100);
            m_trackingActive = false; // Mark as no longer active
        }
        qDebug().noquote()<< "WormTracker ID" << m_wormId << ": Finished processing. Final state:" << static_cast<int>(m_currentState);
        emit finished();
    }
}

void WormTracker::stopTracking() {
    qDebug().noquote()<< "WormTracker ID" << m_wormId << ": stopTracking() called. Current state:" << static_cast<int>(m_currentState);
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
            qDebug().noquote()<< "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
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
            qDebug().noquote()<< "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                     << ": Persisting component identified as 'old overlap'. Centroid:" << oldOverlapComponentDetails.centroid
                     << "Area:" << oldOverlapComponentDetails.area;
        persistingComponent = oldOverlapComponentDetails;
    } else if (newGrowthComponentDetails.isValid && newGrowthComponentDetails.area > (m_minBlobArea * significanceThresholdFactor)) {
        if (originalFrameNumberForDebug != -1)
            qDebug().noquote()<< "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                     << ": Persisting component identified as 'new growth' (old overlap was small/invalid). Centroid:" << newGrowthComponentDetails.centroid
                     << "Area:" << newGrowthComponentDetails.area;
        persistingComponent = newGrowthComponentDetails;
    } else {
        if (originalFrameNumberForDebug != -1)
            qDebug().noquote()<< "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                     << ": Persisting component analysis inconclusive or components too small. Using full current blob.";
        persistingComponent = currentFrameBlob;
    }
    return persistingComponent;
}


bool WormTracker::processFrame(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentFixedSearchRoiRef_InOut)
// Here's the plan: we need to track our worm, even when near other worms.
// If the ROI touches the boundary, then we check whether the size of the blob after expansion is large enough to be a merge.
// If it is not, then we store the full blob and our worm blob as same area and calculate centroid.
// If it is, then we store the full blob, but calculate centroid on the persistent part.
// This way, we can still get a sense of what our worm is within the merged worms.
// Once merged we alert TM and the other function takes over.
{
    int originalFrameNumber;
    int debugId = m_wormId;

    if (m_direction == TrackingDirection::Forward) {
        originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else {
        originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex; // KF is 0-indexed, so KF-1 is last frame for rev
        debugId = -1*m_wormId;
    }
    // Cleaner debug prefix
    QString dmsg = QString("WT ") + QString::number(debugId) + QString(" ") + QString::number(originalFrameNumber) + QString(": ");

    // Capture the search ROI *used for this specific frame's detection* before it's modified for the next frame.
    QRectF searchRoiUsedForThisFrame = currentFixedSearchRoiRef_InOut;

    qDebug().noquote()<< dmsg << m_currentState << "Search begins with" << searchRoiUsedForThisFrame;

    QList<Tracking::DetectedBlob> blobsInFixedRoi = findPlausibleBlobsInRoi(frame, searchRoiUsedForThisFrame);
    int plausibleBlobsInFixedRoi = blobsInFixedRoi.count();

    Tracking::DetectedBlob bestBlobForThisFrame;
    bestBlobForThisFrame.isValid = false;
    Tracking::TrackerState nextState = m_currentState;

    if (plausibleBlobsInFixedRoi == 0) {
        qDebug().noquote()<< dmsg << "No plausible blobs found in initial ROI; returning false" << searchRoiUsedForThisFrame;
        m_lastPrimaryBlob.isValid = false; // Update m_lastPrimaryBlob as no valid blob was found
        // Emit positionUpdated with an invalid blob
        Tracking::DetectedBlob invalidBlob; // Default constructor makes it invalid
        emit positionUpdated(m_wormId, originalFrameNumber, invalidBlob, searchRoiUsedForThisFrame, Tracking::TrackerState::TrackingLost, 0);
        // currentFixedSearchRoiRef_InOut (for next frame) remains unchanged if no blob found
        return false;
    }

    if (plausibleBlobsInFixedRoi == 1) {
        // Whether merged or single, if only one blob is found it doesn't change our interpretation
        Tracking::DetectedBlob singleBlob = blobsInFixedRoi.first();
        qDebug().noquote()<< dmsg <<"Found 1 plausible blob. BBox:" << singleBlob.boundingBox << "Area:" << singleBlob.area;

        bool touchesBoundary = singleBlob.touchesROIboundary ||
                               !searchRoiUsedForThisFrame.contains(singleBlob.boundingBox);

        if (!touchesBoundary) {
            qDebug().noquote()<< dmsg << "Single blob fully contained. Treating as TrackingSingle.";
            bestBlobForThisFrame = singleBlob;
            nextState = Tracking::TrackerState::TrackingSingle;
        } else { // touches boundary -- possibly a merge, or could be just moving too fast
            qDebug().noquote()<< dmsg << "Single blob touches ROI boundary. Initiating expansion analysis.";
            QRectF analysisRoi = searchRoiUsedForThisFrame;
            Tracking::DetectedBlob candidateBlobAfterExpansion = singleBlob; // Initialize with the single blob

            for (int i = 0; i < MAX_EXPANSION_ITERATIONS_BOUNDARY; ++i) {
                qDebug().noquote()<< dmsg << "Expansion iter #" << i + 1 << " Current analysis ROI:" << analysisRoi;
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
                qDebug().noquote()<< dmsg << "Found" << blobsInExpanded.count() << "blobs in expanded ROI:" << analysisRoi;

                if (blobsInExpanded.isEmpty()) {
                    qDebug().noquote()<< dmsg << "No blobs in expanded ROI. Aborting expansion.";
                    // candidateBlobAfterExpansion remains what it was from the previous iteration or initial assignment
                    break;
                }


                candidateBlobAfterExpansion = blobsInExpanded.first();
                qDebug().noquote()<< dmsg << "Found largest candidate... BBox:" << candidateBlobAfterExpansion.boundingBox << "Area:" << candidateBlobAfterExpansion.area;

                if (i == MAX_EXPANSION_ITERATIONS_BOUNDARY - 1 || analysisRoi.contains(candidateBlobAfterExpansion.boundingBox)) {
                    if (!analysisRoi.contains(candidateBlobAfterExpansion.boundingBox))
                        qDebug().noquote()<< dmsg << "Max expansion iterations reached. Using current candidate.";
                    else
                        qDebug().noquote()<< dmsg << "Candidate blob contained in expanded ROI. Stopping expansion.";
                    // Decision made, break from expansion loop
                    break;
                } else {
                    // continue if not contained and not max iterations
                    continue;
                }
                // Unreachable due to break/continue above, but if logic changes, ensure break is hit.
                break; // Break after decision or max iterations
            }

            // Now that we've got our blob, figure out whether it is a merge
            if (m_lastPrimaryBlob.isValid) {
                // Find the persisting component for centroid purposes, but we're storing the whole blob.
                Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, candidateBlobAfterExpansion, frame.size(), originalFrameNumber);
                bestBlobForThisFrame = candidateBlobAfterExpansion;
                bestBlobForThisFrame.centroid = persisted.isValid ? persisted.centroid : candidateBlobAfterExpansion.centroid;
            } else {
                // use candidate from expansion
                bestBlobForThisFrame = candidateBlobAfterExpansion;
            }
            bool confirmedMerge = false;



            // NOTE -- THIS NEEDS TO TAKE INTO ACCOUNT IF MERGED PREVIOUSLY BECAUSE IF SO THE COMPARISON IS NOT EFFECTIVE
            qDebug().noquote()<< dmsg << "Checking if merged..." << m_lastPrimaryBlob.isValid << bestBlobForThisFrame.isValid << candidateBlobAfterExpansion.area << m_lastPrimaryBlob.area;


            if (m_lastPrimaryBlob.isValid && bestBlobForThisFrame.isValid && candidateBlobAfterExpansion.area > m_lastPrimaryBlob.area * MERGE_CONFIRM_RELATIVE_AREA_FACTOR) {
                confirmedMerge = true;
            } else if (bestBlobForThisFrame.isValid && candidateBlobAfterExpansion.area > m_maxBlobArea * MERGE_CONFIRM_ABSOLUTE_AREA_FACTOR) {
                confirmedMerge = true;
            }

            if (confirmedMerge) {
                qDebug().noquote()<< dmsg << "Boundary touch + expansion + area heuristics CONFIRM MERGE. Area:" << candidateBlobAfterExpansion.area << "-- we are using persisted area" << bestBlobForThisFrame.area;
                nextState = Tracking::TrackerState::TrackingMerged;
            } else {
                qDebug().noquote()<< dmsg << "Boundary touch + expansion, NO merge by area. Area:" << bestBlobForThisFrame.area << ".";
                if (m_currentState != Tracking::TrackerState::TrackingMerged) {
                    nextState = Tracking::TrackerState::TrackingSingle;
                    qDebug().noquote()<< dmsg << "Setting state to TrackingSingle.";
                } else {
                    nextState = Tracking::TrackerState::PausedForSplit; // Was merged, now one small blob -> likely a resolved split
                    qDebug().noquote()<< dmsg << "Was merged, now one small blob. Setting state to PausedForSplit.";
                }
            }
        }
    } else { // plausibleBlobsInFixedRoi > 1
        // here, it matters whether we were merged previously or not. if not merged previously, it can't be a split.
        if (m_currentState != Tracking::TrackerState::TrackingMerged) {
            Tracking::DetectedBlob largestInFixedRoi = blobsInFixedRoi.first();
            qDebug().noquote()<< dmsg << "Found >1 (" << plausibleBlobsInFixedRoi << ") plausible blobs (not previously merged). Largest BBox:" << largestInFixedRoi.boundingBox << "Area:" << largestInFixedRoi.area;

            bool largestTouchesBoundary = largestInFixedRoi.touchesROIboundary ||
                                          !searchRoiUsedForThisFrame.contains(largestInFixedRoi.boundingBox);

            if (!largestTouchesBoundary) {
                qDebug().noquote()<< dmsg << "Largest blob fully contained, target acquired.";
                bestBlobForThisFrame = largestInFixedRoi; // Choose largest if multiple and contained
                nextState = Tracking::TrackerState::TrackingSingle;
            } else {
                qDebug().noquote()<< dmsg << "Largest blob touches ROI boundary (with >1 blobs initially, not merged). Expansion for largest.";
                QRectF analysisRoi = searchRoiUsedForThisFrame;
                Tracking::DetectedBlob candidateBlobAfterExpansion = largestInFixedRoi; // Initialize with the largest blob

                for (int i = 0; i < MAX_EXPANSION_ITERATIONS_BOUNDARY; ++i) {
                    qDebug().noquote()<< dmsg << "Expansion iter #" << i + 1 << " Current analysis ROI:" << analysisRoi;
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
                    qDebug().noquote()<< dmsg << "Found" << blobsInExpanded.count() << "blobs in expanded ROI:" << analysisRoi;

                    if (blobsInExpanded.isEmpty()) {
                        qDebug().noquote()<< dmsg << "No blobs in expanded ROI. Aborting expansion.";
                        // candidateBlobAfterExpansion remains what it was
                        break;
                    }


                    candidateBlobAfterExpansion = blobsInExpanded.first();
                    qDebug().noquote()<< dmsg << "After expansion, largest candidate: BBox:" << candidateBlobAfterExpansion.boundingBox << "Area:" << candidateBlobAfterExpansion.area;

                    if (i == MAX_EXPANSION_ITERATIONS_BOUNDARY - 1 || analysisRoi.contains(candidateBlobAfterExpansion.boundingBox)) {
                        if (!analysisRoi.contains(candidateBlobAfterExpansion.boundingBox))
                            qDebug().noquote()<< dmsg << "Max expansion iterations reached. Using current candidate.";
                        else
                            qDebug().noquote()<< dmsg << "Candidate blob contained in expanded ROI. Stopping expansion.";
                        break;
                    } else {
                        continue;
                    }
                    // if (blobsInExpanded.count() > 1) {
                    //     qDebug().noquote()<< dmsg << "Multiple blobs (" << blobsInExpanded.count() << ") still found in final expansion but we're ignoring for now.";
                    // } else if (blobsInExpanded.count() == 1) {
                    //     qDebug().noquote()<< dmsg << "Multiple blobs resolved into single blob, this is probably a merge";
                    // }
                    break; // Break after decision or max iterations
                }

                // Now that we've got our blob, figure out whether it is a merge
                if (m_lastPrimaryBlob.isValid) {
                    Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, candidateBlobAfterExpansion, frame.size(), originalFrameNumber);
                    bestBlobForThisFrame = candidateBlobAfterExpansion;
                    bestBlobForThisFrame.centroid = persisted.isValid ? persisted.centroid : candidateBlobAfterExpansion.centroid;
                } else {
                    bestBlobForThisFrame = candidateBlobAfterExpansion;
                }
                bool confirmedMerge = false;

                qDebug().noquote()<< dmsg << "Checking if merged..." << m_lastPrimaryBlob.isValid << bestBlobForThisFrame.isValid << candidateBlobAfterExpansion.area << m_lastPrimaryBlob.area;
                if (m_lastPrimaryBlob.isValid && bestBlobForThisFrame.isValid && candidateBlobAfterExpansion.area > m_lastPrimaryBlob.area * MERGE_CONFIRM_RELATIVE_AREA_FACTOR) {
                    confirmedMerge = true;
                } else if (bestBlobForThisFrame.isValid && candidateBlobAfterExpansion.area > m_maxBlobArea * MERGE_CONFIRM_ABSOLUTE_AREA_FACTOR) {
                    confirmedMerge = true;
                }

                if (confirmedMerge) {
                    nextState = Tracking::TrackerState::TrackingMerged;
                } else {
                    nextState = Tracking::TrackerState::TrackingSingle; // This is single processing, so it should never be a split here.
                }
            }
        } else { // worms were merged (m_currentState == TrackerState::TrackingMerged), this might be a split.
            qDebug().noquote() << dmsg << "Previously merged, now >1 blobs. Potential split. Last primary blob valid: " << m_lastPrimaryBlob.isValid;

            Tracking::DetectedBlob closestBlob; // Will hold the chosen blob
            closestBlob.isValid = false; // Initialize as invalid

            if (!m_lastPrimaryBlob.isValid) {
                qDebug().noquote() << dmsg << "Warning: In split logic but m_lastPrimaryBlob is invalid. Falling back to largest current blob.";
                // Fallback: consider the largest of the current blobs as the most likely candidate if no history.
                if (!blobsInFixedRoi.isEmpty()) {
                    closestBlob = blobsInFixedRoi.first();
                }
                // If blobsInFixedRoi is also empty, closestBlob remains invalid, handled later.
            } else {
                // Find the blob in blobsInFixedRoi closest to m_lastPrimaryBlob.centroid
                double mindist = std::numeric_limits<double>::max(); // Initialize with a very large distance
                closestBlob = blobsInFixedRoi.first(); // Default to largest if none are "close" or all invalid

                bool foundAValidBlobToCompare = false;
                for (const Tracking::DetectedBlob &blob : blobsInFixedRoi) {
                    if (blob.isValid) {
                        foundAValidBlobToCompare = true;
                        double dist = Tracking::sqDistance(m_lastPrimaryBlob.centroid, blob.centroid);
                        if (dist < mindist) {
                            mindist = dist;
                            closestBlob = blob;
                        }
                    }
                }
                if (!foundAValidBlobToCompare && !blobsInFixedRoi.isEmpty()) {
                    qDebug().noquote() << dmsg << "No valid blobs in blobsInFixedRoi to compare for split, but list not empty. Defaulting to first.";
                    closestBlob = blobsInFixedRoi.first(); // ensure closestBlob is assigned if all were invalid
                } else if (!foundAValidBlobToCompare && blobsInFixedRoi.isEmpty()){
                    qDebug().noquote() << dmsg << "No valid blobs in blobsInFixedRoi and list is empty.";
                    // closestBlob remains invalid
                }
            }

            // If no valid closestBlob could be determined (e.g. m_lastPrimaryBlob invalid AND blobsInFixedRoi empty)
            if (!closestBlob.isValid) {
                qDebug().noquote() << dmsg << "Could not determine a valid closest blob for split. Setting to TrackingLost.";
                // This will lead to the final else block setting TrackingLost
            } else if (closestBlob.touchesROIboundary) {
                qDebug().noquote()<< dmsg << "Closest blob for split touches boundary. Expansion for:" << closestBlob.boundingBox;
                QRectF analysisRoi = searchRoiUsedForThisFrame;
                Tracking::DetectedBlob candidateBlobAfterExpansion = closestBlob; // Initialize with the chosen closest blob

                for (int i = 0; i < MAX_EXPANSION_ITERATIONS_BOUNDARY; ++i) {
                    qDebug().noquote()<< dmsg << "Split Expansion iter #" << i + 1 << " Current analysis ROI:" << analysisRoi;
                    qreal newWidth = analysisRoi.width() * BOUNDARY_ROI_EXPANSION_FACTOR;
                    qreal newHeight = analysisRoi.height() * BOUNDARY_ROI_EXPANSION_FACTOR;
                    QPointF center = candidateBlobAfterExpansion.isValid ? candidateBlobAfterExpansion.centroid : analysisRoi.center();
                    analysisRoi.setSize(QSizeF(newWidth, newHeight));
                    analysisRoi.moveCenter(center);
                    // Clamp ROI
                    analysisRoi.setX(qMax(0.0, analysisRoi.x()));
                    analysisRoi.setY(qMax(0.0, analysisRoi.y()));
                    if (analysisRoi.right() > frame.cols) analysisRoi.setRight(frame.cols);
                    if (analysisRoi.bottom() > frame.rows) analysisRoi.setBottom(frame.rows);
                    analysisRoi.setWidth(qMin(analysisRoi.width(), static_cast<qreal>(frame.cols)));
                    analysisRoi.setHeight(qMin(analysisRoi.height(), static_cast<qreal>(frame.rows)));

                    QList<Tracking::DetectedBlob> blobsInExpanded = findPlausibleBlobsInRoi(frame, analysisRoi);
                    qDebug().noquote()<< dmsg << "Found" << blobsInExpanded.count() << "blobs in expanded ROI for split:" << analysisRoi;

                    if (blobsInExpanded.isEmpty()) {
                        qDebug().noquote()<< dmsg << "No blobs in expanded ROI during split expansion. Aborting expansion.";
                        // candidateBlobAfterExpansion remains what it was
                        break;
                    }

                    // Refind closest blob within this new expanded ROI
                    Tracking::DetectedBlob bestCandidateInThisExpansionIteration;
                    bestCandidateInThisExpansionIteration.isValid = false;
                    double localMinDistForThisIteration = std::numeric_limits<double>::max();

                    if(m_lastPrimaryBlob.isValid) { // Only search by distance if we have a valid reference
                        for (const Tracking::DetectedBlob &blobInExpanded : blobsInExpanded) {
                            if (blobInExpanded.isValid) {
                                if (!bestCandidateInThisExpansionIteration.isValid) { // First valid blob becomes current best
                                    bestCandidateInThisExpansionIteration = blobInExpanded;
                                    localMinDistForThisIteration = Tracking::sqDistance(m_lastPrimaryBlob.centroid, blobInExpanded.centroid);
                                } else {
                                    double dist = Tracking::sqDistance(m_lastPrimaryBlob.centroid, blobInExpanded.centroid);
                                    if (dist < localMinDistForThisIteration) {
                                        localMinDistForThisIteration = dist;
                                        bestCandidateInThisExpansionIteration = blobInExpanded;
                                    }
                                }
                            }
                        }
                    } else { // Fallback if m_lastPrimaryBlob became invalid mid-process (should be rare)
                        if(!blobsInExpanded.isEmpty()) bestCandidateInThisExpansionIteration = blobsInExpanded.first();
                    }

                    if (bestCandidateInThisExpansionIteration.isValid) {
                        candidateBlobAfterExpansion = bestCandidateInThisExpansionIteration;
                        qDebug().noquote()<< dmsg << "Split expansion, new candidate: BBox:" << candidateBlobAfterExpansion.boundingBox;
                    } else {
                        qDebug().noquote()<< dmsg << "Split expansion, no valid candidate found in blobsInExpanded. Using previous candidate.";
                        // candidateBlobAfterExpansion remains what it was, loop will likely break or continue based on containment
                    }


                    if (i == MAX_EXPANSION_ITERATIONS_BOUNDARY - 1 || analysisRoi.contains(candidateBlobAfterExpansion.boundingBox)) {
                        if (!analysisRoi.contains(candidateBlobAfterExpansion.boundingBox))
                            qDebug().noquote()<< dmsg << "Max expansion iterations reached for split. Using current candidate.";
                        else
                            qDebug().noquote()<< dmsg << "Candidate blob for split contained in expanded ROI. Stopping expansion.";
                        break;
                    } else {
                        continue;
                    }
                }
                bestBlobForThisFrame = candidateBlobAfterExpansion; // Assign the result of expansion (or original closest if expansion failed)
            } else { // closestBlob for split does not touch boundary
                bestBlobForThisFrame = closestBlob;
                qDebug().noquote()<< dmsg << "Closest blob for split is fully contained, no expansion needed. BBox:" << bestBlobForThisFrame.boundingBox;
            }

            // If after all this, bestBlobForThisFrame is still not valid (e.g., m_lastPrimaryBlob invalid, blobsInFixedRoi empty, and closestBlob never became valid)
            // it will be caught by the final check. Otherwise, we have a candidate.
            if (bestBlobForThisFrame.isValid) {
                nextState = Tracking::TrackerState::PausedForSplit;
            } else {
                qDebug().noquote() << dmsg << "After split logic, bestBlobForThisFrame is still invalid.";
                // This will lead to TrackingLost in the final section
            }
        }
    }

    if (bestBlobForThisFrame.isValid) {
        if (m_currentState != nextState) {
            // Special handling: if we are PausedForSplit, and the blob area is very small,
            // it could be noise. Consider if TrackerState::TrackingLost is more appropriate.
            // For now, we trust PausedForSplit to be handled by a higher-level logic or subsequent frames.
            m_currentState = nextState;
            emit stateChanged(m_wormId, m_currentState);
            qDebug().noquote()<< dmsg <<"State changed to:" << m_currentState;
        }

        m_lastKnownPosition = cv::Point2f(static_cast<float>(bestBlobForThisFrame.centroid.x()), static_cast<float>(bestBlobForThisFrame.centroid.y()));
        QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, frame.size());
        m_lastPrimaryBlob = bestBlobForThisFrame; // Update for next frame's persistence check

        emit positionUpdated(m_wormId, originalFrameNumber, bestBlobForThisFrame, searchRoiUsedForThisFrame, m_currentState, plausibleBlobsInFixedRoi);
        qDebug().noquote()<< dmsg <<"Position updated. Blob BBox:" << bestBlobForThisFrame.boundingBox
                           << "Search ROI used:" << searchRoiUsedForThisFrame << "Next Search ROI:" << nextFrameSearchRoi;

        currentFixedSearchRoiRef_InOut = nextFrameSearchRoi; // Update ROI for the next frame
        return true;
    } else {
        qDebug().noquote()<< dmsg <<"No valid bestBlobForThisFrame. Plausible blobs: " << plausibleBlobsInFixedRoi << ". Setting state to TrackingLost.";
        m_lastPrimaryBlob.isValid = false;
        Tracking::DetectedBlob invalidBlob;
        // Ensure state is updated to TrackingLost if it wasn't already
        if (m_currentState != Tracking::TrackerState::TrackingLost) {
            m_currentState = Tracking::TrackerState::TrackingLost;
            emit stateChanged(m_wormId, m_currentState);
        }
        emit positionUpdated(m_wormId, originalFrameNumber, invalidBlob, searchRoiUsedForThisFrame, Tracking::TrackerState::TrackingLost, plausibleBlobsInFixedRoi);
        // currentFixedSearchRoiRef_InOut remains unchanged
        return false;
    }
}

void WormTracker::resumeTrackingWithAssignedTarget(const Tracking::DetectedBlob& targetBlob) {
    // No 'targetId' parameter, no 'if (m_wormId == targetId)' check needed.
    // If this slot is called, it's meant for this instance.

    if (m_currentState != Tracking::TrackerState::PausedForSplit) {
        qWarning() << "WormTracker ID" << m_wormId << getDirection() // Helper to get "(Fwd)" or "(Bwd)"
                   << ": resumeTrackingWithAssignedTarget called but not in PausedForSplit state. Current state:"
                   << static_cast<int>(m_currentState) << ". Ignoring.";
        return;
    }

    qDebug().noquote() << "WormTracker ID" << m_wormId << getDirection()
                       << ": resumeTrackingWithAssignedTarget received. Blob Centroid:"
                       << targetBlob.centroid.x() << "," << targetBlob.centroid.y() << "Area:" << targetBlob.area;

    if (!targetBlob.isValid) {
        qWarning() << "WormTracker ID" << m_wormId << getDirection()
        << ": resumeTrackingWithAssignedTarget called with invalid blob. Forcing stop.";
        // Transition to a lost/idle state and stop the tracking loop
        m_currentState = Tracking::TrackerState::TrackingLost; // Or Idle
        m_trackingActive = false; // This will cause continueTracking to stop
        emit stateChanged(m_wormId, m_currentState); // Notify TM of state change
        emit finished(); // Signal that this tracker's work is done
        return;
    }

    // Your existing logic for resuming:
    m_lastKnownPosition = cv::Point2f(static_cast<float>(targetBlob.centroid.x()), static_cast<float>(targetBlob.centroid.y()));

    cv::Size currentFrameSize;
    // Robustly get frame size (your existing logic is good here)
    if (m_framesToProcess && m_currFrameNum < static_cast<int>(m_framesToProcess->size()) && m_currFrameNum >= 0) {
        currentFrameSize = m_framesToProcess->at(m_currFrameNum).size();
    } else if (m_framesToProcess && !m_framesToProcess->empty()) {
        currentFrameSize = m_framesToProcess->at(0).size();
        qWarning() << "WormTracker ID" << m_wormId << getDirection() << ": m_currFrameNum invalid in resume. Using first frame size for ROI.";
    } else {
        qWarning() << "WormTracker ID" << m_wormId << getDirection() << ": No frames available in resume. Cannot set ROI size.";
        currentFrameSize = cv::Size(640, 480); // Fallback
    }

    m_currentSearchRoi = adjustRoiPos(m_lastKnownPosition, currentFrameSize); // adjustRoiPos needs to be defined/accessible
    m_lastPrimaryBlob = targetBlob;
    m_currentState = Tracking::TrackerState::TrackingSingle;
    // emit stateChanged(m_wormId, m_currentState); // TM will know from subsequent positionUpdated

    qDebug().noquote() << "WormTracker ID" << m_wormId << getDirection()
                       << ": Resumed. New Search ROI:" << m_currentSearchRoi
                       << "State:" << static_cast<int>(m_currentState);

    // The tracker was paused at m_currFrameNum. It has now been assigned a target *for that frame*.
    // It should process the *next* frame with this new information.
    m_currFrameNum++;

    if (m_trackingActive) { // Check if not already stopped by invalid blob logic
        QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
    }
}

/*void WormTracker::resumeTrackingWithNewTarget(int targetId, const Tracking::DetectedBlob& targetBlob) {
    if (m_currentState == Tracking::TrackerState::PausedForSplit && m_wormId == targetId) {
        qDebug().noquote()<< "WormTracker ID" << m_wormId << ": resumeTrackingWithNewTarget called. Blob Centroid:" << targetBlob.centroid << "Area:" << targetBlob.area;
        if (!targetBlob.isValid) {
            qWarning() << "WormTracker ID" << m_wormId << ": resumeTrackingWithNewTarget called with invalid blob. Stopping tracking.";
            m_currentState = Tracking::TrackerState::Idle; // Or a 'Lost' state
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
        m_currentState = Tracking::TrackerState::TrackingSingle; // Resume as tracking a single, confirmed target
        //emit stateChanged(m_wormId, m_currentState);

        qDebug().noquote()<< "WormTracker ID" << m_wormId << ": Resumed. New Search ROI:" << m_currentSearchRoi << "State:" << static_cast<int>(m_currentState);

        // Important: The tracker was paused at m_currFrameNum because a split was detected *in that frame*.
        // Now that it's resuming, it should process the *next* frame.
        m_currFrameNum++;

        // Re-activate the tracking loop
        if (m_trackingActive) { // Only if stopTracking wasn't called due to invalid blob
            QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
        }
    }
}*/

void WormTracker::confirmTargetIsMerged(int mergedEntityID, const QPointF& mergedBlobCentroid, const QRectF& mergedBlobRoiFromManager) {
    // mergedEntityID is the representative ID of the merge group from TrackingManager.
    // mergedBlobCentroid is TrackingManager's best estimate of the new merged entity's center.
    // mergedBlobRoiFromManager is TrackingManager's calculated ROI that encompasses the whole merged entity.

    qDebug().noquote()<< "WormTracker ID" << m_wormId << "(Dir:" << m_direction << "): confirmTargetIsMerged by TrackingManager."
             << "Associated MergeID:" << mergedEntityID
             << "Merged Centroid (approx from TM):" << mergedBlobCentroid
             << "Merged ROI (from TM):" << mergedBlobRoiFromManager;

    // Update tracker state
    if (m_currentState != Tracking::TrackerState::TrackingMerged) {
        m_currentState = Tracking::TrackerState::TrackingMerged;
        emit stateChanged(m_wormId, m_currentState, mergedEntityID); // Pass manager's representative ID
    }

    // Update last known position to the centroid of the merged entity
    m_lastKnownPosition = cv::Point2f(static_cast<float>(mergedBlobCentroid.x()), static_cast<float>(mergedBlobCentroid.y()));

    // Set the m_currentSearchRoi for the *next* frame to be this tracker's standard
    // fixed-size ROI, centered on the new mergedBlobCentroid.
    cv::Size currentFrameCvSize;
    if (m_framesToProcess && m_currFrameNum >= 0 && m_currFrameNum < static_cast<int>(m_framesToProcess->size())) {
        if (!m_framesToProcess->at(m_currFrameNum).empty()) {
            currentFrameCvSize = m_framesToProcess->at(m_currFrameNum).size();
        } else {
            qWarning() << "WormTracker ID" << m_wormId << ": Current frame is empty in confirmTargetIsMerged. Cannot get valid frame size.";
            // Attempt to use a previous valid frame size or a default if absolutely necessary
            if (m_framesToProcess && !m_framesToProcess->empty() && !m_framesToProcess->at(0).empty()) {
                currentFrameCvSize = m_framesToProcess->at(0).size();
                qWarning() << "WormTracker ID" << m_wormId << ": Using first frame's size as fallback.";
            } else {
                currentFrameCvSize = cv::Size(640, 480); // Last resort default
                qWarning() << "WormTracker ID" << m_wormId << ": Using arbitrary default frame size (640x480) for ROI adjustment.";
            }
        }
    } else {
        qWarning() << "WormTracker ID" << m_wormId << ": Frame data or m_currFrameNum is invalid in confirmTargetIsMerged. Cannot accurately set ROI size.";
        // Fallback to a default size or handle error more gracefully
        if (m_framesToProcess && !m_framesToProcess->empty() && !m_framesToProcess->at(0).empty()) {
            currentFrameCvSize = m_framesToProcess->at(0).size(); // Fallback to first frame size
        } else {
            currentFrameCvSize = cv::Size(640,480); // Arbitrary fallback
            qWarning() << "WormTracker ID" << m_wormId << ": Using arbitrary default frame size (640x480) for ROI adjustment.";
        }
    }

    m_currentSearchRoi = adjustRoiPos(m_lastKnownPosition, currentFrameCvSize);
    qDebug().noquote()<< "  WormTracker ID" << m_wormId << ": Next search ROI (fixed-size, recentered on merged centroid):" << m_currentSearchRoi;

    // Update m_lastPrimaryBlob to represent this tracker's new understanding of its target.
    // Since it's merged, the "primary blob" for this tracker is now effectively the merged entity,
    // or its best guess of its component within it.
    // For m_lastPrimaryBlob, we use the manager-provided centroid and the manager's encompassing ROI
    // as the best available description of the *entire* merged entity.
    // The tracker's own findPersistingComponent will try to pick out its part in the next frame.
    m_lastPrimaryBlob.isValid = true;
    m_lastPrimaryBlob.centroid = mergedBlobCentroid; // Centroid of the whole merged mass (from TM)
    if (mergedBlobRoiFromManager.isValid()) {
        m_lastPrimaryBlob.boundingBox = mergedBlobRoiFromManager; // BBox of the whole merged mass (from TM)
        m_lastPrimaryBlob.area = mergedBlobRoiFromManager.width() * mergedBlobRoiFromManager.height(); // Approx area of whole merge
    } else {
        // If TM didn't provide a valid ROI, use our (less ideal) fixed-size search ROI as a proxy for the bbox
        m_lastPrimaryBlob.boundingBox = m_currentSearchRoi;
        m_lastPrimaryBlob.area = m_currentSearchRoi.width() * m_currentSearchRoi.height();
        qWarning() << "  WormTracker ID" << m_wormId << ": TrackingManager provided invalid mergedBlobRoi. Using self-generated search ROI for m_lastPrimaryBlob.bbox.";
    }
    m_lastPrimaryBlob.contourPoints.clear(); // The exact contour of our part within the merge is unknown at this point.

    // The tracking loop (continueTracking) will proceed with the current m_currFrameNum,
    // but now in the TrackingMerged state and using the updated m_currentSearchRoi.
    // No need to explicitly call QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
    // if this slot is called from the TrackingManager (different thread) and the tracker's event loop is running.
    // However, if there's a chance this is called from the tracker's own thread synchronously in a way that
    // would prevent the event loop from continuing, a queued call might be safer.
    // Given it's a slot called by TrackingManager (likely from TM's thread or main thread),
    // the tracker's own event loop should pick up `continueTracking` if it was already queued or if
    // this slot call doesn't block its next scheduled invocation.
    // For safety and to ensure the loop continues if it was waiting for this state change:
    if (m_trackingActive && m_currentState != Tracking::TrackerState::PausedForSplit) { // Ensure not paused for other reasons
        // QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
        // Usually, the existing continueTracking loop mechanism will handle this state transition.
        // If this slot is called, the tracker is not in its own processing loop for this frame yet.
    }
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

            // qDebug().noquote()<< "WormTracker ID" << m_wormId << ":" << debugContextName
            //          << "component found. Centroid:" << resultBlob.centroid << "Area:" << resultBlob.area;
        }
    } else {
        // qDebug().noquote()<< "WormTracker ID" << m_wormId << ":" << debugContextName
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

