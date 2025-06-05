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
    m_skipMergeDetectionNextFrame(false),
    m_currentState(Tracking::TrackerState::Idle)
{
    qDebug().noquote()<< "WormTracker (" << this << ") created for worm ID:" << m_wormId
                       << "Direction:" << (direction == TrackingDirection::Forward ? "Forward" : "Backward")
                       << "Initial Fixed ROI:" << initialRoi << "Edge:" << m_initialRoiEdge;
    m_lastPrimaryBlob.isValid = false; // Ensure lastPrimaryBlob starts as invalid
    m_lastFullBlob.isValid = false;    // Initialize new member
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
    m_lastFullBlob.isValid = false;    // Reset for a new tracking session
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

            if (m_currentState == Tracking::TrackerState::TrackingSingle)
            {
                foundTargetThisFrame = processFrameAsSingle(currentFrame, m_currFrameNum, m_currentSearchRoi);
            }
            else if (m_currentState == Tracking::TrackerState::TrackingMerged)
            {
                foundTargetThisFrame = processFrameAsMerged(currentFrame, m_currFrameNum, m_currentSearchRoi);
            }

            if (!foundTargetThisFrame && m_currentState != Tracking::TrackerState::PausedForSplit) {
                // qDebug().noquote()<< "WormTracker ID" << m_wormId << ": Target search at sequence index" << m_currFrameNum << "was skipped or unsuccessful; ROI for next frame remains" << m_currentSearchRoi;
            }
        }

        if (m_currentState != Tracking::TrackerState::PausedForSplit)
        {
            if (m_currFrameNum % 10 == 0 || m_currFrameNum == static_cast<int>(m_framesToProcess->size()) - 1) {
                emit progress(m_wormId, static_cast<int>((static_cast<double>(m_currFrameNum + 1) / m_framesToProcess->size()) * 100.0));
            }
            m_currFrameNum++;
        }

        if (m_trackingActive && (m_currentState != Tracking::TrackerState::PausedForSplit || m_currFrameNum >= static_cast<int>(m_framesToProcess->size())) ) {
            QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
        } else if (!m_trackingActive) {
            qDebug().noquote()<< "WormTracker ID" << m_wormId << ": Tracking no longer active. Emitting finished.";
            emit progress(m_wormId, 100);
            emit finished();
        }

    } else {
        if (m_trackingActive) {
            qDebug().noquote()<< "WormTracker ID" << m_wormId << ": Frame processing loop completed naturally.";
            emit progress(m_wormId, 100);
            m_trackingActive = false;
        }
        qDebug().noquote()<< "WormTracker ID" << m_wormId << ": Finished processing. Final state:" << static_cast<int>(m_currentState);
        emit finished();
    }
}

void WormTracker::stopTracking() {
    qDebug().noquote()<< "WormTracker ID" << m_wormId << ": stopTracking() called. Current state:" << static_cast<int>(m_currentState);
    m_trackingActive = false;
    QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
}

Tracking::DetectedBlob WormTracker::findPersistingComponent(
    const Tracking::DetectedBlob& previousFrameAnchorBlob, // Renamed for clarity
    const Tracking::DetectedBlob& currentFrameFullBlob,    // Renamed for clarity
    const cv::Size& frameSize,
    int originalFrameNumberForDebug)
{
    Tracking::DetectedBlob persistingComponent;
    persistingComponent.isValid = false;

    if (!previousFrameAnchorBlob.isValid || previousFrameAnchorBlob.contourPoints.empty() ||
        !currentFrameFullBlob.isValid || currentFrameFullBlob.contourPoints.empty()) {
        // if (originalFrameNumberForDebug != -1) {
        //     qDebug().noquote()<< "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
        //              << ": findPersistingComponent called with invalid input blobs.";
        // }
        return persistingComponent;
    }

    cv::Mat prevBlobMask = cv::Mat::zeros(frameSize, CV_8UC1);
    cv::Mat currBlobMask = cv::Mat::zeros(frameSize, CV_8UC1);

    std::vector<std::vector<cv::Point>> prevContoursVec = {previousFrameAnchorBlob.contourPoints};
    cv::drawContours(prevBlobMask, prevContoursVec, 0, cv::Scalar(255), cv::FILLED);

    std::vector<std::vector<cv::Point>> currContoursVec = {currentFrameFullBlob.contourPoints};
    cv::drawContours(currBlobMask, currContoursVec, 0, cv::Scalar(255), cv::FILLED);

    cv::Mat newGrowthMask, oldOverlapMask, prevBlobMaskNot;
    cv::bitwise_not(prevBlobMask, prevBlobMaskNot);
    cv::bitwise_and(currBlobMask, prevBlobMaskNot, newGrowthMask); // Part of current not in previous
    cv::bitwise_and(currBlobMask, prevBlobMask, oldOverlapMask);   // Part of current that IS in previous

    Tracking::DetectedBlob oldOverlapComponentDetails = findLargestBlobComponentInMask(oldOverlapMask, "OldOverlapAnalysis");
    Tracking::DetectedBlob newGrowthComponentDetails = findLargestBlobComponentInMask(newGrowthMask, "NewGrowthAnalysis");

    const double significanceThresholdFactor = 0.3; // Relative to min single worm area

    // Prioritize the overlapping part if it's significant
    if (oldOverlapComponentDetails.isValid && oldOverlapComponentDetails.area > (m_minBlobArea * significanceThresholdFactor)) {
        // if (originalFrameNumberForDebug != -1)
        //     qDebug().noquote()<< "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
        //              << ": Persisting component identified as 'old overlap'. Centroid:" << oldOverlapComponentDetails.centroid
        //              << "Area:" << oldOverlapComponentDetails.area;
        persistingComponent = oldOverlapComponentDetails;
    }
    // If old overlap is not good, check if the new growth part is significant
    // This could happen if the worm moved quickly, and the "new growth" is actually the worm in a new spot.
    else if (newGrowthComponentDetails.isValid && newGrowthComponentDetails.area > (m_minBlobArea * significanceThresholdFactor)) {
        // if (originalFrameNumberForDebug != -1)
        //     qDebug().noquote()<< "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
        //              << ": Persisting component identified as 'new growth' (old overlap was small/invalid). Centroid:" << newGrowthComponentDetails.centroid
        //              << "Area:" << newGrowthComponentDetails.area;
        persistingComponent = newGrowthComponentDetails;
    } else {
        // If neither is conclusive, it's hard to define a "persisting" part.
        // Fallback: if the currentFullBlob itself is plausibly a single worm, use that.
        // Otherwise, this function returns an invalid blob, and the caller might use the full current blob.
        if (currentFrameFullBlob.area >= m_minBlobArea && currentFrameFullBlob.area <= m_maxBlobArea * 1.2) { // Allow slightly larger for persistence
            // if (originalFrameNumberForDebug != -1)
            //     qDebug().noquote()<< "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
            //              << ": Persisting component analysis inconclusive. Using full current blob as it's plausible single.";
            persistingComponent = currentFrameFullBlob;
        } else {
            // if (originalFrameNumberForDebug != -1)
            //     qDebug().noquote()<< "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
            //              << ": Persisting component analysis inconclusive, components too small or currentFull too large. Returning invalid.";
            // persistingComponent remains invalid
        }
    }
    return persistingComponent;
}


WormTracker::FrameProcessingContext WormTracker::initializeFrameProcessing(const cv::Mat& frame, int sequenceFrameIndex, const QRectF& searchRoi)
{
    FrameProcessingContext context;

    int debugId = m_wormId;
    if (m_direction == TrackingDirection::Forward) {
        context.originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else {
        context.originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex;
        debugId = -1 * m_wormId;
    }

    context.debugMessage = QString("WT ") + QString::number(debugId) + QString(" FN") + QString::number(context.originalFrameNumber) + QString(": ");
    context.searchRoiUsedForThisFrame = searchRoi;
    context.blobsInFixedRoi = findPlausibleBlobsInRoi(frame, searchRoi);
    context.plausibleBlobsInFixedRoi = context.blobsInFixedRoi.count();

    qDebug().noquote() << context.debugMessage << "State:" << m_currentState << " Search begins with ROI " << context.searchRoiUsedForThisFrame << "LastPrimaryValid:" << m_lastPrimaryBlob.isValid;

    return context;
}

bool WormTracker::processFrameAsSingle(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentFixedSearchRoiRef_InOut)
{
    FrameProcessingContext context = initializeFrameProcessing(frame, sequenceFrameIndex, currentFixedSearchRoiRef_InOut);

    Tracking::DetectedBlob blobForAnchor;     // This will become m_lastPrimaryBlob (tracking anchor)
    Tracking::DetectedBlob blobToReport;      // This will become m_lastFullBlob (what's actually on screen)
    QList<Tracking::DetectedBlob> splitCandidates; // All candidates when transitioning to PausedForSplit
    blobForAnchor.isValid = false;
    blobToReport.isValid = false;
    Tracking::TrackerState nextState = m_currentState;

    if (context.plausibleBlobsInFixedRoi == 0) {
        qDebug().noquote() << context.debugMessage << "No plausible blobs found in initial ROI " << context.searchRoiUsedForThisFrame;
        nextState = Tracking::TrackerState::TrackingLost;
        // blobForAnchor and blobToReport remain invalid
    } else if (context.plausibleBlobsInFixedRoi == 1) {
        Tracking::DetectedBlob singleBlob = context.blobsInFixedRoi.first();
        qDebug().noquote() << context.debugMessage << "Found 1 plausible blob. BBox:" << singleBlob.boundingBox << "Area:" << singleBlob.area;

        bool touchesBoundary = singleBlob.touchesROIboundary ||
                               !context.searchRoiUsedForThisFrame.contains(singleBlob.boundingBox);

        if (!touchesBoundary) {
            qDebug().noquote() << context.debugMessage << "Single blob fully contained.";

            // Check if area has reduced significantly, indicating split without boundary contact
            bool possibleSplitByAreaReduction = false;
            if (m_lastFullBlob.isValid && singleBlob.area < m_lastFullBlob.area * 0.75) {
                possibleSplitByAreaReduction = true;
                qDebug().noquote() << context.debugMessage << "Area significantly reduced from" << m_lastFullBlob.area << "to" << singleBlob.area << ". Possible split without boundary contact.";
            }

            if (possibleSplitByAreaReduction) {
                // Treat as split - provide current blob as split candidate
                blobToReport = singleBlob;
                blobForAnchor = singleBlob;
                splitCandidates.append(singleBlob); // Single candidate found in ROI
                nextState = Tracking::TrackerState::PausedForSplit;
            } else {
                blobToReport = singleBlob;
                blobForAnchor = singleBlob; // For single, anchor and report are the same
            }
        } else { // Touches boundary - perform expansion
            qDebug().noquote() << context.debugMessage << "Single blob touches ROI boundary. Initiating expansion analysis.";
            QRectF analysisRoi = context.searchRoiUsedForThisFrame;
            Tracking::DetectedBlob candidateBlobAfterExpansion = singleBlob;

            for (int i = 0; i < MAX_EXPANSION_ITERATIONS_BOUNDARY; ++i) {
                qreal newWidth = analysisRoi.width() * BOUNDARY_ROI_EXPANSION_FACTOR;
                qreal newHeight = analysisRoi.height() * BOUNDARY_ROI_EXPANSION_FACTOR;
                QPointF center = candidateBlobAfterExpansion.isValid ? candidateBlobAfterExpansion.centroid : analysisRoi.center();
                analysisRoi.setSize(QSizeF(newWidth, newHeight));
                analysisRoi.moveCenter(center);
                analysisRoi.setX(qMax(0.0, analysisRoi.x())); analysisRoi.setY(qMax(0.0, analysisRoi.y()));
                if (analysisRoi.right() > frame.cols) analysisRoi.setRight(frame.cols);
                if (analysisRoi.bottom() > frame.rows) analysisRoi.setBottom(frame.rows);
                analysisRoi.setWidth(qMin(analysisRoi.width(), static_cast<qreal>(frame.cols)));
                analysisRoi.setHeight(qMin(analysisRoi.height(), static_cast<qreal>(frame.rows)));

                QList<Tracking::DetectedBlob> blobsInExpanded = findPlausibleBlobsInRoi(frame, analysisRoi);
                if (blobsInExpanded.isEmpty()) break;

                candidateBlobAfterExpansion = blobsInExpanded.first(); // Take largest in expanded
                if (analysisRoi.contains(candidateBlobAfterExpansion.boundingBox) || i == MAX_EXPANSION_ITERATIONS_BOUNDARY - 1) break;
            }
            // After expansion, candidateBlobAfterExpansion is our best guess for the full entity
            blobToReport = candidateBlobAfterExpansion;

            // Determine anchor: if previously tracked, find persisting part. Otherwise, anchor is the full blob.
            if (m_lastPrimaryBlob.isValid) {
                Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, blobToReport, frame.size(), context.originalFrameNumber);
                blobForAnchor = persisted.isValid ? persisted : blobToReport; // Use persisted if found, else full
            } else {
                blobForAnchor = blobToReport;
            }

            // Check for merge based on blobToReport's area (skip if resuming from split)
            bool confirmedMerge = false;
            if (m_skipMergeDetectionNextFrame) {
                qDebug().noquote() << context.debugMessage << "Skipping merge detection (resuming from split).";
                confirmedMerge = false;
            } else if (m_lastPrimaryBlob.isValid && blobToReport.isValid && blobToReport.area > m_lastPrimaryBlob.area * MERGE_CONFIRM_RELATIVE_AREA_FACTOR) {
                confirmedMerge = true;
            } else if (blobToReport.isValid && blobToReport.area > m_maxBlobArea * MERGE_CONFIRM_ABSOLUTE_AREA_FACTOR) {
                confirmedMerge = true;
            }

            if (confirmedMerge) {
                qDebug().noquote() << context.debugMessage << "Boundary touch + expansion CONFIRMS MERGE. ReportedArea:" << blobToReport.area << "AnchorArea:" << blobForAnchor.area;
                nextState = Tracking::TrackerState::TrackingMerged;
            } else {
                qDebug().noquote() << context.debugMessage << "Boundary touch + expansion, NO merge by area. ReportedArea:" << blobToReport.area;
                // If it wasn't a merge, it's a single worm. Anchor and report should ideally be the same.
                blobForAnchor = blobToReport; // Ensure anchor is the full single blob if not a merge
                nextState = Tracking::TrackerState::TrackingSingle;
            }
        }
    } else { // plausibleBlobsInFixedRoi > 1 - Potential merge (was single, now multiple blobs in ROI)
        Tracking::DetectedBlob searchCandidate; // This will be the blob we evaluate further
        searchCandidate.isValid = false;

        if (!context.blobsInFixedRoi.isEmpty()) { // Should always be true due to 'plausibleBlobsInFixedRoi > 1'
            if (m_lastPrimaryBlob.isValid) {
                double minSqDist = std::numeric_limits<double>::max();
                Tracking::DetectedBlob closestBlob;
                closestBlob.isValid = false; // Initialize to invalid

                for (const Tracking::DetectedBlob& currentBlob : context.blobsInFixedRoi) {
                    if (currentBlob.isValid) { // Ensure blob itself is valid
                        double sqDist = Tracking::sqDistance(m_lastPrimaryBlob.centroid, currentBlob.centroid);
                        if (sqDist < minSqDist) {
                            minSqDist = sqDist;
                            closestBlob = currentBlob;
                        }
                    }
                }
                if (closestBlob.isValid) {
                    searchCandidate = closestBlob;
                } else {
                    // Fallback if no valid blobs were found to compare distance (should be rare if plausibleBlobsInFixedRoi > 0)
                    searchCandidate = context.blobsInFixedRoi.first(); // Fallback to largest
                }
            } else {
                // If m_lastPrimaryBlob is not valid (e.g., first few frames or lost track previously),
                // then falling back to the largest blob is a reasonable strategy.
                searchCandidate = context.blobsInFixedRoi.first();
            }
        }

        bool largestTouchesBoundary = searchCandidate.touchesROIboundary ||
                                      !context.searchRoiUsedForThisFrame.contains(searchCandidate.boundingBox);

        if (!largestTouchesBoundary) {
            blobToReport = searchCandidate;
            blobForAnchor = searchCandidate;
            nextState = Tracking::TrackerState::TrackingSingle;
        } else { // Largest touches boundary, expand to see if it's a merge
            QRectF analysisRoi = context.searchRoiUsedForThisFrame;
            Tracking::DetectedBlob candidateBlobAfterExpansion = searchCandidate;

            for (int i = 0; i < MAX_EXPANSION_ITERATIONS_BOUNDARY; ++i) {
                qreal newWidth = analysisRoi.width() * BOUNDARY_ROI_EXPANSION_FACTOR;
                qreal newHeight = analysisRoi.height() * BOUNDARY_ROI_EXPANSION_FACTOR;
                QPointF center = candidateBlobAfterExpansion.isValid ? candidateBlobAfterExpansion.centroid : analysisRoi.center();
                analysisRoi.setSize(QSizeF(newWidth, newHeight));
                analysisRoi.moveCenter(center);
                analysisRoi.setX(qMax(0.0, analysisRoi.x())); analysisRoi.setY(qMax(0.0, analysisRoi.y()));
                if (analysisRoi.right() > frame.cols) analysisRoi.setRight(frame.cols);
                if (analysisRoi.bottom() > frame.rows) analysisRoi.setBottom(frame.rows);
                analysisRoi.setWidth(qMin(analysisRoi.width(), static_cast<qreal>(frame.cols)));
                analysisRoi.setHeight(qMin(analysisRoi.height(), static_cast<qreal>(frame.rows)));

                QList<Tracking::DetectedBlob> blobsInExpanded = findPlausibleBlobsInRoi(frame, analysisRoi);
                if (blobsInExpanded.isEmpty()) break;
                candidateBlobAfterExpansion = blobsInExpanded.first();
                if (analysisRoi.contains(candidateBlobAfterExpansion.boundingBox) || i == MAX_EXPANSION_ITERATIONS_BOUNDARY - 1) break;
            }
            blobToReport = candidateBlobAfterExpansion;

            if (m_lastPrimaryBlob.isValid) {
                Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, blobToReport, frame.size(), context.originalFrameNumber);
                blobForAnchor = persisted.isValid ? persisted : blobToReport;
            } else {
                blobForAnchor = blobToReport;
            }

            bool confirmedMerge = false;
            if (m_skipMergeDetectionNextFrame) {
                qDebug().noquote() << context.debugMessage << "Skipping merge detection (resuming from split).";
                confirmedMerge = false;
            } else if (m_lastPrimaryBlob.isValid && blobToReport.isValid && blobToReport.area > m_lastPrimaryBlob.area * MERGE_CONFIRM_RELATIVE_AREA_FACTOR) {
                confirmedMerge = true;
            } else if (blobToReport.isValid && blobToReport.area > m_maxBlobArea * MERGE_CONFIRM_ABSOLUTE_AREA_FACTOR) {
                confirmedMerge = true;
            }

            if (confirmedMerge) {
                nextState = Tracking::TrackerState::TrackingMerged;
            } else {
                // If not a confirmed merge, we assume the largest blob found after expansion is our single worm.
                blobForAnchor = blobToReport;
                nextState = Tracking::TrackerState::TrackingSingle;
            }
        }
    }

    // --- Final decision and update ---
    if (blobToReport.isValid && blobForAnchor.isValid) { // Both must be valid to proceed with tracking
        if (m_currentState != nextState) {
            m_currentState = nextState;
            emit stateChanged(m_wormId, m_currentState);
        }

        m_lastKnownPosition = cv::Point2f(static_cast<float>(blobForAnchor.centroid.x()), static_cast<float>(blobForAnchor.centroid.y()));
        QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, frame.size());

        m_lastPrimaryBlob = blobForAnchor; // Store the anchor blob
        m_lastFullBlob = blobToReport;     // Store the full reported blob

        emit positionUpdated(m_wormId, context.originalFrameNumber, m_lastPrimaryBlob, m_lastFullBlob, context.searchRoiUsedForThisFrame, m_currentState, splitCandidates);

        currentFixedSearchRoiRef_InOut = nextFrameSearchRoi;
        
        // Reset merge detection skip flag after processing this frame
        m_skipMergeDetectionNextFrame = false;
        
        return true;
    } else { // Lost track
        m_lastPrimaryBlob.isValid = false;
        m_lastFullBlob.isValid = false;

        if (m_currentState != Tracking::TrackerState::TrackingLost && m_currentState != Tracking::TrackerState::PausedForSplit) { // Don't override PausedForSplit if it was set
            m_currentState = Tracking::TrackerState::TrackingLost;
            emit stateChanged(m_wormId, m_currentState);
        } else if (m_currentState == Tracking::TrackerState::PausedForSplit && !blobToReport.isValid) {
            // If it was PausedForSplit but then we determined no valid blob to report (e.g. split candidate vanished)
            // then it's effectively lost from that paused state.
            m_currentState = Tracking::TrackerState::TrackingLost;
            emit stateChanged(m_wormId, m_currentState);
        }

        Tracking::DetectedBlob invalidBlobForSignal; // Default invalid
        emit positionUpdated(m_wormId, context.originalFrameNumber, invalidBlobForSignal, invalidBlobForSignal, context.searchRoiUsedForThisFrame, m_currentState, splitCandidates);
        // currentFixedSearchRoiRef_InOut remains unchanged from previous valid frame or initial ROI
        
        // Reset merge detection skip flag after processing this frame
        m_skipMergeDetectionNextFrame = false;
        
        return false;
    }
}

bool WormTracker::processFrameAsMerged(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentFixedSearchRoiRef_InOut)
{
    FrameProcessingContext context = initializeFrameProcessing(frame, sequenceFrameIndex, currentFixedSearchRoiRef_InOut);

    Tracking::DetectedBlob blobForAnchor;     // This will become m_lastPrimaryBlob (tracking anchor)
    Tracking::DetectedBlob blobToReport;      // This will become m_lastFullBlob (what's actually on screen)
    QList<Tracking::DetectedBlob> splitCandidates; // All candidates when transitioning to PausedForSplit
    blobForAnchor.isValid = false;
    blobToReport.isValid = false;
    Tracking::TrackerState nextState = m_currentState;

    if (context.plausibleBlobsInFixedRoi == 0) {
        qDebug().noquote() << context.debugMessage << "No plausible blobs found in initial ROI " << context.searchRoiUsedForThisFrame;
        nextState = Tracking::TrackerState::TrackingLost;
        // blobForAnchor and blobToReport remain invalid
    } else if (context.plausibleBlobsInFixedRoi == 1) {
        Tracking::DetectedBlob singleBlob = context.blobsInFixedRoi.first();
        //qDebug().noquote() << context.debugMessage << "Found 1 plausible blob. BBox:" << singleBlob.boundingBox << "Area:" << singleBlob.area;

        bool touchesBoundary = singleBlob.touchesROIboundary ||
                               !context.searchRoiUsedForThisFrame.contains(singleBlob.boundingBox);

        if (!touchesBoundary) {
            qDebug().noquote() << context.debugMessage << "Single blob fully contained.";
            // Check if area has reduced significantly, indicating split without boundary contact
            bool possibleSplitByAreaReduction = false;
            if (m_lastFullBlob.isValid && singleBlob.area < m_lastFullBlob.area * 0.75) {
                possibleSplitByAreaReduction = true;
                qDebug().noquote() << context.debugMessage << "Area significantly reduced from" << m_lastFullBlob.area << "to" << singleBlob.area << ". Possible split without boundary contact.";
            }

            if (possibleSplitByAreaReduction) {
                // Treat as split - provide current blob as split candidate
                blobToReport = singleBlob;
                blobForAnchor = singleBlob;
                splitCandidates.append(singleBlob); // Single candidate found in ROI
                nextState = Tracking::TrackerState::PausedForSplit;
            } else {
                blobToReport = singleBlob;
                blobForAnchor = singleBlob; // For single, anchor and report are the same
            }
        } else { // Touches boundary - perform expansion
            qDebug().noquote() << context.debugMessage << "Single blob touches ROI boundary. Initiating expansion analysis.";
            QRectF analysisRoi = context.searchRoiUsedForThisFrame;
            Tracking::DetectedBlob candidateBlobAfterExpansion = singleBlob;

            for (int i = 0; i < MAX_EXPANSION_ITERATIONS_BOUNDARY; ++i) {
                qreal newWidth = analysisRoi.width() * BOUNDARY_ROI_EXPANSION_FACTOR;
                qreal newHeight = analysisRoi.height() * BOUNDARY_ROI_EXPANSION_FACTOR;
                QPointF center = candidateBlobAfterExpansion.isValid ? candidateBlobAfterExpansion.centroid : analysisRoi.center();
                analysisRoi.setSize(QSizeF(newWidth, newHeight));
                analysisRoi.moveCenter(center);
                analysisRoi.setX(qMax(0.0, analysisRoi.x())); analysisRoi.setY(qMax(0.0, analysisRoi.y()));
                if (analysisRoi.right() > frame.cols) analysisRoi.setRight(frame.cols);
                if (analysisRoi.bottom() > frame.rows) analysisRoi.setBottom(frame.rows);
                analysisRoi.setWidth(qMin(analysisRoi.width(), static_cast<qreal>(frame.cols)));
                analysisRoi.setHeight(qMin(analysisRoi.height(), static_cast<qreal>(frame.rows)));

                QList<Tracking::DetectedBlob> blobsInExpanded = findPlausibleBlobsInRoi(frame, analysisRoi);
                if (blobsInExpanded.isEmpty()) break;

                candidateBlobAfterExpansion = blobsInExpanded.first(); // Take largest in expanded
                if (analysisRoi.contains(candidateBlobAfterExpansion.boundingBox) || i == MAX_EXPANSION_ITERATIONS_BOUNDARY - 1) break;
            }
            // After expansion, candidateBlobAfterExpansion is our best guess for the full entity
            blobToReport = candidateBlobAfterExpansion;

            // Determine anchor: if previously tracked, find persisting part. Otherwise, anchor is the full blob.
            if (m_lastPrimaryBlob.isValid) {
                Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, blobToReport, frame.size(), context.originalFrameNumber);
                blobForAnchor = persisted.isValid ? persisted : blobToReport; // Use persisted if found, else full
            } else {
                blobForAnchor = blobToReport;
            }
        }
    } else { // plausibleBlobsInFixedRoi > 1 - Potential split (was merged, now multiple blobs in ROI)
        qDebug().noquote() << context.debugMessage << "Previously merged, now >1 blobs (" << context.plausibleBlobsInFixedRoi << "). Analyzing for split.";
        
        // Step 1: Expand any boundary-touching blobs to get full entities
        QList<Tracking::DetectedBlob> expandedBlobCandidates;
        for (const Tracking::DetectedBlob& blob : context.blobsInFixedRoi) {
            if (!blob.isValid) continue;
            
            if (blob.touchesROIboundary || !context.searchRoiUsedForThisFrame.contains(blob.boundingBox)) {
                // Expand this blob to see its full extent
                QRectF analysisRoi = context.searchRoiUsedForThisFrame;
                Tracking::DetectedBlob expandedBlob = blob;
                
                for (int i = 0; i < MAX_EXPANSION_ITERATIONS_BOUNDARY; ++i) {
                    qreal newWidth = analysisRoi.width() * BOUNDARY_ROI_EXPANSION_FACTOR;
                    qreal newHeight = analysisRoi.height() * BOUNDARY_ROI_EXPANSION_FACTOR;
                    QPointF center = expandedBlob.isValid ? expandedBlob.centroid : analysisRoi.center();
                    analysisRoi.setSize(QSizeF(newWidth, newHeight));
                    analysisRoi.moveCenter(center);
                    analysisRoi.setX(qMax(0.0, analysisRoi.x())); analysisRoi.setY(qMax(0.0, analysisRoi.y()));
                    if (analysisRoi.right() > frame.cols) analysisRoi.setRight(frame.cols);
                    if (analysisRoi.bottom() > frame.rows) analysisRoi.setBottom(frame.rows);
                    analysisRoi.setWidth(qMin(analysisRoi.width(), static_cast<qreal>(frame.cols)));
                    analysisRoi.setHeight(qMin(analysisRoi.height(), static_cast<qreal>(frame.rows)));

                    QList<Tracking::DetectedBlob> blobsInExpanded = findPlausibleBlobsInRoi(frame, analysisRoi);
                    if (blobsInExpanded.isEmpty()) break;
                    
                    // Find closest to original blob centroid in expanded area
                    Tracking::DetectedBlob bestInExpansion;
                    bestInExpansion.isValid = false;
                    double minDistToOriginal = std::numeric_limits<double>::max();
                    for (const auto& b : blobsInExpanded) {
                        if (b.isValid) {
                            double d = Tracking::sqDistance(blob.centroid, b.centroid);
                            if (d < minDistToOriginal) {
                                minDistToOriginal = d;
                                bestInExpansion = b;
                            }
                        }
                    }
                    
                    if (bestInExpansion.isValid) expandedBlob = bestInExpansion;
                    else break;
                    
                    if (analysisRoi.contains(expandedBlob.boundingBox) || i == MAX_EXPANSION_ITERATIONS_BOUNDARY - 1) break;
                }
                expandedBlobCandidates.append(expandedBlob);
            } else {
                // Blob fully contained, use as-is
                expandedBlobCandidates.append(blob);
            }
        }
        
        // Step 2: Choose best candidate using both distance and area similarity
        Tracking::DetectedBlob chosenCandidate;
        chosenCandidate.isValid = false;
        
        if (m_lastFullBlob.isValid && !expandedBlobCandidates.isEmpty()) {
            // Find candidates by distance and area similarity
            Tracking::DetectedBlob closestByDistance, closestByArea;
            closestByDistance.isValid = false;
            closestByArea.isValid = false;
            
            double minDistance = std::numeric_limits<double>::max();
            double minAreaDiff = std::numeric_limits<double>::max();
            
            for (const Tracking::DetectedBlob& candidate : expandedBlobCandidates) {
                if (!candidate.isValid) continue;
                
                // Check distance to last primary blob
                if (m_lastPrimaryBlob.isValid) {
                    double dist = Tracking::sqDistance(m_lastPrimaryBlob.centroid, candidate.centroid);
                    if (dist < minDistance) {
                        minDistance = dist;
                        closestByDistance = candidate;
                    }
                }
                
                // Check area similarity to last full blob
                double areaDiff = qAbs(candidate.area - m_lastFullBlob.area);
                if (areaDiff < minAreaDiff) {
                    minAreaDiff = areaDiff;
                    closestByArea = candidate;
                }
            }
            
            // Choose best candidate: prioritize distance, use area only as minimum size sanity check
            if (closestByDistance.isValid) {
                // Check if distance-closest candidate meets minimum size requirement
                double minWormAreaThreshold = m_minBlobArea * 0.5; // Half of minimum blob area as sanity check
                if (closestByDistance.area >= minWormAreaThreshold) {
                    chosenCandidate = closestByDistance;
                    qDebug().noquote() << context.debugMessage << "Chose distance-closest candidate. Area:" << closestByDistance.area << "Distance-based selection.";
                } else {
                    qDebug().noquote() << context.debugMessage << "Distance-closest too small (" << closestByDistance.area << " < " << minWormAreaThreshold << "). Trying area-similar fallback.";
                    if (closestByArea.isValid && closestByArea.area >= minWormAreaThreshold) {
                        chosenCandidate = closestByArea;
                        qDebug().noquote() << context.debugMessage << "Chose area-similar candidate as fallback. Area:" << closestByArea.area << "vs previous:" << m_lastFullBlob.area;
                    } else {
                        chosenCandidate = closestByDistance; // Use anyway if no better option
                        qDebug().noquote() << context.debugMessage << "No valid fallback, using distance-closest despite small size.";
                    }
                }
            } else if (closestByArea.isValid) {
                chosenCandidate = closestByArea;
                qDebug().noquote() << context.debugMessage << "Only area-similar candidate available. Area:" << closestByArea.area;
            }
        } else if (!expandedBlobCandidates.isEmpty()) {
            // Fallback to largest
            chosenCandidate = expandedBlobCandidates.first();
        }
        
        // Step 3: Check for area reduction (split detection)
        if (chosenCandidate.isValid && m_lastFullBlob.isValid) {
            bool possibleSplitByAreaReduction = false;
            if (chosenCandidate.area < m_lastFullBlob.area * 0.75) {
                possibleSplitByAreaReduction = true;
                qDebug().noquote() << context.debugMessage << "Area significantly reduced from" << m_lastFullBlob.area << "to" << chosenCandidate.area << ". Split detected by area reduction.";
            }
            
            if (possibleSplitByAreaReduction) {
                // Treat as split
                blobToReport = chosenCandidate;
                blobForAnchor = chosenCandidate;
                splitCandidates = context.blobsInFixedRoi; // Provide all original candidates found in ROI
                nextState = Tracking::TrackerState::PausedForSplit;
            } else {
                // Step 4: Still looks like merge continuation
                blobToReport = chosenCandidate;
                if (m_lastPrimaryBlob.isValid) {
                    Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, blobToReport, frame.size(), context.originalFrameNumber);
                    blobForAnchor = persisted.isValid ? persisted : blobToReport;
                } else {
                    blobForAnchor = blobToReport;
                }
                nextState = Tracking::TrackerState::TrackingMerged;
                qDebug().noquote() << context.debugMessage << "Area maintained, continuing merge. Area:" << chosenCandidate.area;
            }
        } else if (chosenCandidate.isValid) {
            // No previous blob to compare, treat as continued merge
            blobToReport = chosenCandidate;
            blobForAnchor = chosenCandidate;
            nextState = Tracking::TrackerState::TrackingMerged;
        } else {
            // No valid candidate found
            nextState = Tracking::TrackerState::TrackingLost;
        }
    }

    // --- Final decision and update ---
    if (blobToReport.isValid && blobForAnchor.isValid) { // Both must be valid to proceed with tracking
        if (m_currentState != nextState) {
            m_currentState = nextState;
            emit stateChanged(m_wormId, m_currentState);
        }

        m_lastKnownPosition = cv::Point2f(static_cast<float>(blobForAnchor.centroid.x()), static_cast<float>(blobForAnchor.centroid.y()));
        QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, frame.size());

        m_lastPrimaryBlob = blobForAnchor; // Store the anchor blob
        m_lastFullBlob = blobToReport;     // Store the full reported blob

        emit positionUpdated(m_wormId, context.originalFrameNumber, m_lastPrimaryBlob, m_lastFullBlob, context.searchRoiUsedForThisFrame, m_currentState, splitCandidates);

        currentFixedSearchRoiRef_InOut = nextFrameSearchRoi;
        return true;
    } else { // Lost track
        m_lastPrimaryBlob.isValid = false;
        m_lastFullBlob.isValid = false;

        if (m_currentState != Tracking::TrackerState::TrackingLost && m_currentState != Tracking::TrackerState::PausedForSplit) { // Don't override PausedForSplit if it was set
            m_currentState = Tracking::TrackerState::TrackingLost;
            emit stateChanged(m_wormId, m_currentState);
        } else if (m_currentState == Tracking::TrackerState::PausedForSplit && !blobToReport.isValid) {
            // If it was PausedForSplit but then we determined no valid blob to report (e.g. split candidate vanished)
            // then it's effectively lost from that paused state.
            m_currentState = Tracking::TrackerState::TrackingLost;
            emit stateChanged(m_wormId, m_currentState);
        }

        Tracking::DetectedBlob invalidBlobForSignal; // Default invalid
        emit positionUpdated(m_wormId, context.originalFrameNumber, invalidBlobForSignal, invalidBlobForSignal, context.searchRoiUsedForThisFrame, m_currentState, splitCandidates);
        // currentFixedSearchRoiRef_InOut remains unchanged from previous valid frame or initial ROI
        return false;
    }
}

void WormTracker::resumeTrackingWithAssignedTarget(const Tracking::DetectedBlob& targetBlob)
{
    if (m_currentState != Tracking::TrackerState::PausedForSplit) {
        qWarning() << "WormTracker ID" << m_wormId << getDirection()
        << ": resumeTrackingWithAssignedTarget called but not in PausedForSplit state. Current state:"
        << static_cast<int>(m_currentState) << ". Ignoring.";
        return;
    }

    int signedId = -1 * m_wormId ? getDirection() == TrackingDirection::Backward : m_wormId;
    int originalFrameNumber = (m_direction == TrackingDirection::Forward) ?
                                  m_videoKeyFrameNum + m_currFrameNum :
                                  m_videoKeyFrameNum - 1 - m_currFrameNum;

    QString dmsg = QString("WT %1 FN%2 | resumeTracking | ").arg(m_wormId).arg(originalFrameNumber);

    qDebug().noquote() << dmsg << targetBlob.centroid.x() << "," << targetBlob.centroid.y() << "Area:" << targetBlob.area;

    if (!targetBlob.isValid) {
        qWarning() << "WormTracker ID" << m_wormId << getDirection()
        << ": resumeTrackingWithAssignedTarget called with invalid blob. Transitioning to Lost.";
        m_currentState = Tracking::TrackerState::TrackingLost;
        m_lastPrimaryBlob.isValid = false;
        m_lastFullBlob.isValid = false;
        // m_trackingActive = false; // Don't stop the loop, let it try to recover or finish
        emit stateChanged(m_wormId, m_currentState);
        // Emit a lost position update for this frame
        if (m_framesToProcess && m_currFrameNum < static_cast<int>(m_framesToProcess->size()) && m_currFrameNum >=0) {
            Tracking::DetectedBlob invalidBlob;
            QList<Tracking::DetectedBlob> emptySplitCandidates;
            emit positionUpdated(m_wormId, originalFrameNumber, invalidBlob, invalidBlob, m_currentSearchRoi, m_currentState, emptySplitCandidates);
        }
        m_currFrameNum++; // Advance frame, even if lost, to continue processing sequence
        if (m_trackingActive) {
            QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
        } else {
            emit finished();
        }
        return;
    }

    m_lastKnownPosition = cv::Point2f(static_cast<float>(targetBlob.centroid.x()), static_cast<float>(targetBlob.centroid.y()));

    cv::Size currentFrameCvSize = cv::Size(640,480); // Fallback
    if (m_framesToProcess && m_currFrameNum < static_cast<int>(m_framesToProcess->size()) && m_currFrameNum >= 0) {
        if (!m_framesToProcess->at(m_currFrameNum).empty()) {
            currentFrameCvSize = m_framesToProcess->at(m_currFrameNum).size();
        }
    } else if (m_framesToProcess && !m_framesToProcess->empty()) {
        if (!m_framesToProcess->at(0).empty()) {
            currentFrameCvSize = m_framesToProcess->at(0).size();
        }
    }

    m_currentSearchRoi = adjustRoiPos(m_lastKnownPosition, currentFrameCvSize);
    m_lastPrimaryBlob = targetBlob; // This is the anchor
    m_lastFullBlob = targetBlob;    // And also what's reported
    m_currentState = Tracking::TrackerState::TrackingSingle; // Resumed as single
    m_skipMergeDetectionNextFrame = true; // Skip merge detection for one frame after resuming from split
    // emit stateChanged(m_wormId, m_currentState); // State change will be implicit in next positionUpdate

    qDebug().noquote() << "WormTracker ID" << m_wormId << getDirection()
                       << ": Resumed. New Search ROI:" << m_currentSearchRoi
                       << "State:" << static_cast<int>(m_currentState);

    // Emit position update for the *current* frame where it was paused, now with the assigned target
    if (m_framesToProcess && m_currFrameNum < static_cast<int>(m_framesToProcess->size()) && m_currFrameNum >=0) {
        int originalFrameNumber = (m_direction == TrackingDirection::Forward) ?
                                      m_videoKeyFrameNum + m_currFrameNum :
                                      m_videoKeyFrameNum - 1 - m_currFrameNum;
        // searchRoiUsedForThisFrame would be the ROI when it paused. We might not have it easily here.
        // Using m_currentSearchRoi (which is now for the *next* frame) is not ideal for this specific emit.
        // For simplicity, we can pass the new m_currentSearchRoi or the old one if stored.
        // Let's assume the ROI that *led* to pause is what TM cares about, but we don't have it.
        // So, we pass the ROI that will be used *next*.
        QList<Tracking::DetectedBlob> emptySplitCandidates;
        emit positionUpdated(m_wormId, originalFrameNumber, m_lastPrimaryBlob, m_lastFullBlob, m_currentSearchRoi, m_currentState, emptySplitCandidates);
    }


    m_currFrameNum++; // Advance to process the *next* frame

    if (m_trackingActive) {
        QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
    } else { // Should not happen if resume is called correctly
        emit finished();
    }
}


Tracking::DetectedBlob WormTracker::findLargestBlobComponentInMask(const cv::Mat& mask, const QString& /*debugContextName*/) {
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

    if (largestContourIdx != -1 && maxArea > (m_minBlobArea * 0.1)) {
        const auto& largestContour = contours[largestContourIdx];
        cv::Moments mu = cv::moments(largestContour);
        if (mu.m00 > std::numeric_limits<double>::epsilon()) {
            resultBlob.centroid = QPointF(static_cast<qreal>(mu.m10 / mu.m00), static_cast<qreal>(mu.m01 / mu.m00));
            cv::Rect cvBox = cv::boundingRect(largestContour);
            resultBlob.boundingBox = QRectF(cvBox.x, cvBox.y, cvBox.width, cvBox.height);
            resultBlob.area = maxArea;
            resultBlob.contourPoints = largestContour;
            resultBlob.isValid = true;
        }
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
    roiX = qMax(0.0, roiX);
    roiY = qMax(0.0, roiY);

    roiWidth = qMin(roiWidth, static_cast<qreal>(frameSize.width));
    roiHeight = qMin(roiHeight, static_cast<qreal>(frameSize.height));

    return QRectF(roiX, roiY, roiWidth, roiHeight);
}
