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
        qWarning().noquote() << getDebugLabel("startTracking") << "No frames provided to tracker.";
        emit errorOccurred(m_wormId, "No frames provided to tracker.");
        emit finished();
        return;
    }

    m_trackingActive = true;
    m_currentState = Tracking::TrackerState::TrackingSingle; // Initial state when tracking starts
    m_lastPrimaryBlob.isValid = false; // Reset for a new tracking session
    m_lastFullBlob.isValid = false;    // Reset for a new tracking session
    m_currFrameNum = 0; // Reset frame counter for new tracking session

    qDebug().noquote() << getDebugLabel("startTracking") << "Starting tracking. Frames:" << m_framesToProcess->size()
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

            // Use the unified processFrame function with appropriate mode
            bool asMerged = (m_currentState == Tracking::TrackerState::TrackingMerged);
            foundTargetThisFrame = processFrame(asMerged, currentFrame, m_currFrameNum, m_currentSearchRoi);

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
            qDebug().noquote() << getDebugLabel("continueTracking") << "Tracking no longer active. Emitting finished.";
            emit progress(m_wormId, 100);
            emit finished();
        }

    } else {
        if (m_trackingActive) {
            qDebug().noquote() << getDebugLabel("continueTracking") << "Frame processing loop completed naturally.";
            emit progress(m_wormId, 100);
            m_trackingActive = false;
        }
        qDebug().noquote() << getDebugLabel("continueTracking") << "Finished processing. Final state:" << static_cast<int>(m_currentState);
        emit finished();
    }
}

void WormTracker::stopTracking() {
    qDebug().noquote() << getDebugLabel("stopTracking") << "Called. Current state:" << static_cast<int>(m_currentState);
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
        qDebug().noquote()<< "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                     << ": Persisting component identified as 'old overlap'. Centroid:" << oldOverlapComponentDetails.centroid
                     << "Area:" << oldOverlapComponentDetails.area;
        persistingComponent = oldOverlapComponentDetails;
    }
    // If old overlap is not good, check if the new growth part is significant
    // This could happen if the worm moved quickly, and the "new growth" is actually the worm in a new spot.
    else if (newGrowthComponentDetails.isValid && newGrowthComponentDetails.area > (m_minBlobArea * significanceThresholdFactor)) {
        qDebug().noquote()<< "WormTracker ID" << m_wormId << "Frame" << originalFrameNumberForDebug
                     << ": Persisting component identified as 'new growth' (old overlap was small/invalid). Centroid:" << newGrowthComponentDetails.centroid
                     << "Area:" << newGrowthComponentDetails.area;
        persistingComponent = newGrowthComponentDetails;
    } else {
        // If neither is conclusive, it's hard to define a "persisting" part.
        // Fallback: if the currentFullBlob itself is plausibly a single worm, use that.
        // Otherwise, this function returns an invalid blob, and the caller might use the full current blob.
        if (currentFrameFullBlob.area >= m_minBlobArea && currentFrameFullBlob.area <= m_maxBlobArea * 1.2) { // Allow slightly larger for persistence
            qDebug().noquote() << getDebugLabel("findPersistingComponent") << "Persisting component analysis inconclusive. Using full current blob as it's plausible single.";
            persistingComponent = currentFrameFullBlob;
        } else {
            qDebug().noquote() << getDebugLabel("findPersistingComponent") << "Persisting component analysis inconclusive, components too small or currentFull too large. Returning invalid.";
            // persistingComponent remains invalid
        }
    }
    return persistingComponent;
}


WormTracker::FrameProcessingContext WormTracker::initializeFrameProcessing(const cv::Mat& frame, int sequenceFrameIndex, const QRectF& searchRoi)
{
    FrameProcessingContext context;

    if (m_direction == TrackingDirection::Forward) {
        context.originalFrameNumber = m_videoKeyFrameNum + sequenceFrameIndex;
    } else {
        context.originalFrameNumber = m_videoKeyFrameNum - 1 - sequenceFrameIndex;
    }

    context.debugMessage = getDebugLabel("processFrame");
    context.searchRoiUsedForThisFrame = searchRoi;
    context.blobsInFixedRoi = findPlausibleBlobsInRoi(frame, searchRoi);
    context.plausibleBlobsInFixedRoi = context.blobsInFixedRoi.count();

    qDebug().noquote() << getDebugLabel("initializeFrameProcessing") << "State:" << static_cast<int>(m_currentState) << " Search begins with ROI " << context.searchRoiUsedForThisFrame << "LastPrimaryValid:" << m_lastPrimaryBlob.isValid;

    return context;
}

// Unified frame processing - handles both single and merged tracking modes
bool WormTracker::processFrame(bool asMerged, const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentRoi)
{
    // 1. Initialize processing context
    FrameProcessingContext context = initializeFrameProcessing(frame, sequenceFrameIndex, currentRoi);
    
    // 2. Process based on blob count
    if (context.plausibleBlobsInFixedRoi == 0) {
        return handleLostTracking(context, currentRoi);
    } else if (context.plausibleBlobsInFixedRoi == 1) {
        return handleSingleBlobCase(asMerged, context.blobsInFixedRoi.first(), frame, context, currentRoi);
    } else {
        return handleMultipleBlobsCase(asMerged, context.blobsInFixedRoi, frame, context, currentRoi);
    }
}

// Handle case when no blobs are found
bool WormTracker::handleLostTracking(const FrameProcessingContext& context, QRectF& currentRoi)
{
    qDebug().noquote() << getDebugLabel("handleLostTracking") << "No plausible blobs found in initial ROI " << context.searchRoiUsedForThisFrame 
                      << " Size: " << context.searchRoiUsedForThisFrame.size() 
                      << " Width: " << context.searchRoiUsedForThisFrame.width() 
                      << " Height: " << context.searchRoiUsedForThisFrame.height();
    
    qDebug().noquote() << getDebugLabel("handleLostTracking") << "Current ROI (before update): " << currentRoi 
                      << " Size: " << currentRoi.size() 
                      << " Width: " << currentRoi.width() 
                      << " Height: " << currentRoi.height();
    
    m_lastPrimaryBlob.isValid = false;
    m_lastFullBlob.isValid = false;

    if (m_currentState != Tracking::TrackerState::TrackingLost && 
        m_currentState != Tracking::TrackerState::PausedForSplit) {
        m_currentState = Tracking::TrackerState::TrackingLost;
        emit stateChanged(m_wormId, m_currentState);
    } else if (m_currentState == Tracking::TrackerState::PausedForSplit && 
               !m_lastPrimaryBlob.isValid) {
        // If it was PausedForSplit but then we determined no valid blob to report
        // then it's effectively lost from that paused state.
        m_currentState = Tracking::TrackerState::TrackingLost;
        emit stateChanged(m_wormId, m_currentState);
    }

    Tracking::DetectedBlob invalidBlobForSignal; // Default invalid
    QList<Tracking::DetectedBlob> emptySplitCandidates;
    
    emit positionUpdated(m_wormId, context.originalFrameNumber, invalidBlobForSignal, 
                         invalidBlobForSignal, context.searchRoiUsedForThisFrame, 
                         m_currentState, emptySplitCandidates);
    
    // Reset merge detection skip flag
    m_skipMergeDetectionNextFrame = false;
    
    // Debug: Verify ROI isn't changing unexpectedly
    qDebug().noquote() << getDebugLabel("handleLostTracking") << "Current ROI (after update): " << currentRoi 
                      << " Size: " << currentRoi.size() 
                      << " Width: " << currentRoi.width() 
                      << " Height: " << currentRoi.height()
                      << " m_initialRoiEdge: " << m_initialRoiEdge;
    
    return false;
}

// Check if blob is touching boundary
bool WormTracker::isBlobTouchingBoundary(const Tracking::DetectedBlob& blob, const QRectF& roi)
{
    return blob.touchesROIboundary || !roi.contains(blob.boundingBox);
}

// Handle a single blob case
bool WormTracker::handleSingleBlobCase(bool asMerged, const Tracking::DetectedBlob& blob, 
                                      const cv::Mat& frame, const FrameProcessingContext& context, 
                                      QRectF& currentRoi)
{
    qDebug().noquote() << getDebugLabel("handleSingleBlobCase") << "Found 1 plausible blob. BBox:" << blob.boundingBox 
                      << " Area:" << blob.area << " Hull Area:" << blob.convexHullArea;
    
    bool touchesBoundary = isBlobTouchingBoundary(blob, context.searchRoiUsedForThisFrame);
    
    if (!touchesBoundary) {
        return handleNonBoundaryBlob(asMerged, blob, context, frame, currentRoi);
    } else {
        return handleBoundaryTouchingBlob(asMerged, blob, frame, context, currentRoi);
    }
}

// Handle a blob that's fully contained in the ROI
bool WormTracker::handleNonBoundaryBlob(bool asMerged, const Tracking::DetectedBlob& blob, 
                                       const FrameProcessingContext& context, const cv::Mat& frame, QRectF& currentRoi)
{
    qDebug().noquote() << getDebugLabel("handleNonBoundaryBlob") << "Single blob fully contained.";
    
    // Check for split by area reduction
    bool isSplit = detectSplitByAreaReduction(blob);
    
    if (isSplit) {
        // Report as split
        QList<Tracking::DetectedBlob> splitCandidates;
        splitCandidates.append(blob);
        return updateTrackingState(blob, blob, splitCandidates, 
                                  Tracking::TrackerState::PausedForSplit, 
                                  context, frame.size(), currentRoi);
    } else if (!asMerged) {
        // In single mode, check for merge
        bool confirmedMerge = detectMergeByAreaIncrease(blob);
        
        if (confirmedMerge) {
            qDebug().noquote() << getDebugLabel("handleNonBoundaryBlob") << "Single blob fully contained CONFIRMS MERGE. Area:" << blob.area;
            return updateTrackingState(blob, blob, QList<Tracking::DetectedBlob>(), 
                                      Tracking::TrackerState::TrackingMerged, 
                                      context, frame.size(), currentRoi);
        } else {
            // Continue normal single tracking
            return updateTrackingState(blob, blob, QList<Tracking::DetectedBlob>(), 
                                      Tracking::TrackerState::TrackingSingle, 
                                      context, frame.size(), currentRoi);
        }
    } else {
        // In merged mode, continue as merged
        return updateTrackingState(blob, blob, QList<Tracking::DetectedBlob>(), 
                                  Tracking::TrackerState::TrackingMerged, 
                                  context, frame.size(), currentRoi);
    }
}

// Expand a blob that touches the boundary
Tracking::DetectedBlob WormTracker::expandBlobTouchingBoundary(const Tracking::DetectedBlob& initialBlob, 
                                                             QRectF& expandedRoi, const cv::Mat& frame)
{
    qDebug().noquote() << getDebugLabel("expandBlobTouchingBoundary") << "Single blob touches ROI boundary. Initiating expansion analysis.";
    Tracking::DetectedBlob expandedBlob = initialBlob;
    
    for (int i = 0; i < MAX_EXPANSION_ITERATIONS_BOUNDARY; ++i) {
        qreal newWidth = expandedRoi.width() * BOUNDARY_ROI_EXPANSION_FACTOR;
        qreal newHeight = expandedRoi.height() * BOUNDARY_ROI_EXPANSION_FACTOR;
        QPointF center = expandedBlob.isValid ? expandedBlob.centroid : expandedRoi.center();
        
        expandedRoi.setSize(QSizeF(newWidth, newHeight));
        expandedRoi.moveCenter(center);
        
        // Ensure ROI stays within frame bounds
        expandedRoi.setX(qMax(0.0, expandedRoi.x())); 
        expandedRoi.setY(qMax(0.0, expandedRoi.y()));
        if (expandedRoi.right() > frame.cols) expandedRoi.setRight(frame.cols);
        if (expandedRoi.bottom() > frame.rows) expandedRoi.setBottom(frame.rows);
        expandedRoi.setWidth(qMin(expandedRoi.width(), static_cast<qreal>(frame.cols)));
        expandedRoi.setHeight(qMin(expandedRoi.height(), static_cast<qreal>(frame.rows)));
        
        QList<Tracking::DetectedBlob> blobsInExpanded = findPlausibleBlobsInRoi(frame, expandedRoi);
        if (blobsInExpanded.isEmpty()) break;
        
        // Find closest blob to original blob's centroid
        Tracking::DetectedBlob closestInExpansion;
        closestInExpansion.isValid = false;
        double minDistToOriginal = std::numeric_limits<double>::max();
        
        for (const auto& b : blobsInExpanded) {
            if (b.isValid) {
                double d = Tracking::sqDistance(initialBlob.centroid, b.centroid);
                if (d < minDistToOriginal) {
                    minDistToOriginal = d;
                    closestInExpansion = b;
                }
            }
        }
        
        expandedBlob = closestInExpansion.isValid ? closestInExpansion : blobsInExpanded.first();
        
        if (expandedRoi.contains(expandedBlob.boundingBox) || i == MAX_EXPANSION_ITERATIONS_BOUNDARY - 1) {
            break;
        }
    }
    
    return expandedBlob;
}
// Handle a blob that touches the boundary
bool WormTracker::handleBoundaryTouchingBlob(bool asMerged, const Tracking::DetectedBlob& blob, 
                                          const cv::Mat& frame, const FrameProcessingContext& context, 
                                          QRectF& currentRoi)
{
    // Expand ROI to get full blob
    QRectF expandedRoi = context.searchRoiUsedForThisFrame;
    Tracking::DetectedBlob expandedBlob = expandBlobTouchingBoundary(blob, expandedRoi, frame);
    
    // After expansion, expandedBlob is our best guess for the full entity
    Tracking::DetectedBlob blobToReport = expandedBlob;
    Tracking::DetectedBlob blobForAnchor;
    
    // Determine anchor: if previously tracked, find persisting part. Otherwise, anchor is the full blob.
    if (m_lastPrimaryBlob.isValid) {
        Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, blobToReport, frame.size(), context.originalFrameNumber);
        blobForAnchor = persisted.isValid ? persisted : blobToReport;
        qDebug().noquote() << getDebugLabel("handleBoundaryTouchingBlob") << "findPersistingComponent result: valid=" << persisted.isValid 
                          << " centroid=" << (persisted.isValid ? persisted.centroid : QPointF(-1, -1))
                          << " area=" << (persisted.isValid ? persisted.area : -1);
    } else {
        blobForAnchor = blobToReport;
    }
    
    // Check for potential split by area reduction
    bool isSplit = detectSplitByAreaReduction(expandedBlob);
    if (isSplit) {
        QList<Tracking::DetectedBlob> splitCandidates;
        splitCandidates.append(expandedBlob);
        qDebug().noquote() << getDebugLabel("handleBoundaryTouchingBlob") << "After boundary expansion, detected potential split by area.";
        return updateTrackingState(blobForAnchor, blobToReport, splitCandidates, 
                                 Tracking::TrackerState::PausedForSplit, context, frame.size(), currentRoi);
    }
    
    // Check for merge if in single mode
    if (!asMerged) {
        bool confirmedMerge = detectMergeByAreaIncrease(blobToReport);
        
        if (confirmedMerge) {
            qDebug().noquote() << getDebugLabel("handleBoundaryTouchingBlob") << "Boundary touch + expansion CONFIRMS MERGE. ReportedArea:" 
                              << blobToReport.area << " AnchorArea:" << blobForAnchor.area;
            return updateTrackingState(blobForAnchor, blobToReport, QList<Tracking::DetectedBlob>(), 
                                     Tracking::TrackerState::TrackingMerged, context, frame.size(), currentRoi);
        } else {
            qDebug().noquote() << getDebugLabel("handleBoundaryTouchingBlob") << "Boundary touch + expansion, NO merge by area. ReportedArea:" << blobToReport.area;
            // If it wasn't a merge, it's a single worm. Anchor and report should ideally be the same.
            blobForAnchor = blobToReport;
            return updateTrackingState(blobForAnchor, blobToReport, QList<Tracking::DetectedBlob>(), 
                                     Tracking::TrackerState::TrackingSingle, context, frame.size(), currentRoi);
        }
    } else {
        // In merged mode, continue as merged
        return updateTrackingState(blobForAnchor, blobToReport, QList<Tracking::DetectedBlob>(), 
                                 Tracking::TrackerState::TrackingMerged, context, frame.size(), currentRoi);
    }
}

// Handle multiple blobs case
bool WormTracker::handleMultipleBlobsCase(bool asMerged, const QList<Tracking::DetectedBlob>& blobs, 
                                        const cv::Mat& frame, const FrameProcessingContext& context, 
                                        QRectF& currentRoi)
{
    if (asMerged) {
        // For merged mode, multiple blobs might indicate a split
        qDebug().noquote() << getDebugLabel("handleMultipleBlobsCase") << "Previously merged, now >1 blobs (" 
                          << blobs.size() << "). Analyzing for split.";
        
        // Log information about each blob
        int blobIndex = 0;
        bool anyBlobTouchesBoundary = false;
        
        for (const auto& blob : blobs) {
            double hollowness = (blob.convexHullArea - blob.area) / blob.convexHullArea;
            bool touchesBoundary = isBlobTouchingBoundary(blob, context.searchRoiUsedForThisFrame);
            anyBlobTouchesBoundary = anyBlobTouchesBoundary || touchesBoundary;
            
            qDebug().noquote() << getDebugLabel("handleMultipleBlobsCase") << "Blob[" << blobIndex++ << "] Area:" << blob.area 
                              << " Hull:" << blob.convexHullArea << " Hollowness:" << QString::number(hollowness, 'f', 2)
                              << " BBox:" << blob.boundingBox
                              << " Touches:" << touchesBoundary;
        }
        
        // Select best candidate
        Tracking::DetectedBlob bestCandidate = selectBestBlobCandidate(blobs);
        
        if (!bestCandidate.isValid) {
            return handleLostTracking(context, currentRoi);
        }
        
        // Check if this is a split
        bool isSplit = detectSplitByAreaReduction(bestCandidate);
        
        if (isSplit) {
            // Report all blobs as split candidates
            return updateTrackingState(bestCandidate, bestCandidate, blobs, 
                                     Tracking::TrackerState::PausedForSplit, context, frame.size(), currentRoi);
        } else {
            // Find persisting component for anchor
            Tracking::DetectedBlob anchor;
            if (m_lastPrimaryBlob.isValid) {
                Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, bestCandidate, frame.size(), context.originalFrameNumber);
                anchor = persisted.isValid ? persisted : bestCandidate;
            } else {
                anchor = bestCandidate;
            }
            
            return updateTrackingState(anchor, bestCandidate, QList<Tracking::DetectedBlob>(), 
                                     Tracking::TrackerState::TrackingMerged, context, frame.size(), currentRoi);
        }
    } else {
        // For single mode, multiple blobs might indicate a merge
        qDebug().noquote() << getDebugLabel("handleMultipleBlobsCase") << "Previously single, now >1 blobs. Potential merge.";
        
        // Select the blob closest to the previous primary blob
        Tracking::DetectedBlob searchCandidate = selectBestBlobCandidate(blobs);
        
        if (!searchCandidate.isValid) {
            return handleLostTracking(context, currentRoi);
        }
        
        bool touchesBoundary = isBlobTouchingBoundary(searchCandidate, context.searchRoiUsedForThisFrame);
        
        if (!touchesBoundary) {
            // Non-boundary case
            return updateTrackingState(searchCandidate, searchCandidate, QList<Tracking::DetectedBlob>(), 
                                     Tracking::TrackerState::TrackingSingle, context, frame.size(), currentRoi);
        } else {
            // Boundary case, need to expand
            return handleBoundaryTouchingBlob(asMerged, searchCandidate, frame, context, currentRoi);
        }
    }
}

// Find persisting anchor using existing findPersistingComponent
Tracking::DetectedBlob WormTracker::findPersistingAnchor(const Tracking::DetectedBlob& currentBlob, 
                                                      const cv::Size& frameSize, 
                                                      const FrameProcessingContext& context)
{
    if (!m_lastPrimaryBlob.isValid) {
        return currentBlob;
    }
    
    Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, currentBlob, frameSize, context.originalFrameNumber);
    return persisted.isValid ? persisted : currentBlob;
}

// Select best blob from multiple candidates
Tracking::DetectedBlob WormTracker::selectBestBlobCandidate(const QList<Tracking::DetectedBlob>& blobs)
{
    if (blobs.isEmpty()) {
        Tracking::DetectedBlob invalidBlob;
        invalidBlob.isValid = false;
        return invalidBlob;
    }
    
    // Default to largest blob (blobs are already sorted by area in findPlausibleBlobsInRoi)
    Tracking::DetectedBlob bestCandidate = blobs.first();
    
    // If we have previous tracking info, refine selection
    if (m_lastPrimaryBlob.isValid) {
        double minSqDist = std::numeric_limits<double>::max();
        Tracking::DetectedBlob closestBlob;
        closestBlob.isValid = false;
        
        for (const Tracking::DetectedBlob& currentBlob : blobs) {
            if (currentBlob.isValid) {
                double sqDist = Tracking::sqDistance(m_lastPrimaryBlob.centroid, currentBlob.centroid);
                if (sqDist < minSqDist) {
                    minSqDist = sqDist;
                    closestBlob = currentBlob;
                }
            }
        }
        
        if (closestBlob.isValid) {
            bestCandidate = closestBlob;
        }
    }
    
    return bestCandidate;
}

// Detect split by area reduction
bool WormTracker::detectSplitByAreaReduction(const Tracking::DetectedBlob& currentBlob)
{
    if (!m_lastFullBlob.isValid) return false;
    
    // Calculate both area metrics
    double hullAreaRatio = currentBlob.convexHullArea / m_lastFullBlob.convexHullArea;
    double regularAreaRatio = currentBlob.area / m_lastFullBlob.area;
    
    // Log area changes
    qDebug().noquote() << getDebugLabel("detectSplitByAreaReduction") << "Area check: prev hull=" << m_lastFullBlob.convexHullArea
                      << " current hull=" << currentBlob.convexHullArea
                      << " hull ratio=" << QString::number(hullAreaRatio, 'f', 2)
                      << " (regular area ratio=" << QString::number(regularAreaRatio, 'f', 2) << ")";
                      
    // Split threshold: Both hull area AND regular area must show significant reduction
    // Both metrics must be less than 80% of previous frame's values
    return (hullAreaRatio < 0.80) && (regularAreaRatio < 0.80);
}

// Detect merge by area increase
bool WormTracker::detectMergeByAreaIncrease(const Tracking::DetectedBlob& currentBlob)
{
    if (m_skipMergeDetectionNextFrame) {
        qDebug().noquote() << getDebugLabel("detectMergeByAreaIncrease") << "Skipping merge detection (resuming from split).";
        return false;
    }
    
    if (m_lastPrimaryBlob.isValid && 
        currentBlob.area > m_lastPrimaryBlob.area * MERGE_CONFIRM_RELATIVE_AREA_FACTOR) {
        qDebug().noquote() << getDebugLabel("detectMergeByAreaIncrease") << "Area significantly increased from" << m_lastPrimaryBlob.area 
                          << "to" << currentBlob.area << ". Merge detected.";
        return true;
    }
    
    if (currentBlob.area > m_maxBlobArea * MERGE_CONFIRM_ABSOLUTE_AREA_FACTOR) {
        qDebug().noquote() << getDebugLabel("detectMergeByAreaIncrease") << "Area exceeds merge threshold:" << currentBlob.area 
                          << "vs" << (m_maxBlobArea * MERGE_CONFIRM_ABSOLUTE_AREA_FACTOR) << ". Merge detected.";
        return true;
    }
    
    return false;
}

// Update tracking state and emit signals
bool WormTracker::updateTrackingState(const Tracking::DetectedBlob& blobForAnchor, 
                                    const Tracking::DetectedBlob& blobToReport,
                                    const QList<Tracking::DetectedBlob>& splitCandidates, 
                                    Tracking::TrackerState nextState,
                                    const FrameProcessingContext& context,
                                    const cv::Size& frameSize,
                                    QRectF& currentRoi)
{
    if (!blobToReport.isValid || !blobForAnchor.isValid) {
        qDebug().noquote() << getDebugLabel("updateTrackingState") << "Invalid blobs in updateTrackingState, calling handleLostTracking";
        return handleLostTracking(context, currentRoi);
    }
    
    // Update state if changed
    if (m_currentState != nextState) {
        m_currentState = nextState;
        emit stateChanged(m_wormId, m_currentState);
    }
    
    // Update position tracking
    m_lastKnownPosition = cv::Point2f(static_cast<float>(blobForAnchor.centroid.x()), 
                                     static_cast<float>(blobForAnchor.centroid.y()));
    
    // Verify frame size is valid
    if (frameSize.width <= 0 || frameSize.height <= 0) {
        qWarning().noquote() << getDebugLabel("updateTrackingState") << "Invalid frame size:" << frameSize.width << "x" << frameSize.height;
        // Use a fallback size to prevent ROI calculation errors
        cv::Size fallbackSize(1280, 720);
        QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, fallbackSize);
        qDebug().noquote() << getDebugLabel("updateTrackingState") << "Using FALLBACK frame size:" << fallbackSize.width << "x" << fallbackSize.height;
        qDebug().noquote() << getDebugLabel("updateTrackingState") << "Current ROI (before update):" << currentRoi;
        qDebug().noquote() << getDebugLabel("updateTrackingState") << "Adjusted ROI with fallback:" << nextFrameSearchRoi;
        currentRoi = nextFrameSearchRoi;
    } else {
        QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, frameSize);
        qDebug().noquote() << getDebugLabel("updateTrackingState") << "Current ROI (before update):" << currentRoi;
        qDebug().noquote() << getDebugLabel("updateTrackingState") << "Adjusted ROI for next frame:" << nextFrameSearchRoi 
                          << "from position" << m_lastKnownPosition.x << "," << m_lastKnownPosition.y
                          << "with frame size" << frameSize.width << "x" << frameSize.height;
        currentRoi = nextFrameSearchRoi;
    }
    
    // Store blob information
    m_lastPrimaryBlob = blobForAnchor;
    m_lastFullBlob = blobToReport;
    
    // Emit position update
    emit positionUpdated(m_wormId, context.originalFrameNumber, 
                        m_lastPrimaryBlob, m_lastFullBlob, 
                        context.searchRoiUsedForThisFrame, 
                        m_currentState, splitCandidates);
    
    // ROI has already been updated above, verify it's valid
    qDebug().noquote() << getDebugLabel("updateTrackingState") << "Final ROI:" << currentRoi 
                      << " Size: " << currentRoi.size() 
                      << " Width: " << currentRoi.width() 
                      << " Height: " << currentRoi.height();
    
    if (!currentRoi.isValid() || currentRoi.isEmpty()) {
        qWarning().noquote() << getDebugLabel("updateTrackingState") << "Invalid ROI after update! Using emergency fallback.";
        // Create emergency fallback ROI centered on the worm with the initial ROI size
        currentRoi = QRectF(
            m_lastKnownPosition.x - m_initialRoiEdge/2,
            m_lastKnownPosition.y - m_initialRoiEdge/2,
            m_initialRoiEdge,
            m_initialRoiEdge
        );
        qDebug().noquote() << getDebugLabel("updateTrackingState") << "Emergency fallback ROI:" << currentRoi;
    }
    
    // Reset merge detection skip flag
    m_skipMergeDetectionNextFrame = false;
    
    return true;
}

// Legacy method implementations for backward compatibility
bool WormTracker::processFrameAsSingle(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentFixedSearchRoiRef_InOut)
{
    return processFrame(false, frame, sequenceFrameIndex, currentFixedSearchRoiRef_InOut);
}

bool WormTracker::processFrameAsMerged(const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentFixedSearchRoiRef_InOut)
{
    return processFrame(true, frame, sequenceFrameIndex, currentFixedSearchRoiRef_InOut);
}


void WormTracker::resumeTrackingWithAssignedTarget(const Tracking::DetectedBlob& targetBlob)
{
    if (m_currentState != Tracking::TrackerState::PausedForSplit) {
        qWarning().noquote() << getDebugLabel("resumeTrackingWithAssignedTarget") << "Called but not in PausedForSplit state. Current state:"
        << static_cast<int>(m_currentState) << ". Ignoring.";
        return;
    }

    int signedId = -1 * m_wormId ? getDirection() == TrackingDirection::Backward : m_wormId;
    int originalFrameNumber = (m_direction == TrackingDirection::Forward) ?
                                  m_videoKeyFrameNum + m_currFrameNum :
                                  m_videoKeyFrameNum - 1 - m_currFrameNum;

    qDebug().noquote() << getDebugLabel("resumeTrackingWithAssignedTarget") << "Target blob at " << targetBlob.centroid.x() << "," << targetBlob.centroid.y() << " Area:" << targetBlob.area;

    if (!targetBlob.isValid) {
        qWarning().noquote() << getDebugLabel("resumeTrackingWithAssignedTarget") << "Called with invalid blob. Transitioning to Lost.";
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

    qDebug().noquote() << getDebugLabel("resumeTrackingWithAssignedTarget") << "Resumed. New Search ROI:" << m_currentSearchRoi
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

// Helper method to generate consistent debug labels
QString WormTracker::getDebugLabel(const QString& functionName) const {
int displayId = m_wormId;
if (m_direction == TrackingDirection::Backward) {
    displayId = -m_wormId;
}
    
int frameNumber = -1;
if (m_direction == TrackingDirection::Forward) {
    frameNumber = (m_videoKeyFrameNum + m_currFrameNum);
} else {
    frameNumber = (m_videoKeyFrameNum - 1 - m_currFrameNum);
}
    
return QString("WT: %1|FN%2|%3-->").arg(displayId).arg(frameNumber).arg(functionName);
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
