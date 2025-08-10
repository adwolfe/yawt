// wormtracker.cpp
#include "wormtracker.h" // Lowercase include
#include <QDebug>
#include <QtMath>       // For qSqrt, qPow, qAbs
#include <algorithm>    // For std::sort, std::min_element etc. if needed
#include <limits>       // For std::numeric_limits
#include <QLineF>


// Constants for boundary expansion logic
const int MAX_EXPANSION_ITERATIONS_BOUNDARY = 5;  // Increased from 3 for more complete expansion
const double BOUNDARY_ROI_EXPANSION_FACTOR = 1.4; // Increased from 1.25 for larger expansion steps
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
    // Constructor debug removed
    m_lastPrimaryBlob.isValid = false; // Ensure lastPrimaryBlob starts as invalid
    m_lastFullBlob.isValid = false;    // Initialize new member
}

WormTracker::~WormTracker() {
    // Destructor debug removed
}

void WormTracker::setFrames(const std::vector<cv::Mat>* frames) {
    m_framesToProcess = frames;
}

void WormTracker::startTracking()
{
    qDebug().noquote() << getDebugLabel("startTracking") << "startTracking - STARTING TRACKING";
    if (!m_framesToProcess || m_framesToProcess->empty()) {
        qWarning().noquote() << getDebugLabel("startTracking") << "No frames";
        emit errorOccurred(m_wormId, "No frames provided to tracker.");
        emit finished();
        return;
    }

    m_trackingActive = true;
    m_currentState = Tracking::TrackerState::TrackingSingle; // Initial state when tracking starts
    m_lastPrimaryBlob.isValid = false; // Reset for a new tracking session
    m_lastFullBlob.isValid = false;    // Reset for a new tracking session
    m_currFrameNum = 0; // Reset frame counter for new tracking session

    // Start tracking debug removed

    // Start the processing loop
    QMetaObject::invokeMethod(this, &WormTracker::continueTracking, Qt::QueuedConnection);
}

void WormTracker::continueTracking()
{
    qDebug().noquote() << getDebugLabel("continueTracking") << "continueTracking - CALLED for frame:" << m_currFrameNum;
    
    if (QThread::currentThread()->isInterruptionRequested())
    {
        m_trackingActive = false; // Stop active tracking without debug message
    }

    if (m_trackingActive && m_currFrameNum < static_cast<int>(m_framesToProcess->size()))
    {
        const cv::Mat& currentFrame = (*m_framesToProcess)[m_currFrameNum];
        QRectF searchRoiForThisFrame = m_currentSearchRoi; // Capture the ROI used for *this* frame's search

        if (currentFrame.empty())
        {
            qWarning().noquote() << getDebugLabel("continueTracking") << "Empty frame";
        }
        else
        {
            bool foundTargetThisFrame = false;
            if (!m_currentSearchRoi.isValid() || m_currentSearchRoi.isEmpty()) {
                qWarning().noquote() << getDebugLabel("continueTracking") << "Invalid ROI, resetting";
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
            emit progress(m_wormId, 100);
            emit finished();
        }

    } else {
        if (m_trackingActive) {
            emit progress(m_wormId, 100);
            m_trackingActive = false;
        }
        // Finished debug removed
        emit finished();
    }
}

void WormTracker::stopTracking() {
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
        // Debug message removed - findPersistingComponent internal details
        persistingComponent = oldOverlapComponentDetails;
    }
    // If old overlap is not good, check if the new growth part is significant
    // This could happen if the worm moved quickly, and the "new growth" is actually the worm in a new spot.
    else if (newGrowthComponentDetails.isValid && newGrowthComponentDetails.area > (m_minBlobArea * significanceThresholdFactor)) {
        // Debug message removed - findPersistingComponent internal details
        persistingComponent = newGrowthComponentDetails;
    } else {
        // If neither is conclusive, it's hard to define a "persisting" part.
        // Fallback: if the currentFullBlob itself is plausibly a single worm, use that.
        // Otherwise, this function returns an invalid blob, and the caller might use the full current blob.
        if (currentFrameFullBlob.area >= m_minBlobArea && currentFrameFullBlob.area <= m_maxBlobArea * 1.2) { // Allow slightly larger for persistence
            // Debug message removed - findPersistingComponent internal details
            persistingComponent = currentFrameFullBlob;
        } else {
            // Debug message removed - findPersistingComponent internal details
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

    // debugMessage field has been removed - using getDebugLabel directly in debug statements
    context.searchRoiUsedForThisFrame = searchRoi;
    context.blobsInFixedRoi = findPlausibleBlobsInRoi(frame, searchRoi);
    context.plausibleBlobsInFixedRoi = context.blobsInFixedRoi.count();

    // Processing state debug removed

    return context;
}

// Unified frame processing - handles both single and merged tracking modes
bool WormTracker::processFrame(bool asMerged, const cv::Mat& frame, int sequenceFrameIndex, QRectF& currentRoi)
{
    qDebug().noquote() << getDebugLabel("processFrame") << "processFrame - CALLED asMerged:" << asMerged;
    
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
    // No debug needed for common case of no blobs found
    
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
    // Debug message removed - redundant ROI update info
    
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
    qDebug().noquote() << getDebugLabel("handleSingleBlobCase") << "handleSingleBlobCase - CALLED asMerged:" << asMerged << "area:" << blob.area;
    
    // Debug removed - found single blob
    
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
    qDebug().noquote() << getDebugLabel("handleNonBoundaryBlob") << "handleNonBoundaryBlob - CALLED asMerged:" << asMerged << "area:" << blob.area;
    
    // Check for split by area reduction
    bool isSplit = detectSplitByAreaReduction(blob);
    
    if (isSplit) {
        // qDebug().noquote() << getDebugLabel("handleNonBoundaryBlob") << "Potential split detected. Expanding to look for other fragments...";
        
        // Proactively expand ROI to look for other fragments, even though this blob doesn't touch the boundary
        QRectF expandedRoi = context.searchRoiUsedForThisFrame;
        
        // Apply expansion similar to expandBlobTouchingBoundary
        for (int i = 0; i < MAX_EXPANSION_ITERATIONS_BOUNDARY; ++i) {
            qreal newWidth = expandedRoi.width() * BOUNDARY_ROI_EXPANSION_FACTOR;
            qreal newHeight = expandedRoi.height() * BOUNDARY_ROI_EXPANSION_FACTOR;
            expandedRoi.setSize(QSizeF(newWidth, newHeight));
            expandedRoi.moveCenter(blob.centroid);
            
            // Ensure ROI stays within frame bounds
            expandedRoi.setX(qMax(0.0, expandedRoi.x())); 
            expandedRoi.setY(qMax(0.0, expandedRoi.y()));
            if (expandedRoi.right() > frame.cols) expandedRoi.setRight(frame.cols);
            if (expandedRoi.bottom() > frame.rows) expandedRoi.setBottom(frame.rows);
            expandedRoi.setWidth(qMin(expandedRoi.width(), static_cast<qreal>(frame.cols)));
            expandedRoi.setHeight(qMin(expandedRoi.height(), static_cast<qreal>(frame.rows)));
            
            // qDebug().noquote() << getDebugLabel("handleNonBoundaryBlob") << "Expanded ROI: " << expandedRoi;
        }
        
        // Look for all blobs in the expanded area
        QList<Tracking::DetectedBlob> splitCandidates;
        QList<Tracking::DetectedBlob> blobsInExpandedRoi = findPlausibleBlobsInRoi(frame, expandedRoi);
        
        // qDebug().noquote() << getDebugLabel("handleNonBoundaryBlob") << "Found " 
        //                   << blobsInExpandedRoi.size() << " blobs in expanded ROI";
        
        // Filter and add split candidates
        double minSplitBlobArea = m_minBlobArea * 0.5; // Half of minimum area as threshold
        for (const auto& b : blobsInExpandedRoi) {
            if (b.isValid && b.area >= minSplitBlobArea) {
                splitCandidates.append(b);
                // qDebug().noquote() << getDebugLabel("handleNonBoundaryBlob") << "Adding split candidate: Area=" 
                //                   << b.area << " BBox=" << b.boundingBox;
            }
        }
        
        // If we didn't find additional candidates, add the original blob
        if (splitCandidates.isEmpty()) {
            splitCandidates.append(blob);
            // qDebug().noquote() << getDebugLabel("handleNonBoundaryBlob") << "No additional split candidates found, using original blob.";
        }
        
        return updateTrackingState(blob, blob, splitCandidates, 
                                  Tracking::TrackerState::PausedForSplit, 
                                  context, frame.size(), currentRoi);
    } else if (!asMerged) {
        // In single mode, check for merge
        bool confirmedMerge = detectMergeByAreaIncrease(blob);
        
        if (confirmedMerge) {
            qDebug().noquote() << getDebugLabel("handleNonBoundaryBlob") << "handleNonBoundaryBlob - MERGE confirmed - Area:" << blob.area;
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
    // Debug removed - boundary expansion routine
    Tracking::DetectedBlob expandedBlob = initialBlob;
    
    // Declare blobsInExpanded outside the loop so it's accessible after the loop completes
    QList<Tracking::DetectedBlob> blobsInExpanded;
    
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
        
        // Debug message removed - expansion iteration details
        
        blobsInExpanded = findPlausibleBlobsInRoi(frame, expandedRoi);
        // Debug message removed - expanded ROI blob count
        
        if (blobsInExpanded.isEmpty()) {
            // Debug message removed - empty expanded ROI info
            break;
        }
        
        // Find closest blob to original blob's centroid
        Tracking::DetectedBlob closestInExpansion;
        closestInExpansion.isValid = false;
        double minDistToOriginal = std::numeric_limits<double>::max();
        
        for (const auto& b : blobsInExpanded) {
            if (b.isValid) {
                double d = Tracking::sqDistance(initialBlob.centroid, b.centroid);
                // Debug message removed - candidate blob details
                                  
                if (d < minDistToOriginal) {
                    minDistToOriginal = d;
                    closestInExpansion = b;
                }
            }
        }
        
        expandedBlob = closestInExpansion.isValid ? closestInExpansion : blobsInExpanded.first();
        
        // Debug message removed - selected blob details
        
        // Check if ANY blob in the expanded ROI touches the boundary
        bool anyBlobTouchesBoundary = false;
        for (const auto& b : blobsInExpanded) {
            if (b.isValid && (b.touchesROIboundary || !expandedRoi.contains(b.boundingBox))) {
                anyBlobTouchesBoundary = true;
                // Debug message removed - boundary touching blob details
            }
        }
        
        // Only stop expansion if NO blobs touch boundary or we've reached max iterations
        bool allBlobsFullyContained = !anyBlobTouchesBoundary;
        if (allBlobsFullyContained || i == MAX_EXPANSION_ITERATIONS_BOUNDARY - 1) {
            // Debug message removed - expansion completion details
            break;
        }
    }
    
    // Expanded blob details debug removed
    
    // Log information about all blobs in the final expanded ROI
    if (blobsInExpanded.size() > 1) {
        // Debug message removed - expanded ROI blob details
    }
    
    return expandedBlob;
}
// Handle a blob that touches the boundary
bool WormTracker::handleBoundaryTouchingBlob(bool asMerged, const Tracking::DetectedBlob& blob, 
                                            const cv::Mat& frame, const FrameProcessingContext& context, 
                                            QRectF& currentRoi)
{
    qDebug().noquote() << getDebugLabel("handleBoundaryTouchingBlob") << "handleBoundaryTouchingBlob - CALLED asMerged:" << asMerged << "area:" << blob.area;
    
    // Try to expand the blob by growing the ROI and re-thresholding
    // Expand ROI to get full blob
    QRectF expandedRoi = context.searchRoiUsedForThisFrame;
    Tracking::DetectedBlob expandedBlob = expandBlobTouchingBoundary(blob, expandedRoi, frame);
    
    // After expansion, expandedBlob is our best guess for the full entity
    Tracking::DetectedBlob blobToReport = expandedBlob;
    Tracking::DetectedBlob blobForAnchor;
    
    // Debug message removed - expanded blob measurements
    
    // Log previous blob info for comparison
    if (m_lastPrimaryBlob.isValid) {
        // Debug message removed - previous primary blob details
    }
    
    if (m_lastFullBlob.isValid) {
        // Debug message removed - previous full blob details
    }
    
    // Determine anchor: if previously tracked, find persisting part. Otherwise, anchor is the full blob.
    if (m_lastPrimaryBlob.isValid) {
        // Debug message removed - persisting component search message
        Tracking::DetectedBlob persisted = findPersistingComponent(m_lastPrimaryBlob, blobToReport, frame.size(), context.originalFrameNumber);
        blobForAnchor = persisted.isValid ? persisted : blobToReport;
        // Debug removed - component persistence validation
        
        if (persisted.isValid) {
            // Calculate what percentage of the full blob this represents
            double percentOfFullArea = (persisted.area / blobToReport.area) * 100.0;
            
            // Check if the persisting component seems unusually small
            if (percentOfFullArea < 20.0) {
                // Debug removed - unusually small component warning
            }
        }
    } else {
        blobForAnchor = blobToReport;
        // Debug message removed - anchor assignment info
    }
    
    // Check for potential split by area reduction
    // Debug message removed - split check details
    
    bool isSplit = detectSplitByAreaReduction(expandedBlob);
    if (isSplit) {
        // When split is detected, include ALL blobs found in expanded ROI as candidates
        QList<Tracking::DetectedBlob> splitCandidates;
        QList<Tracking::DetectedBlob> blobsInExpandedRoi = findPlausibleBlobsInRoi(frame, expandedRoi);
        
        // Filter out blobs that are too small
        double minSplitBlobArea = m_minBlobArea * 0.5; // Half of minimum area as threshold
        for (const auto& b : blobsInExpandedRoi) {
            if (b.isValid && b.area >= minSplitBlobArea) {
                splitCandidates.append(b);
                // Debug message removed - split candidate details
            }
        }
        
        qDebug().noquote() << getDebugLabel("handleBoundaryTouchingBlob") << "handleBoundaryTouchingBlob - SPLIT detected with " << splitCandidates.size() << " candidates";
        // for (int i = 0; i < splitCandidates.size(); ++i) {
        //     const auto& candidate = splitCandidates[i];
        //     qDebug().noquote() << getDebugLabel("handleBoundaryTouchingBlob") 
        //                       << "Split candidate" << i << "Area:" << candidate.area 
        //                       << "Centroid:" << candidate.centroid.x() << "," << candidate.centroid.y();
        // }
        return updateTrackingState(blobForAnchor, blobToReport, splitCandidates, 
                                 Tracking::TrackerState::PausedForSplit, context, frame.size(), currentRoi);
    }
    
    // Check for merge if in single mode
    if (!asMerged) {
        bool confirmedMerge = detectMergeByAreaIncrease(blobToReport);
        
        if (confirmedMerge) {
            qDebug().noquote() << getDebugLabel("handleBoundaryTouchingBlob") << "handleBoundaryTouchingBlob - MERGE confirmed - Area:" << blobToReport.area;
            return updateTrackingState(blobForAnchor, blobToReport, QList<Tracking::DetectedBlob>(), 
                                     Tracking::TrackerState::TrackingMerged, context, frame.size(), currentRoi);
        } else {
            // Debug message removed - no merge case details
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
    qDebug().noquote() << getDebugLabel("handleMultipleBlobsCase") << "handleMultipleBlobsCase - CALLED asMerged:" << asMerged << "blobCount:" << blobs.size();
    if (asMerged) {
        // For merged mode, multiple blobs might indicate a split
        // Multiple blobs debug removed
        
        // Initialize variables without logging each blob
        bool anyBlobTouchesBoundary = false;
        for (const auto& blob : blobs) {
            bool touchesBoundary = isBlobTouchingBoundary(blob, context.searchRoiUsedForThisFrame);
            anyBlobTouchesBoundary = anyBlobTouchesBoundary || touchesBoundary;
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
        // Multiple blobs debug removed
        
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
    qDebug().noquote() << getDebugLabel("detectSplitByAreaReduction") << "detectSplitByAreaReduction - CALLED with currentArea:" << currentBlob.area;
    
    if (!m_lastFullBlob.isValid) {
        qDebug().noquote() << getDebugLabel("detectSplitByAreaReduction") << "detectSplitByAreaReduction - No lastFullBlob, returning false";
        return false;
    }
    
    // Calculate both area metrics
    double hullAreaRatio = currentBlob.convexHullArea / m_lastFullBlob.convexHullArea;
    double regularAreaRatio = currentBlob.area / m_lastFullBlob.area;
    
    // Split threshold: Both hull area AND regular area must show significant reduction
    // Both metrics must be less than 80% of previous frame's values
    bool isSplit = (hullAreaRatio < 0.80) && (regularAreaRatio < 0.80);
    
    qDebug().noquote() << getDebugLabel("detectSplitByAreaReduction") 
                      << "detectSplitByAreaReduction - CurrentArea:" << currentBlob.area 
                      << "LastArea:" << m_lastFullBlob.area
                      << "HullRatio:" << QString::number(hullAreaRatio, 'f', 3)
                      << "RegularRatio:" << QString::number(regularAreaRatio, 'f', 3) 
                      << "IsSplit:" << isSplit;
    
    return isSplit;
}

// Detect merge by area increase
bool WormTracker::detectMergeByAreaIncrease(const Tracking::DetectedBlob& currentBlob)
{
    qDebug().noquote() << getDebugLabel("detectMergeByAreaIncrease") << "detectMergeByAreaIncrease - CALLED with currentArea:" << currentBlob.area;
    
    if (m_skipMergeDetectionNextFrame) {
        qDebug().noquote() << getDebugLabel("detectMergeByAreaIncrease") << "detectMergeByAreaIncrease - Skipping merge detection this frame";
        return false;
    }
    
    bool isMerge = false;
    if (m_lastPrimaryBlob.isValid && 
        currentBlob.area > m_lastPrimaryBlob.area * MERGE_CONFIRM_RELATIVE_AREA_FACTOR) {
        isMerge = true;
        qDebug().noquote() << getDebugLabel("detectMergeByAreaIncrease") 
                          << "detectMergeByAreaIncrease - MERGE by relative area - Current:" << currentBlob.area 
                          << "Last:" << m_lastPrimaryBlob.area 
                          << "Factor:" << MERGE_CONFIRM_RELATIVE_AREA_FACTOR;
    }
    
    if (currentBlob.area > m_maxBlobArea * MERGE_CONFIRM_ABSOLUTE_AREA_FACTOR) {
        isMerge = true;
        qDebug().noquote() << getDebugLabel("detectMergeByAreaIncrease") 
                          << "detectMergeByAreaIncrease - MERGE by absolute area - Current:" << currentBlob.area 
                          << "Max:" << m_maxBlobArea 
                          << "Factor:" << MERGE_CONFIRM_ABSOLUTE_AREA_FACTOR;
    }
    
    qDebug().noquote() << getDebugLabel("detectMergeByAreaIncrease") << "detectMergeByAreaIncrease - Result isMerge:" << isMerge;
    
    return isMerge;
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
        // Debug removed - invalid blobs case
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
        qWarning().noquote() << getDebugLabel("updateTrackingState") << "Invalid frame size";
        // Use a fallback size to prevent ROI calculation errors
        cv::Size fallbackSize(1280, 720);
        QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, fallbackSize);
        // Debug removed - fallback frame size message
        currentRoi = nextFrameSearchRoi;
    } else {
        QRectF nextFrameSearchRoi = adjustRoiPos(m_lastKnownPosition, frameSize);
        // Debug messages removed - ROI adjustment details
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
    // Debug message removed - final ROI details
    
    if (!currentRoi.isValid() || currentRoi.isEmpty()) {
        qWarning().noquote() << getDebugLabel("updateTrackingState") << "Invalid ROI";
        // Create emergency fallback ROI centered on the worm with the initial ROI size
        currentRoi = QRectF(
            m_lastKnownPosition.x - m_initialRoiEdge/2,
            m_lastKnownPosition.y - m_initialRoiEdge/2,
            m_initialRoiEdge,
            m_initialRoiEdge
        );
        // Debug removed - emergency fallback ROI message
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
        qWarning().noquote() << getDebugLabel("resumeTrackingWithAssignedTarget") << "Not in PausedForSplit state";
        return;
    }

    int signedId = -1 * m_wormId ? getDirection() == TrackingDirection::Backward : m_wormId;
    int originalFrameNumber = (m_direction == TrackingDirection::Forward) ?
                                  m_videoKeyFrameNum + m_currFrameNum :
                                  m_videoKeyFrameNum - 1 - m_currFrameNum;

    qDebug().noquote() << getDebugLabel("resumeTrackingWithAssignedTarget") 
                      << "resumeTrackingWithAssignedTarget - Resuming with target - Valid:" << targetBlob.isValid 
                      << (targetBlob.isValid ? QString(" Area:%1 Pos:%2,%3").arg(targetBlob.area).arg(targetBlob.centroid.x()).arg(targetBlob.centroid.y()) : QString());

    if (!targetBlob.isValid) {
        qWarning().noquote() << getDebugLabel("resumeTrackingWithAssignedTarget") << "Invalid blob";
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

    qDebug().noquote() << getDebugLabel("resumeTrackingWithAssignedTarget") << "resumeTrackingWithAssignedTarget - Resumed tracking as single worm";

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
        
    return QString("WT: %1|FN%2|").arg(displayId).arg(frameNumber);
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
