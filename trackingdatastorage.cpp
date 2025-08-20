#include "trackingdatastorage.h"
#include <stdexcept>
#include <QtMath>

// Define a default small ROI size for when no worms are present or dimensions are zero
const QSizeF DEFAULT_ROI_SIZE(20.0, 20.0);

TrackingDataStorage::TrackingDataStorage(QObject *parent)
    : QObject(parent),
      m_nextId(1),
      m_currentColorIndex(0),
      m_minObservedArea(std::numeric_limits<double>::max()),
      m_maxObservedArea(0.0),
      m_minObservedAspectRatio(std::numeric_limits<double>::max()),
      m_maxObservedAspectRatio(0.0),
      m_currentFixedRoiSize(DEFAULT_ROI_SIZE),
      m_roiSizeMultiplier(1.5) // Default ROI size multiplier
{
    initializeColors();
}

void TrackingDataStorage::initializeColors() {
    m_predefinedColors
        << QColor(0, 63, 92, 255).lighter(120)
        << QColor(47, 75, 124, 255).lighter(120)
        << QColor(102, 81, 145, 255).lighter(120)
        << QColor(160, 81, 149, 255).lighter(120)
        << QColor(212, 80, 135, 255).lighter(120)
        << QColor(249, 93, 106, 255).lighter(120)
        << QColor(255, 124, 67, 255).lighter(120)
        << QColor(255, 166, 0, 255).lighter(120);
    // Add more distinct colors if needed
}

QColor TrackingDataStorage::getNextColor() {
    if (m_predefinedColors.isEmpty()) {
        return QColor(Qt::gray); // Fallback
    }
    QColor color = m_predefinedColors.at(m_currentColorIndex);
    m_currentColorIndex = (m_currentColorIndex + 1) % m_predefinedColors.count();
    return color;
}

// --- Item Management Methods ---

int TrackingDataStorage::addItem(const QPointF& centroid, const QRectF& boundingBox, int frameNumber, TableItems::ItemType type) {
    TableItems::ClickedItem newItem;
    newItem.id = m_nextId++;
    
    // Assign special colors for Fix type blobs
    if (type == TableItems::ItemType::Fix) {
        newItem.color = QColor(255, 0, 0, 180); // Semi-transparent red for Fix blobs
    } else {
        newItem.color = getNextColor();
    }
    
    newItem.type = type;
    newItem.initialCentroid = centroid;
    newItem.originalClickedBoundingBox = boundingBox;
    newItem.frameOfSelection = frameNumber;
    newItem.visible = true;
    
    m_items.append(newItem);
    updateIdToIndexMap();
    
    // Recalculate global metrics and update all item ROIs
    recalculateGlobalMetricsAndROIs();
    
    emit itemAdded(newItem.id);
    emit allDataChanged();
    emit itemsChanged(m_items);
    
    qDebug() << "TrackingDataStorage: Added item ID" << newItem.id << "Original BBox:" << boundingBox;
    return newItem.id;
}

bool TrackingDataStorage::removeItem(int itemId) {
    int index = getIndexFromId(itemId);
    if (index < 0) {
        return false; // Item not found
    }
    
    // Remove from items list
    m_items.removeAt(index);
    
    // Remove any associated track data
    if (m_tracks.count(itemId)) {
        m_tracks.erase(itemId);
        emit trackRemoved(itemId);
    }
    
    updateIdToIndexMap();
    recalculateGlobalMetricsAndROIs();
    
    emit itemRemoved(itemId);
    emit allDataChanged();
    emit itemsChanged(m_items);
    
    return true;
}

bool TrackingDataStorage::removeAllItems() {
    if (m_items.isEmpty()) {
        return false; // Already empty
    }
    
    // Gather IDs of removed items for signals
    QList<int> removedIds;
    for (const auto& item : m_items) {
        removedIds.append(item.id);
    }
    
    // Clear all data
    m_items.clear();
    m_tracks.clear();
    m_idToIndexMap.clear();
    m_nextId = 1; // Reset ID counter
    
    // Update metrics with empty state
    recalculateGlobalMetricsAndROIs();
    
    // Emit signals for removed items and tracks
    for (int id : removedIds) {
        emit itemRemoved(id);
        emit trackRemoved(id);
    }
    
    emit allDataChanged();
    emit itemsChanged(m_items);
    return true;
}

void TrackingDataStorage::setItemVisibility(int itemId, bool visible) {
    int index = getIndexFromId(itemId);
    if (index < 0) {
        return; // Item not found
    }
    
    if (m_items[index].visible != visible) {
        m_items[index].visible = visible;
        emit itemVisibilityChanged(itemId, visible);
        emit itemChanged(itemId);
    }
}

void TrackingDataStorage::setAllItemsVisibility(bool visible) {
    bool anyChanged = false;
    
    for (int i = 0; i < m_items.size(); ++i) {
        if (m_items[i].visible != visible) {
            m_items[i].visible = visible;
            emit itemVisibilityChanged(m_items[i].id, visible);
            anyChanged = true;
        }
    }
    
    if (anyChanged) {
        emit allDataChanged();
        emit itemsChanged(m_items);
    }
}

void TrackingDataStorage::setItemColor(int itemId, const QColor& color) {
    int index = getIndexFromId(itemId);
    if (index < 0) {
        return; // Item not found
    }
    
    if (m_items[index].color != color) {
        m_items[index].color = color;
        emit itemColorChanged(itemId, color);
        emit itemChanged(itemId);
    }
}

void TrackingDataStorage::setItemType(int itemId, TableItems::ItemType type) {
    int index = getIndexFromId(itemId);
    if (index < 0) {
        return; // Item not found
    }
    
    if (m_items[index].type != type) {
        m_items[index].type = type;
        recalculateGlobalMetricsAndROIs(); // Type changes may affect metrics (especially for worm types)
        emit itemChanged(itemId);
    }
}

void TrackingDataStorage::setRoiSizeMultiplier(double multiplier) {
    if (!qFuzzyCompare(m_roiSizeMultiplier, multiplier)) {
        m_roiSizeMultiplier = multiplier;
        recalculateGlobalMetricsAndROIs();
    }
}

// --- Track Management Methods ---

void TrackingDataStorage::setTrackForItem(int itemId, const std::vector<Tracking::WormTrackPoint>& trackPoints) {
    // Check if item exists
    if (getIndexFromId(itemId) < 0) {
        qWarning() << "TrackingDataStorage: Tried to set track for non-existent item ID" << itemId;
        return;
    }
    
    bool isNewTrack = !m_tracks.count(itemId);
    m_tracks[itemId] = trackPoints;
    
    // Rebuild frame index for fast lookups
    buildFrameIndex();
    
    if (isNewTrack) {
        emit trackAdded(itemId);
    } else {
        emit trackRemoved(itemId); // Remove old track
        emit trackAdded(itemId);   // Add new track
    }
    
    emit allDataChanged();
    emit itemsChanged(m_items);
}

void TrackingDataStorage::clearTrackForItem(int itemId) {
    if (m_tracks.erase(itemId)) {
        // Rebuild frame index after removing track
        buildFrameIndex();
        emit trackRemoved(itemId);
        emit allDataChanged();
        emit itemsChanged(m_items);
    }
}

void TrackingDataStorage::clearAllTracks() {
    if (m_tracks.empty()) {
        return; // No tracks to clear
    }
    
    // Gather IDs of removed tracks for signals
    QList<int> removedTrackIds;
    for (const auto& track : m_tracks) {
        removedTrackIds.append(track.first);
    }
    
    m_tracks.clear();
    
    // Emit signals for all removed tracks
    for (int id : removedTrackIds) {
        emit trackRemoved(id);
    }
    
    // Clear frame index
    m_frameIndex.clear();
    
    emit allDataChanged();
}

void TrackingDataStorage::clearAndCompactTrackData() {
    // Get count before clearing for reporting
    int trackCount = m_tracks.size();
    
    // Aggressively clear and compact track data
    m_tracks.clear();
    Tracking::AllWormTracks().swap(m_tracks); // Force memory deallocation
    
    // Clear frame index
    m_frameIndex.clear();
    
    // Also compact other related data structures
    QMap<int, int>().swap(m_idToIndexMap);
    updateIdToIndexMap(); // Rebuild the map
    
    qDebug() << "TrackingDataStorage: Cleared and compacted" << trackCount << "track datasets, memory deallocated";
    emit allDataChanged();
}

// --- Merge History Methods ---

void TrackingDataStorage::setMergeGroupsForFrame(int frameNumber, const QList<QList<int>>& groups) {
    if (frameNumber < 0) return; // silently ignore invalid frame numbers
    m_mergeHistory.insert(frameNumber, groups);
}

QList<QList<int>> TrackingDataStorage::getMergeGroupsForFrame(int frameNumber) const {
    return m_mergeHistory.value(frameNumber);
}


// --- Data Access Methods ---

const QList<TableItems::ClickedItem>& TrackingDataStorage::getAllItems() const {
    return m_items;
}

const TableItems::ClickedItem* TrackingDataStorage::getItem(int itemId) const {
    int index = getIndexFromId(itemId);
    if (index < 0 || index >= m_items.count()) {
        return nullptr; // Item not found or index out of range
    }
    return &m_items[index];
}

const TableItems::ClickedItem& TrackingDataStorage::getItemByIndex(int index) const {
    if (index < 0 || index >= m_items.count()) {
        throw std::out_of_range("Index out of range in TrackingDataStorage::getItemByIndex");
    }
    return m_items.at(index);
}

const Tracking::AllWormTracks& TrackingDataStorage::getAllTracks() const {
    return m_tracks;
}

QSet<int> TrackingDataStorage::getAllItemIds() const {
    QSet<int> ids;
    for (const auto& item : m_items) {
        ids.insert(item.id);
    }
    return ids;
}

QSet<int> TrackingDataStorage::getItemsWithTracks() const {
    QSet<int> ids;
    for (const auto& track : m_tracks) {
        ids.insert(track.first);
    }
    return ids;
}

bool TrackingDataStorage::getWormDataForFrame(int wormId, int frameNumber, QPointF& outPosition, QRectF& outRoi) const {
    // First, check if we can get the initial position from the ClickedItem (for keyframe)
    const TableItems::ClickedItem* item = getItem(wormId);
    if (item && item->frameOfSelection == frameNumber) {
        outPosition = item->initialCentroid;
        outRoi = item->initialBoundingBox;
        return true;
    }
    
    // Try to get from tracking data using frame index for O(1) lookup
    auto wormIndexIt = m_frameIndex.find(wormId);
    if (wormIndexIt != m_frameIndex.end()) {
        auto frameIt = wormIndexIt.value().find(frameNumber);
        if (frameIt != wormIndexIt.value().end()) {
            const Tracking::WormTrackPoint* trackPoint = frameIt.value();
            // Don't return data for lost tracking points
            if (trackPoint->quality == Tracking::TrackPointQuality::Lost) {
                return false;
            }
            // Convert cv::Point2f to QPointF
            outPosition = QPointF(trackPoint->position.x, trackPoint->position.y);
            outRoi = trackPoint->roi;
            return true;
        }
    }
    
    // If we still have the item data but no specific frame match, and we're close to the keyframe,
    // use the initial position as fallback
    if (item && qAbs(frameNumber - item->frameOfSelection) <= 1) {
        outPosition = item->initialCentroid;
        outRoi = item->initialBoundingBox;
        return true;
    }
    
    return false;  // Worm not found for this frame
}

bool TrackingDataStorage::getLastKnownPositionBefore(int wormId, int beforeFrame, QPointF& outPosition, QRectF& outRoi) const {
    // Check if we have tracking data for this worm
    auto wormIndexIt = m_frameIndex.find(wormId);
    if (wormIndexIt == m_frameIndex.end()) {
        return false;  // No tracking data for this worm
    }
    
    const auto& frameMap = wormIndexIt.value();
    
    // Search backwards from beforeFrame-1 to find the last valid position
    for (int frame = beforeFrame - 1; frame >= 0; frame--) {
        auto frameIt = frameMap.find(frame);
        if (frameIt != frameMap.end()) {
            const Tracking::WormTrackPoint* trackPoint = frameIt.value();
            // Only return positions with good tracking quality (not Lost)
            if (trackPoint->quality != Tracking::TrackPointQuality::Lost) {
                outPosition = QPointF(trackPoint->position.x, trackPoint->position.y);
                outRoi = trackPoint->roi;
                return true;
            }
        }
    }
    
    // If no valid tracking data found, try to use initial position from ClickedItem
    const TableItems::ClickedItem* item = getItem(wormId);
    if (item) {
        outPosition = item->initialCentroid;
        outRoi = item->initialBoundingBox;
        return true;
    }
    
    return false;  // No valid position found
}

QSet<int> TrackingDataStorage::getLostTrackingFrames(int wormId) const {
    QSet<int> lostFrames;
    
    // Check if we have tracking data for this worm
    auto trackIt = m_tracks.find(wormId);
    if (trackIt == m_tracks.end()) {
        return lostFrames; // No tracking data for this worm
    }
    
    const std::vector<Tracking::WormTrackPoint>& trackPoints = trackIt->second;
    for (const auto& point : trackPoints) {
        if (point.quality == Tracking::TrackPointQuality::Lost) {
            lostFrames.insert(point.frameNumberOriginal);
        }
    }
    
    return lostFrames;
}

QList<QPair<int, int>> TrackingDataStorage::getLostTrackingSegments(int wormId) const {
    QList<QPair<int, int>> segments;
    QSet<int> lostFrames = getLostTrackingFrames(wormId);
    
    if (lostFrames.isEmpty()) {
        return segments;
    }
    
    // Convert set to sorted list for processing
    QList<int> sortedLostFrames = lostFrames.values();
    std::sort(sortedLostFrames.begin(), sortedLostFrames.end());
    
    // Group consecutive frame numbers into segments
    int segmentStart = sortedLostFrames.first();
    int segmentEnd = segmentStart;
    
    for (int i = 1; i < sortedLostFrames.size(); ++i) {
        int currentFrame = sortedLostFrames[i];
        
        if (currentFrame == segmentEnd + 1) {
            // Consecutive frame, extend current segment
            segmentEnd = currentFrame;
        } else {
            // Gap found, close current segment and start new one
            segments.append(qMakePair(segmentStart, segmentEnd));
            segmentStart = currentFrame;
            segmentEnd = currentFrame;
        }
    }
    
    // Don't forget the last segment
    segments.append(qMakePair(segmentStart, segmentEnd));
    
    return segments;
}

int TrackingDataStorage::getItemCount() const {
    return m_items.count();
}

int TrackingDataStorage::getIndexFromId(int itemId) const {
    // Use the map for fast lookup, return -1 if not found
    return m_idToIndexMap.value(itemId, -1);
}

QSizeF TrackingDataStorage::getCurrentFixedRoiSize() const {
    return m_currentFixedRoiSize;
}

double TrackingDataStorage::getRoiSizeMultiplier() const {
    return m_roiSizeMultiplier;
}

double TrackingDataStorage::getMinObservedArea() const {
    return m_minObservedArea;
}

double TrackingDataStorage::getMaxObservedArea() const {
    return m_maxObservedArea;
}

double TrackingDataStorage::getMinObservedAspectRatio() const {
    return m_minObservedAspectRatio;
}

double TrackingDataStorage::getMaxObservedAspectRatio() const {
    return m_maxObservedAspectRatio;
}

// --- Private Helper Methods ---

void TrackingDataStorage::updateIdToIndexMap() {
    m_idToIndexMap.clear();
    for (int i = 0; i < m_items.count(); ++i) {
        m_idToIndexMap[m_items[i].id] = i;
    }
    qDebug() << "TrackingDataStorage: ID-to-index map updated with" << m_idToIndexMap.size() << "entries";
}

void TrackingDataStorage::recalculateGlobalMetricsAndROIs() {
    double newMinArea = std::numeric_limits<double>::max();
    double newMaxArea = 0.0;
    double newMinAspectRatio = std::numeric_limits<double>::max();
    double newMaxAspectRatio = 0.0;
    double maxObservedDimensionL = 0.0;
    int wormCount = 0;

    for (const TableItems::ClickedItem &item : std::as_const(m_items)) {
        if (item.type == TableItems::ItemType::Worm) {
            wormCount++;
            const QRectF& originalBox = item.originalClickedBoundingBox;
            if (originalBox.isValid() && originalBox.width() > 0 && originalBox.height() > 0) {
                double area = originalBox.width() * originalBox.height();
                newMinArea = qMin(newMinArea, area);
                newMaxArea = qMax(newMaxArea, area);

                double w = originalBox.width();
                double h = originalBox.height();
                double aspectRatio = (w > h) ? (w / h) : (h / w); // Ensure aspect ratio >= 1
                if (h == 0 && w == 0) aspectRatio = 1.0; // Avoid division by zero for zero-size box
                else if (h == 0 || w == 0) aspectRatio = std::numeric_limits<double>::max(); // Or some large number for degenerate cases

                newMinAspectRatio = qMin(newMinAspectRatio, aspectRatio);
                newMaxAspectRatio = qMax(newMaxAspectRatio, aspectRatio);

                maxObservedDimensionL = qMax(maxObservedDimensionL, qMax(w, h));
            }
        }
    }

    // If no worms, reset metrics to defaults
    if (wormCount == 0) {
        newMinArea = 0.0; // Or some other sensible default
        newMaxArea = 0.0;
        newMinAspectRatio = 1.0; // Aspect ratio of 1 for a square
        newMaxAspectRatio = 1.0;
        maxObservedDimensionL = 0.0; // This will lead to DEFAULT_ROI_SIZE
    }

    // Update stored metrics if they changed
    bool metricsChanged = false;
    if (!qFuzzyCompare(m_minObservedArea, newMinArea) ||
        !qFuzzyCompare(m_maxObservedArea, newMaxArea) ||
        !qFuzzyCompare(m_minObservedAspectRatio, newMinAspectRatio) ||
        !qFuzzyCompare(m_maxObservedAspectRatio, newMaxAspectRatio)) {
        metricsChanged = true;
    }

    m_minObservedArea = newMinArea;
    m_maxObservedArea = newMaxArea;
    m_minObservedAspectRatio = newMinAspectRatio;
    m_maxObservedAspectRatio = newMaxAspectRatio;

    QSizeF newFixedRoiSize;
    if (maxObservedDimensionL > 0) {
        double sideLength = maxObservedDimensionL * m_roiSizeMultiplier;
        newFixedRoiSize = QSizeF(sideLength, sideLength);
    } else {
        newFixedRoiSize = DEFAULT_ROI_SIZE;
    }

    if (m_currentFixedRoiSize != newFixedRoiSize) {
        metricsChanged = true; // Also consider ROI size change as a metric change
        m_currentFixedRoiSize = newFixedRoiSize;
    }

    // Update initialBoundingBox for all items
    bool itemROIsChanged = false;
    for (TableItems::ClickedItem &item : m_items) {
        QRectF oldItemRoi = item.initialBoundingBox;
        QPointF center = item.initialCentroid;
        double w = m_currentFixedRoiSize.width();
        double h = m_currentFixedRoiSize.height();
        item.initialBoundingBox = QRectF(center.x() - w / 2.0,
                                        center.y() - h / 2.0,
                                        w, h);
        if (item.initialBoundingBox != oldItemRoi) {
            itemROIsChanged = true;
        }
    }

    // Emit signals
    if (metricsChanged) {
        qDebug() << "TrackingDataStorage: Global metrics updated."
                << "Area (min/max):" << m_minObservedArea << "/" << m_maxObservedArea
                << "Aspect (min/max):" << m_minObservedAspectRatio << "/" << m_maxObservedAspectRatio
                << "Fixed ROI Size:" << m_currentFixedRoiSize;
        emit globalMetricsUpdated(m_minObservedArea, m_maxObservedArea,
                                m_minObservedAspectRatio, m_maxObservedAspectRatio,
                                m_currentFixedRoiSize);
    }

    if (itemROIsChanged || metricsChanged) {
        emit allDataChanged();
        emit itemsChanged(m_items);
    }
}

void TrackingDataStorage::buildFrameIndex() {
    // Clear existing index
    m_frameIndex.clear();

    // Build new index: wormId -> frameNumber -> trackPoint pointer
    for (const auto& trackPair : m_tracks) {
        int wormId = trackPair.first;
        const std::vector<Tracking::WormTrackPoint>& trackPoints = trackPair.second;
    
        QMap<int, const Tracking::WormTrackPoint*> frameMap;
        for (const auto& trackPoint : trackPoints) {
            frameMap[trackPoint.frameNumberOriginal] = &trackPoint;
        }
    
        m_frameIndex[wormId] = frameMap;
    }

    qDebug() << "TrackingDataStorage: Built frame index for" << m_tracks.size() << "worms";
}