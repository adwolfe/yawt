/**
 * @file trackingdatastorage.cpp
 * @brief Implementation of TrackingDataStorage: centralized, single source-of-truth for items, tracks, per-frame blobs, and merge history.
 *
 * Responsibilities:
 * - Manage item lifecycle (IDs, types, colors, visibility, ROI sizing).
 * - Store/retrieve per-item tracks and maintain fast per-frame indexes.
 * - Maintain global metrics and derived fixed ROI size with a user multiplier.
 * - Record per-frame merge groups and per-frame detected blobs for overlays/analysis.
 *
 * Concurrency:
 * - Intended for GUI-thread access. Do not mutate/read from multiple threads concurrently.
 * - Updaters should funnel writes to the GUI thread (e.g., via queued signals).
 *
 * Signals:
 * - itemsChanged(...) for full item list updates, trackAdded/trackRemoved for per-item track mutations,
 *   allDataChanged for broad refresh, globalMetricsUpdated for metric/ROI changes.
 */
#include "trackingdatastorage.h"
#include <stdexcept>
#include <QtMath>
#include "../utils/loggingcategories.h"
#include "wormsjsoncodec.h"
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

// Define a default small ROI size for when no worms are present or dimensions are zero
const QSizeF DEFAULT_ROI_SIZE(20.0, 20.0);

/**
 * @brief Construct storage with initialized color palette and defaults.
 * Initializes ID counters, metrics, ROI multiplier, and supporting indexes/maps.
 */
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
    TableItems::AnnotationItem newItem;
    newItem.id = m_nextId++;
    
    newItem.color = getNextColor();
    
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
    
    YAWT_DEBUG(lcDataStorage) << "Added item ID" << newItem.id << "Original BBox:" << boundingBox;
    return newItem.id;
}

bool TrackingDataStorage::removeItem(int itemId) {
    int index = getIndexFromId(itemId);
    if (index < 0) {
        return false; // Item not found
    }
    
    // Remove from items list
    m_items.removeAt(index);
    
    purgeProcessingDataForItem(itemId);
    buildFrameIndex();
    
    updateIdToIndexMap();
    recalculateGlobalMetricsAndROIs();
    
    emit itemRemoved(itemId);
    emit allDataChanged();
    emit itemsChanged(m_items);
    
    return true;
}

bool TrackingDataStorage::hasAnyData() const {
    return !m_items.isEmpty()
        || !m_tracks.empty()
        || !m_frameIndex.isEmpty()
        || !m_mergeHistory.isEmpty()
        || !m_detectedBlobsByFrame.isEmpty()
        || !m_tipBaselines.isEmpty();
}

bool TrackingDataStorage::removeAllItems() {
    const bool hadData = hasAnyData();
    if (hadData) clearAllData();
    return hadData;
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
        // Emit the bulk itemsChanged so consumers rebuild any id->color maps
        // from the authoritative list instead of relying on a per-item signal.
        emit itemsChanged(m_items);
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

/**
 * @brief Replace or set the full track for an item and rebuild frame index.
 * Emits trackRemoved/trackAdded appropriately and signals allDataChanged/itemsChanged for UI/model refresh.
 */
void TrackingDataStorage::setTrackForWorm(int wormId, const std::vector<Tracking::WormTrackPoint>& trackPoints) {
    // Check if item exists
    if (getIndexFromId(wormId) < 0) {
        YAWT_WARN(lcDataStorage) << "Tried to set track for non-existent item ID" << wormId;
        return;
    }
    
    bool isNewTrack = !m_tracks.count(wormId);
    m_tracks[wormId] = trackPoints;

    // Trackers supply position/ROI/quality only; join in the blob-derived
    // geometry so the track is complete in memory, not just once serialized.
    refreshDerivedTrackData(wormId);

    // Rebuild frame index for fast lookups
    buildFrameIndex();
    
    if (isNewTrack) {
        emit trackAdded(wormId);
    } else {
        emit trackRemoved(wormId); // Remove old track
        emit trackAdded(wormId);   // Add new track
    }
    
    emit allDataChanged();
    emit itemsChanged(m_items);
}

void TrackingDataStorage::clearTrackForWorm(int wormId) {
    if (m_tracks.erase(wormId)) {
        // Rebuild frame index after removing track
        buildFrameIndex();
        emit trackRemoved(wormId);
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

void TrackingDataStorage::clearAllData() {
    QList<int> removedItemIds;
    for (const auto& item : m_items) {
        removedItemIds.append(item.id);
    }
    QList<int> removedTrackIds;
    for (const auto& track : m_tracks) {
        removedTrackIds.append(track.first);
    }

    m_items.clear();
    m_tracks.clear();
    m_mergeHistory.clear();
    m_detectedBlobsByFrame.clear();
    m_tipBaselines.clear();
    m_frameIndex.clear();
    m_idToIndexMap.clear();
    m_nextId = 1;
    m_currentColorIndex = 0;

    recalculateGlobalMetricsAndROIs();

    for (int id : removedItemIds) {
        emit itemRemoved(id);
    }
    for (int id : removedTrackIds) {
        emit trackRemoved(id);
    }

    emit allDataChanged();
    emit itemsChanged(m_items);
}

void TrackingDataStorage::purgeProcessingDataForItem(int itemId) {
    const bool hadTrack = (m_tracks.erase(itemId) > 0);

    auto mergeIt = m_mergeHistory.begin();
    while (mergeIt != m_mergeHistory.end()) {
        QList<QList<int>>& groups = mergeIt.value();
        for (auto groupIt = groups.begin(); groupIt != groups.end();) {
            groupIt->removeAll(itemId);
            if (groupIt->size() < 2) {
                groupIt = groups.erase(groupIt);
            } else {
                ++groupIt;
            }
        }
        if (groups.isEmpty()) {
            mergeIt = m_mergeHistory.erase(mergeIt);
        } else {
            ++mergeIt;
        }
    }

    auto frameIt = m_detectedBlobsByFrame.begin();
    while (frameIt != m_detectedBlobsByFrame.end()) {
        frameIt.value().remove(itemId);
        if (frameIt.value().isEmpty()) {
            frameIt = m_detectedBlobsByFrame.erase(frameIt);
        } else {
            ++frameIt;
        }
    }

    m_tipBaselines.remove(itemId);

    if (hadTrack) {
        emit trackRemoved(itemId);
    }
}

static bool isRoiPointType(TableItems::ItemType type) {
    return type == TableItems::ItemType::Region ||
           type == TableItems::ItemType::StartPoint ||
           type == TableItems::ItemType::EndPoint ||
           type == TableItems::ItemType::CenterPoint;
}

bool TrackingDataStorage::loadFromWormsJson(const QString& filePath) {
    WormsJson::Document doc;
    QString error;
    if (!WormsJson::read(filePath, doc, &error)) {
        qWarning() << "TrackingDataStorage: cannot read worms.json:" << filePath << error;
        return false;
    }

    clearAllData();

    bool hasMetrics = false;
    if (doc.metrics.valid) {
        m_roiSizeMultiplier      = doc.metrics.roiSizeMultiplier;
        m_minObservedArea        = doc.metrics.minObservedArea;
        m_maxObservedArea        = doc.metrics.maxObservedArea;
        m_minObservedAspectRatio = doc.metrics.minObservedAspectRatio;
        m_maxObservedAspectRatio = doc.metrics.maxObservedAspectRatio;
        if (doc.metrics.fixedRoiSize.isValid()) {
            m_currentFixedRoiSize = doc.metrics.fixedRoiSize;
            hasMetrics = true;
        }
    }

    if (!doc.items.isEmpty()) {
        int maxId = 0;
        for (const TableItems::AnnotationItem& item : doc.items) {
            maxId = qMax(maxId, item.id);
            m_items.append(item);
        }
        m_nextId = maxId + 1;
    }

    m_tracks = std::move(doc.tracks);
    m_detectedBlobsByFrame = std::move(doc.blobsByFrame);
    m_mergeHistory = std::move(doc.mergeGroupsByFrame);
    m_tipBaselines = std::move(doc.tipBaselines);

    refreshDerivedTrackData();
    updateIdToIndexMap();
    buildFrameIndex();

    if (!hasMetrics) {
        recalculateGlobalMetricsAndROIs();
    } else {
        emit globalMetricsUpdated(m_minObservedArea, m_maxObservedArea,
                                  m_minObservedAspectRatio, m_maxObservedAspectRatio,
                                  m_currentFixedRoiSize);
    }

    emit allDataChanged();
    emit itemsChanged(m_items);
    return true;
}

bool TrackingDataStorage::loadFromRoiJson(const QString& filePath) {
    if (!QFile::exists(filePath)) {
        qWarning() << "TrackingDataStorage: Cannot read roi_points.json:" << filePath;
        return false;
    }

    QSet<int> existingIds;
    for (const auto& item : std::as_const(m_items)) {
        existingIds.insert(item.id);
    }

    for (TableItems::AnnotationItem item : WormsJson::readRoiPoints(filePath)) {
        if (!isRoiPointType(item.type)) continue;

        if (item.id <= 0 || existingIds.contains(item.id)) {
            item.id = m_nextId++;
        } else if (item.id >= m_nextId) {
            m_nextId = item.id + 1;
        }
        existingIds.insert(item.id);
        m_items.append(item);
    }

    updateIdToIndexMap();
    recalculateGlobalMetricsAndROIs();
    emit allDataChanged();
    emit itemsChanged(m_items);
    return true;
}

/**
 * @brief Aggressively clear tracks and compact memory.
 * Use when large runs should release memory immediately after save/cancel/fail.
 */
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
    
    YAWT_INFO(lcDataStorage) << "Cleared and compacted" << trackCount << "track datasets, memory deallocated";
    emit allDataChanged();
}

// --- Merge History Methods ---

/**
 * @brief Persist per-frame conceptual merge groups (for overlays and post-run analysis).
 * Each group is a list of conceptual worm IDs present in the same shared blob at that frame.
 */
void TrackingDataStorage::setMergeGroupsForFrame(int frameNumber, const QList<QList<int>>& groups) {
    if (frameNumber < 0) return; // silently ignore invalid frame numbers
    m_mergeHistory.insert(frameNumber, groups);
}

QList<QList<int>> TrackingDataStorage::getMergeGroupsForFrame(int frameNumber) const {
    return m_mergeHistory.value(frameNumber);
}

QMap<int, QList<QList<int>>> TrackingDataStorage::getAllMergeGroups() const {
    return m_mergeHistory;
}

// --- Detected blob persistence API ---

/**
 * @brief Record a DetectedBlob for a worm at a given frame (latest wins).
 * Enables per-frame overlay reconstruction and debugging of merge/split decisions.
 */
void TrackingDataStorage::setDetectedBlobForFrame(int frameNumber, int wormId, const Tracking::DetectedBlob& blob) {
    if (frameNumber < 0) return;
    m_detectedBlobsByFrame[frameNumber].insert(wormId, blob);
}

QMap<int, Tracking::DetectedBlob> TrackingDataStorage::getDetectedBlobsForFrame(int frameNumber) const {
    return m_detectedBlobsByFrame.value(frameNumber);
}

const Tracking::DetectedBlob* TrackingDataStorage::findDetectedBlob(int frameNumber, int wormId) const {
    const auto frameIt = m_detectedBlobsByFrame.constFind(frameNumber);
    if (frameIt == m_detectedBlobsByFrame.constEnd()) return nullptr;
    const auto wormIt = frameIt->constFind(wormId);
    if (wormIt == frameIt->constEnd()) return nullptr;
    return &wormIt.value();
}

void TrackingDataStorage::applyBlobDerivedFields(Tracking::WormTrackPoint& point,
                                                 const Tracking::DetectedBlob& blob) const {
    if (blob.area > 0.0)
        point.area = static_cast<float>(blob.area);

    const QRectF& box = blob.boundingBox;
    if (box.width() > 0.0 && box.height() > 0.0) {
        const double ratio = box.width() / box.height();
        point.aspectRatio = static_cast<float>(ratio < 1.0 ? 1.0 / ratio : ratio);
    }

    if (blob.centerline.points.size() >= 2) {
        double arcLength = 0.0;
        for (size_t i = 1; i < blob.centerline.points.size(); ++i) {
            const cv::Point2f d = blob.centerline.points[i] - blob.centerline.points[i - 1];
            arcLength += std::sqrt(d.x * d.x + d.y * d.y);
        }
        point.bodyLength = static_cast<float>(arcLength);
    }

    // Head/tail is authoritative when a blob exists: the centerline pass may
    // have swapped or withdrawn an assignment, and that has to show through.
    const int tipCount = static_cast<int>(blob.centerline.tipCandidates.size());
    const bool hasHead = blob.centerline.headTipIdx >= 0 && blob.centerline.headTipIdx < tipCount;
    const bool hasTail = blob.centerline.tailTipIdx >= 0 && blob.centerline.tailTipIdx < tipCount;
    if (hasHead && hasTail) {
        point.headTip = blob.centerline.tipCandidates[blob.centerline.headTipIdx].point;
        point.tailTip = blob.centerline.tipCandidates[blob.centerline.tailTipIdx].point;
        point.hasTips = true;
    } else {
        point.hasTips = false;
    }
}

void TrackingDataStorage::refreshDerivedTrackData(int onlyWormId) {
    if (m_detectedBlobsByFrame.isEmpty()) return;

    for (auto& entry : m_tracks) {
        const int wormId = entry.first;
        if (onlyWormId >= 0 && wormId != onlyWormId) continue;

        for (Tracking::WormTrackPoint& point : entry.second) {
            if (const Tracking::DetectedBlob* blob =
                    findDetectedBlob(point.frameNumber, wormId)) {
                applyBlobDerivedFields(point, *blob);
            }
        }
    }
}

// --- Per-worm tip-feature baselines (Phase A) ---

void TrackingDataStorage::recordTipFeatureSample(int wormId, float curvatureMagnitude, float width) {
    Centerline::TipFeatureBaseline& baseline = m_tipBaselines[wormId];
    baseline.addCurvatureSample(curvatureMagnitude);
    baseline.addWidthSample(width);
}

void TrackingDataStorage::recordBodyLengthSample(int wormId, float length) {
    m_tipBaselines[wormId].addLengthSample(length);
}

Centerline::TipFeatureBaseline TrackingDataStorage::getTipBaseline(int wormId) const {
    return m_tipBaselines.value(wormId);
}

QMap<int, Centerline::TipFeatureBaseline> TrackingDataStorage::getAllTipBaselines() const {
    return m_tipBaselines;
}

void TrackingDataStorage::clearAllTipBaselines() {
    m_tipBaselines.clear();
}


// --- Data Access Methods ---

const QList<TableItems::AnnotationItem>& TrackingDataStorage::getAllItems() const {
    return m_items;
}

const TableItems::AnnotationItem* TrackingDataStorage::getItem(int itemId) const {
    int index = getIndexFromId(itemId);
    if (index < 0 || index >= m_items.count()) {
        return nullptr; // Item not found or index out of range
    }
    return &m_items[index];
}

const TableItems::AnnotationItem& TrackingDataStorage::getItemByIndex(int index) const {
    if (index < 0 || index >= m_items.count()) {
        throw std::out_of_range("Index out of range in TrackingDataStorage::getItemByIndex");
    }
    return m_items.at(index);
}

const Tracking::AllWormTracks& TrackingDataStorage::getAllTracks() const {
    return m_tracks;
}

bool TrackingDataStorage::getTrackPointQuality(int wormId, int frameNumber, Tracking::TrackPointQuality& outQuality) const {
    auto wormIndexIt = m_frameIndex.find(wormId);
    if (wormIndexIt == m_frameIndex.end()) return false;
    auto frameIt = wormIndexIt.value().find(frameNumber);
    if (frameIt == wormIndexIt.value().end()) return false;
    const Tracking::WormTrackPoint* tp = frameIt.value();
    if (!tp) return false;
    outQuality = tp->quality;
    return true;
}

QSet<int> TrackingDataStorage::getAllItemIds() const {
    QSet<int> ids;
    for (const auto& item : m_items) {
        ids.insert(item.id);
    }
    return ids;
}

QSet<int> TrackingDataStorage::getWormsWithTracks() const {
    QSet<int> ids;
    for (const auto& track : m_tracks) {
        ids.insert(track.first);
    }
    return ids;
}

bool TrackingDataStorage::getWormDataForFrame(int wormId, int frameNumber, QPointF& outPosition, QRectF& outSearchWindow) const {
    // First, check if we can get the initial position from the AnnotationItem (for keyframe)
    const TableItems::AnnotationItem* item = getItem(wormId);
    if (item && item->frameOfSelection == frameNumber) {
        outPosition = item->initialCentroid;
        outSearchWindow = item->initialBoundingBox;
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
            outSearchWindow = trackPoint->searchWindow;
            return true;
        }
    }
    
    // If we still have the item data but no specific frame match, and we're close to the keyframe,
    // use the initial position as fallback
    if (item && qAbs(frameNumber - item->frameOfSelection) <= 1) {
        outPosition = item->initialCentroid;
        outSearchWindow = item->initialBoundingBox;
        return true;
    }
    
    return false;  // Worm not found for this frame
}

bool TrackingDataStorage::getLastKnownPositionBefore(int wormId, int beforeFrame, QPointF& outPosition, QRectF& outSearchWindow) const {
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
                outSearchWindow = trackPoint->searchWindow;
                return true;
            }
        }
    }
    
    // If no valid tracking data found, try to use initial position from AnnotationItem
    const TableItems::AnnotationItem* item = getItem(wormId);
    if (item) {
        outPosition = item->initialCentroid;
        outSearchWindow = item->initialBoundingBox;
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
            lostFrames.insert(point.frameNumber);
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
    YAWT_DEBUG(lcDataStorage) << "ID-to-index map updated with" << m_idToIndexMap.size() << "entries";
}

void TrackingDataStorage::recalculateGlobalMetricsAndROIs() {
    double newMinArea = std::numeric_limits<double>::max();
    double newMaxArea = 0.0;
    double newMinAspectRatio = std::numeric_limits<double>::max();
    double newMaxAspectRatio = 0.0;
    double maxObservedDimensionL = 0.0;
    int wormCount = 0;

    for (const TableItems::AnnotationItem &item : std::as_const(m_items)) {
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
    for (TableItems::AnnotationItem &item : m_items) {
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
        YAWT_INFO(lcDataStorage) << "Global metrics updated."
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
            frameMap[trackPoint.frameNumber] = &trackPoint;
        }
    
        m_frameIndex[wormId] = frameMap;
    }

    YAWT_DEBUG(lcDataStorage) << "Built frame index for" << m_tracks.size() << "worms";
}
