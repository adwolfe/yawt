#ifndef TRACKINGDATASTORAGE_H
#define TRACKINGDATASTORAGE_H

#include <QObject>
#include <QList>
#include <QMap>
#include <QSet>
#include <QPointF>
#include <QRectF>
#include <QColor>
#include <QSizeF>
#include <QDebug>
#include <vector>
#include <limits>
#include "trackingcommon.h"

/**
 * @brief Central storage class for all tracking data
 * 
 * TrackingDataStorage serves as the single source of truth for both blob and track data.
 * It manages the storage, retrieval, and modification of all tracking-related data.
 * UI components like BlobTableModel and VideoLoader will use this class instead of
 * storing data themselves.
 */
class TrackingDataStorage : public QObject {
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param parent Parent QObject for memory management
     */
    explicit TrackingDataStorage(QObject *parent = nullptr);
    
    // --- Item (blob) management ---
    
    /**
     * @brief Add a new item (blob) to the storage
     * @param centroid The centroid of the blob in video coordinates
     * @param boundingBox The bounding box of the blob in video coordinates
     * @param frameNumber The frame number where the blob was selected
     * @param type The type of the item (default: Worm)
     * @return The ID of the newly added item
     */
    int addItem(const QPointF& centroid, const QRectF& boundingBox, int frameNumber, 
                TableItems::ItemType type = TableItems::ItemType::Worm);
    
    /**
     * @brief Remove a specific item by ID
     * @param itemId The ID of the item to remove
     * @return True if the item was removed, false if not found
     */
    bool removeItem(int itemId);
    
    /**
     * @brief Remove all items and their associated tracks
     * @return True if items were removed, false if already empty
     */
    bool removeAllItems();
    
    /**
     * @brief Set the visibility of a specific item
     * @param itemId The ID of the item
     * @param visible True to make the item visible, false to hide it
     */
    void setItemVisibility(int itemId, bool visible);
    
    /**
     * @brief Set visibility for all items
     * @param visible True to make all items visible, false to hide all
     */
    void setAllItemsVisibility(bool visible);
    
    /**
     * @brief Set the color of a specific item
     * @param itemId The ID of the item
     * @param color The new color
     */
    void setItemColor(int itemId, const QColor& color);
    
    /**
     * @brief Set the type of a specific item
     * @param itemId The ID of the item
     * @param type The new type
     */
    void setItemType(int itemId, TableItems::ItemType type);
    
    /**
     * @brief Set the ROI size multiplier for calculating ROI size
     * @param multiplier The new multiplier value
     */
    void setRoiSizeMultiplier(double multiplier);
    
    // --- Track management ---
    
    /**
     * @brief Set tracking data for a specific item
     * @param itemId The ID of the item
     * @param trackPoints Vector of track points
     */
    void setTrackForItem(int itemId, const std::vector<Tracking::WormTrackPoint>& trackPoints);
    
    /**
     * @brief Clear track data for a specific item
     * @param itemId The ID of the item
     */
    void clearTrackForItem(int itemId);
    
    /**
     * @brief Clear all track data for all items
     */
    void clearAllTracks();
    
    /**
     * @brief Clear track data and compact memory usage
     * This is more aggressive than clearAllTracks() and forces memory deallocation
     */
    void clearAndCompactTrackData();
    
    // --- Data access ---
    
    /**
     * @brief Get all items
     * @return Const reference to the list of all items
     */
    const QList<TableItems::ClickedItem>& getAllItems() const;
    
    /**
     * @brief Get a specific item by ID
     * @param itemId The ID of the item
     * @return Pointer to the item, or nullptr if not found
     */
    const TableItems::ClickedItem* getItem(int itemId) const;
    
    /**
     * @brief Get a specific item by index
     * @param index The index in the items list
     * @return Const reference to the item
     * @throws std::out_of_range if index is invalid
     */
    const TableItems::ClickedItem& getItemByIndex(int index) const;
    
    /**
     * @brief Get all track data
     * @return Map of item IDs to track points
     */
    const Tracking::AllWormTracks& getAllTracks() const;
    
    /**
     * @brief Get all item IDs
     * @return Set of all item IDs
     */
    QSet<int> getAllItemIds() const;
    
    /**
     * @brief Get IDs of items that have track data
     * @return Set of item IDs with tracks
     */
    QSet<int> getItemsWithTracks() const;
    
    /**
     * @brief Get worm position and ROI for a specific frame
     * @param wormId The ID of the worm
     * @param frameNumber The frame number to get data for
     * @param outPosition Output parameter for the worm's position (centroid)
     * @param outRoi Output parameter for the worm's ROI
     * @return True if worm data was found for the frame, false otherwise
     */
    bool getWormDataForFrame(int wormId, int frameNumber, QPointF& outPosition, QRectF& outRoi) const;
    
    /**
     * @brief Get the total number of items
     * @return Number of items
     */
    int getItemCount() const;
    
    /**
     * @brief Get the index of an item by its ID
     * @param itemId The ID of the item
     * @return The index of the item, or -1 if not found
     */
    int getIndexFromId(int itemId) const;
    
    /**
     * @brief Get the current fixed ROI size
     * @return The fixed ROI size
     */
    QSizeF getCurrentFixedRoiSize() const;
    
    /**
     * @brief Get the ROI size multiplier
     * @return The current multiplier value
     */
    double getRoiSizeMultiplier() const;
    
    /**
     * @brief Get lost tracking segments for a specific worm
     * @param wormId The ID of the worm to analyze
     * @return List of frame number ranges where tracking was lost (pairs of start/end frame numbers)
     */
    QList<QPair<int, int>> getLostTrackingSegments(int wormId) const;
    
    /**
     * @brief Get all frames where tracking was lost for a specific worm
     * @param wormId The ID of the worm to analyze
     * @return Set of frame numbers where tracking quality is Lost
     */
    QSet<int> getLostTrackingFrames(int wormId) const;
    
    /**
     * @brief Get minimum observed area
     * @return Minimum area value
     */
    double getMinObservedArea() const;
    
    /**
     * @brief Get maximum observed area
     * @return Maximum area value
     */
    double getMaxObservedArea() const;
    
    /**
     * @brief Get minimum observed aspect ratio
     * @return Minimum aspect ratio value
     */
    double getMinObservedAspectRatio() const;
    
    /**
     * @brief Get maximum observed aspect ratio
     * @return Maximum aspect ratio value
     */
    double getMaxObservedAspectRatio() const;

signals:
    /**
     * @brief Emitted when an item is added
     * @param itemId ID of the added item
     */
    void itemAdded(int itemId);
    
    /**
     * @brief Emitted when an item is removed
     * @param itemId ID of the removed item
     */
    void itemRemoved(int itemId);
    
    /**
     * @brief Emitted when an item's data is changed
     * @param itemId ID of the changed item
     */
    void itemChanged(int itemId);
    
    /**
     * @brief Emitted when an item's visibility is changed
     * @param itemId ID of the item
     * @param visible New visibility state
     */
    void itemVisibilityChanged(int itemId, bool visible);
    
    /**
     * @brief Emitted when an item's color is changed
     * @param itemId ID of the item
     * @param color New color
     */
    void itemColorChanged(int itemId, const QColor& color);
    
    /**
     * @brief Emitted when the list of items changes (for compatibility with BlobTableModel)
     * @param allItems The complete current list of items
     */
    void itemsChanged(const QList<TableItems::ClickedItem>& allItems);
    
    /**
     * @brief Emitted when track data is added for an item
     * @param itemId ID of the item
     */
    void trackAdded(int itemId);
    
    /**
     * @brief Emitted when track data is removed for an item
     * @param itemId ID of the item
     */
    void trackRemoved(int itemId);
    
    /**
     * @brief Emitted when all data has significantly changed
     * Used for major updates that require complete refresh
     */
    void allDataChanged();
    
    /**
     * @brief Emitted when global metrics (min/max area, aspect ratio, ROI size) change
     * @param minArea Minimum observed area
     * @param maxArea Maximum observed area
     * @param minAspectRatio Minimum observed aspect ratio
     * @param maxAspectRatio Maximum observed aspect ratio
     * @param fixedRoiSize Fixed ROI size calculated from metrics
     */
    void globalMetricsUpdated(double minArea, double maxArea,
                             double minAspectRatio, double maxAspectRatio,
                             const QSizeF& fixedRoiSize);

private:
    QList<TableItems::ClickedItem> m_items;                // List of all blob items
    QMap<int, int> m_idToIndexMap;                         // Maps item ID to index in m_items
    Tracking::AllWormTracks m_tracks;                      // Maps item ID to track points
    
    // Frame index for fast track point lookup: wormId -> frameNumber -> trackPoint pointer
    QMap<int, QMap<int, const Tracking::WormTrackPoint*>> m_frameIndex;
    
    int m_nextId;                                          // Next available ID
    
    // Color management (from BlobTableModel)
    QList<QColor> m_predefinedColors;                      // List of predefined colors
    int m_currentColorIndex;                               // Current index in color list
    
    // Global metrics (from BlobTableModel)
    double m_minObservedArea;                             
    double m_maxObservedArea;
    double m_minObservedAspectRatio;
    double m_maxObservedAspectRatio;
    QSizeF m_currentFixedRoiSize;                          // Standard ROI size
    double m_roiSizeMultiplier;                            // User-adjustable multiplier
    
    // Helper methods
    QColor getNextColor();                                 // Get next color from palette
    void initializeColors();                               // Initialize color palette
    void recalculateGlobalMetricsAndROIs();                // Update metrics and ROIs
    void updateIdToIndexMap();                             // Rebuild ID-to-index map
    void buildFrameIndex();                                // Build frame index for fast lookups
};

#endif // TRACKINGDATASTORAGE_H