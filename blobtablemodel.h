#ifndef BLOBTABLEMODEL_H
#define BLOBTABLEMODEL_H

#include <QAbstractTableModel>
#include <QList>
#include <QPointF>
#include <QRectF>
#include <QColor>
#include <QSizeF> // Added for QSizeF
#include "trackingcommon.h" // Contains TrackedItem and ItemType
#include "trackingdatastorage.h" // Central data storage
#include <limits> // For std::numeric_limits, good to have explicitly

// Forward declaration
class TrackingDataStorage;



class BlobTableModel : public QAbstractTableModel {
    Q_OBJECT

public:
    explicit BlobTableModel(TrackingDataStorage* storage, QObject *parent = nullptr);

    // Header:
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    
    // ROI size factor getter
    double getRoiSizeMultiplier() const;

    // Basic functionality:
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    
    // Override for checkbox handling
    Qt::ItemFlags flags(const QModelIndex &index) const override;
    
    // Header click handling
    bool setHeaderData(int section, Qt::Orientation orientation, const QVariant &value, int role = Qt::EditRole) override;

    // Data handling:
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;

    // Editing:
    // Already declared the flags method above

    // Custom methods for model manipulation:
    /**
     * @brief Adds a new item to the model and assigns it a color.
     * The item's initialBoundingBox will be updated based on global metrics.
     * @param centroid The centroid of the blob in video coordinates.
     * @param boundingBox The original bounding box of the blob as clicked.
     * @param frameNumber The frame number on which the blob was selected.
     * @param type The initial type of the item (defaults to Worm).
     * @return True if added successfully, false otherwise.
     */
    bool addItem(const QPointF& centroid, const QRectF& boundingBox, int frameNumber, TableItems::ItemType type = TableItems::ItemType::Worm);

    /**
     * @brief Removes rows from the model.
     * @param position The starting row index.
     * @param rows The number of rows to remove.
     * @return True if removal was successful, false otherwise.
     */
    bool removeRows(int position, int rows, const QModelIndex &parent = QModelIndex()) override;

    /**
     * @brief Gets the TrackedItem at a specific row.
     * @param row The row index.
     * @return Const reference to TrackedItem. Throws std::out_of_range if row is invalid.
     */
    const TableItems::ClickedItem& getItem(int row) const;

    /**
     * @brief Gets a list of all TrackedItems.
     * @return Const reference to the internal list of items.
     */
    const QList<TableItems::ClickedItem>& getAllItems() const;

    // --- New Public Getters for Metrics ---
    double getMinObservedArea() const;
    double getMaxObservedArea() const;
    double getMinObservedAspectRatio() const;
    double getMaxObservedAspectRatio() const;
    QSizeF getCurrentFixedRoiSize() const;


    // Enum for column indices for clarity
    enum Column {
        Show = 0,       // Show/Hide checkbox
        ID = 1,
        Color = 2,
        Type = 3,
        Frame = 4,      // Frame of selection
        CentroidX = 5,  // Optional: display centroid X
        CentroidY = 6   // Optional: display centroid Y
        // Add more columns if needed, e.g., for bounding box details
    };

signals:
    /**
     * @brief Emitted when the list of items in the model changes (add, remove, data modification).
     * This is the primary signal VideoLoader should connect to for display updates.
     * @param allItems The complete current list of TrackedItems in the model.
     */
    void itemsChanged(const QList<TableItems::ClickedItem>& allItems);

    /**
     * @brief Emitted specifically when a new item's color is first assigned or changed by user.
     * Useful for components that only care about individual color updates without needing the whole list.
     * @param id The ID of the item.
     * @param color The new color of the item.
     */
    void itemColorChanged(int id, const QColor& color);

    /**
     * @brief Emitted when an item's visibility is changed.
     * Useful for components that need to update visibility without refreshing all items.
     * @param id The ID of the item.
     * @param visible The new visibility state of the item.
     */
    void itemVisibilityChanged(int id, bool visible);

    /**
     * @brief Emitted when the calculated global metrics (min/max area, aspect ratio, fixed ROI size) change.
     * This can be used by UI elements to display these values.
     * @param minArea Minimum observed area of worms.
     * @param maxArea Maximum observed area of worms.
     * @param minAspectRatio Minimum observed aspect ratio of worms.
     * @param maxAspectRatio Maximum observed aspect ratio of worms.
     * @param fixedRoiSize The calculated QSizeF for standardized ROIs.
     */
    void globalMetricsUpdated(double minArea, double maxArea,
                              double minAspectRatio, double maxAspectRatio,
                              const QSizeF& fixedRoiSize);

public slots:
    /**
     * @brief Updates the ROI size multiplier when the user adjusts the spinbox
     * @param newMultiplier The new multiplier value
     */
    void updateRoiSizeMultiplier(double newMultiplier);
    
    /**
     * @brief Toggles visibility for all items in the model
     * @param checked If true, all items will be visible; if false, all will be hidden
     */
    void toggleAllVisibility(bool checked);

private:
    TrackingDataStorage* m_storage; // Pointer to the central data storage
    
    // Private slots to handle storage signals
    private slots:
        void onStorageItemAdded(int itemId);
        void onStorageItemRemoved(int itemId);
        void onStorageItemChanged(int itemId);
        void onStorageItemVisibilityChanged(int itemId, bool visible);
        void onStorageItemColorChanged(int itemId, const QColor& color);
        void onStorageAllDataChanged();
        void onStorageGlobalMetricsUpdated(double minArea, double maxArea,
                                          double minAspectRatio, double maxAspectRatio,
                                          const QSizeF& fixedRoiSize);
};

#endif // BLOBTABLEMODEL_H
