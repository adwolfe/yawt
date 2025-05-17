#ifndef BLOBTABLEMODEL_H
#define BLOBTABLEMODEL_H

#include <QAbstractTableModel>
#include <QList>
#include <QPointF>
#include <QRectF>
#include <QColor>
#include "trackingcommon.h" // Contains TrackedItem and ItemType

class BlobTableModel : public QAbstractTableModel {
    Q_OBJECT

public:
    explicit BlobTableModel(QObject *parent = nullptr);

    // Header:
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

    // Basic functionality:
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;

    // Data handling:
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;

    // Editing:
    Qt::ItemFlags flags(const QModelIndex& index) const override;

    // Custom methods for model manipulation:
    /**
     * @brief Adds a new item to the model and assigns it a color.
     * @param centroid The centroid of the blob in video coordinates.
     * @param boundingBox The bounding box of the blob in video coordinates.
     * @param frameNumber The frame number on which the blob was selected.
     * @param type The initial type of the item (defaults to Worm).
     * @return True if added successfully, false otherwise.
     */
    bool addItem(const QPointF& centroid, const QRectF& boundingBox, int frameNumber, ItemType type = ItemType::Worm);

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
    const TrackedItem& getItem(int row) const;

    /**
     * @brief Gets a list of all TrackedItems.
     * @return Const reference to the internal list of items.
     */
    const QList<TrackedItem>& getAllItems() const;


    // Enum for column indices for clarity
    enum Column {
        ID = 0,
        Color = 1,
        Type = 2,
        Frame = 3,      // Frame of selection
        CentroidX = 4,  // Optional: display centroid X
        CentroidY = 5   // Optional: display centroid Y
        // Add more columns if needed, e.g., for bounding box details
    };

signals:
    /**
     * @brief Emitted when the list of items in the model changes (add, remove, data modification).
     * @param allItems The complete current list of TrackedItems in the model.
     */
    void itemsChanged(const QList<TrackedItem>& allItems);

    /**
     * @brief Emitted specifically when a new item's color is first assigned or changed by user.
     * Useful for components that only care about individual color updates without needing the whole list.
     * @param id The ID of the item.
     * @param color The new color of the item.
     */
    void itemColorChanged(int id, const QColor& color);


private:
    QList<TrackedItem> m_items;
    int m_nextId; // For auto-generating IDs
    QList<QColor> m_predefinedColors; // List of predefined colors
    int m_currentColorIndex; // To cycle through predefined colors

    QColor getNextColor();
    void initializeColors();
};

#endif // BLOBTABLEMODEL_H
