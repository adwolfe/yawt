#include "blobtablemodel.h"
#include <stdexcept> // For std::out_of_range
#include <QDebug>
#include <QtMath> // For qMax, qMin, qSqrt, etc.
#include <limits> // For std::numeric_limits

BlobTableModel::BlobTableModel(TrackingDataStorage* storage, QObject *parent)
    : QAbstractTableModel(parent),
    m_storage(storage)
{
    // Connect to storage signals
    connect(m_storage, &TrackingDataStorage::itemAdded, this, &BlobTableModel::onStorageItemAdded);
    connect(m_storage, &TrackingDataStorage::itemRemoved, this, &BlobTableModel::onStorageItemRemoved);
    connect(m_storage, &TrackingDataStorage::itemChanged, this, &BlobTableModel::onStorageItemChanged);
    connect(m_storage, &TrackingDataStorage::itemVisibilityChanged, this, &BlobTableModel::onStorageItemVisibilityChanged);
    connect(m_storage, &TrackingDataStorage::itemColorChanged, this, &BlobTableModel::onStorageItemColorChanged);
    connect(m_storage, &TrackingDataStorage::allDataChanged, this, &BlobTableModel::onStorageAllDataChanged);
    connect(m_storage, &TrackingDataStorage::globalMetricsUpdated, this, &BlobTableModel::onStorageGlobalMetricsUpdated);
}

QVariant BlobTableModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (orientation != Qt::Horizontal) {
        return QVariant();
    }

    if (role == Qt::DisplayRole) {
        switch (static_cast<Column>(section)) {
        case Column::Show:      return "Show";
        case Column::ID:        return "ID";
        case Column::Color:     return "Color";
        case Column::Type:      return "Type";
        case Column::Frame:     return "Frame";
        case Column::CentroidX: return "X";
        case Column::CentroidY: return "Y";
        default:                return QVariant();
        }
    } else if (role == Qt::CheckStateRole && static_cast<Column>(section) == Column::Show) {
        // Support checkbox in header for the Show column
        // Count how many items are visible to determine header check state
        const QList<TableItems::ClickedItem>& items = m_storage->getAllItems();
        int visibleCount = 0;
        for (const auto& item : items) {
            if (item.visible) {
                visibleCount++;
            }
        }
        
        if (items.isEmpty()) {
            return QVariant(); // No items, no checkbox state
        } else if (visibleCount == 0) {
            return Qt::Unchecked;
        } else if (visibleCount == items.count()) {
            return Qt::Checked;
        } else {
            return Qt::PartiallyChecked;
        }
    }
    
    return QVariant();
}

int BlobTableModel::rowCount(const QModelIndex &parent) const {
    return parent.isValid() ? 0 : m_storage->getItemCount();
}

int BlobTableModel::columnCount(const QModelIndex &parent) const {
    return parent.isValid() ? 0 : 7; // Show, ID, Color, Type, Frame, CentroidX, CentroidY
}

QVariant BlobTableModel::data(const QModelIndex &index, int role) const {
    if (!index.isValid() || index.row() >= m_storage->getItemCount() || index.row() < 0) {
        return QVariant();
    }

    try {
        const TableItems::ClickedItem &item = m_storage->getItemByIndex(index.row());
        
        // Handle checkbox for Show column
        if (static_cast<Column>(index.column()) == Column::Show) {
            if (role == Qt::CheckStateRole) {
                return item.visible ? Qt::Checked : Qt::Unchecked;
            }
            // No display text for checkbox column
            return QVariant();
        }

        if (role == Qt::DisplayRole || role == Qt::EditRole) {
            switch (static_cast<Column>(index.column())) {
            case Column::ID:
                return item.id;
            case Column::Color:
                return item.color; // Delegate handles QColor directly for display and edit
            case Column::Type:
                return TableItems::itemTypeToString(item.type); // For display, delegate might use enum for edit
            case Column::Frame:
                return item.frameOfSelection;
            case Column::CentroidX:
                return QString::number(item.initialCentroid.x(), 'f', 2);
            case Column::CentroidY:
                return QString::number(item.initialCentroid.y(), 'f', 2);
            default:
                return QVariant();
            }
        } else if (role == Qt::DecorationRole) {
            if (static_cast<Column>(index.column()) == Column::Color) {
                return item.color; // Provide color for basic swatch if no delegate or for other roles
            }
        }
    } catch (const std::out_of_range& e) {
        qWarning() << "BlobTableModel: Error accessing item at index" << index.row() << ":" << e.what();
    }
    
    return QVariant();
}

bool BlobTableModel::setData(const QModelIndex &index, const QVariant &value, int role) {
    if (!index.isValid() || index.row() >= m_storage->getItemCount() || index.row() < 0) {
        return false;
    }

    try {
        const TableItems::ClickedItem &item = m_storage->getItemByIndex(index.row());
        int itemId = item.id;
        
        // Handle checkbox for Show column
        if (static_cast<Column>(index.column()) == Column::Show && role == Qt::CheckStateRole) {
            Qt::CheckState checkState = static_cast<Qt::CheckState>(value.toInt());
            bool newVisible = (checkState == Qt::Checked);
            if (item.visible != newVisible) {
                m_storage->setItemVisibility(itemId, newVisible);
                return true;
            }
        } else if (role == Qt::EditRole) {
            switch (static_cast<Column>(index.column())) {
            case Column::Color:
                if (value.canConvert<QColor>()) {
                    QColor newColor = value.value<QColor>();
                    if (item.color != newColor) {
                        m_storage->setItemColor(itemId, newColor);
                        return true;
                    }
                }
                break;
            case Column::Type: {
                QString typeStr = value.toString();
                TableItems::ItemType newType = TableItems::stringToItemType(typeStr);
                if (item.type != newType) {
                    m_storage->setItemType(itemId, newType);
                    return true;
                }
                break;
            }
            case Column::ID:
            case Column::Frame:
            case Column::CentroidX:
            case Column::CentroidY:
            case Column::Show: // Already handled above with CheckStateRole
            default:
                return false;
            }
        }
    } catch (const std::out_of_range& e) {
        qWarning() << "BlobTableModel: Error accessing item at index" << index.row() << ":" << e.what();
    }
    
    return false;
}

Qt::ItemFlags BlobTableModel::flags(const QModelIndex& index) const {
    if (!index.isValid()) {
        return Qt::NoItemFlags;
    }
    Qt::ItemFlags defaultFlags = QAbstractTableModel::flags(index);
    
    Column col = static_cast<Column>(index.column());
    if (col == Column::Type || col == Column::Color) {
        return defaultFlags | Qt::ItemIsEditable;
    } else if (col == Column::Show) {
        return defaultFlags | Qt::ItemIsUserCheckable;
    }
    
    return defaultFlags;
}

bool BlobTableModel::addItem(const QPointF& centroid, const QRectF& boundingBox, int frameNumber, TableItems::ItemType type) {
    // Delegate to storage
    int newId = m_storage->addItem(centroid, boundingBox, frameNumber, type);
    return (newId > 0);
}

bool BlobTableModel::removeRows(int position, int rows, const QModelIndex &parent) {
    Q_UNUSED(parent);
    if (position < 0 || position + rows > m_storage->getItemCount() || rows <= 0) {
        return false;
    }
    
    // We need to remove items one by one by their IDs
    bool success = true;
    QList<int> itemsToRemove;
    
    // First collect all the item IDs to remove
    for (int i = 0; i < rows; ++i) {
        try {
            const TableItems::ClickedItem &item = m_storage->getItemByIndex(position + i);
            itemsToRemove.append(item.id);
        } catch (const std::out_of_range& e) {
            qWarning() << "BlobTableModel: Error accessing item at index" << (position + i) << ":" << e.what();
            success = false;
        }
    }
    
    // Then remove them from storage
    for (int id : itemsToRemove) {
        if (!m_storage->removeItem(id)) {
            success = false;
        }
    }
    
    return success;
}

bool BlobTableModel::setHeaderData(int section, Qt::Orientation orientation, const QVariant &value, int role) {
    // Only handle the Show column header for CheckStateRole
    if (orientation == Qt::Horizontal && 
        static_cast<Column>(section) == Column::Show && 
        role == Qt::CheckStateRole) {
        
        Qt::CheckState newState = static_cast<Qt::CheckState>(value.toInt());
        bool checked = (newState == Qt::Checked);
        
        // Toggle all items visibility
        toggleAllVisibility(checked);
        
        return true;
    }
    
    return QAbstractTableModel::setHeaderData(section, orientation, value, role);
}

void BlobTableModel::toggleAllVisibility(bool checked) {
    m_storage->setAllItemsVisibility(checked);
}

const TableItems::ClickedItem& BlobTableModel::getItem(int row) const {
    return m_storage->getItemByIndex(row);
}

const QList<TableItems::ClickedItem>& BlobTableModel::getAllItems() const {
    return m_storage->getAllItems();
}

// --- Public Getters for Metrics (now from storage) ---
double BlobTableModel::getMinObservedArea() const {
    return m_storage->getMinObservedArea();
}

double BlobTableModel::getMaxObservedArea() const {
    return m_storage->getMaxObservedArea();
}

double BlobTableModel::getMinObservedAspectRatio() const {
    return m_storage->getMinObservedAspectRatio();
}

double BlobTableModel::getMaxObservedAspectRatio() const {
    return m_storage->getMaxObservedAspectRatio();
}

QSizeF BlobTableModel::getCurrentFixedRoiSize() const {
    return m_storage->getCurrentFixedRoiSize();
}

double BlobTableModel::getRoiSizeMultiplier() const {
    return m_storage->getRoiSizeMultiplier();
}

void BlobTableModel::updateRoiSizeMultiplier(double newMultiplier) {
    m_storage->setRoiSizeMultiplier(newMultiplier);
}

// --- Private slots to handle storage signals ---

void BlobTableModel::onStorageItemAdded(int itemId) {
    Q_UNUSED(itemId);
    // Full model reset is simplest but we could optimize with beginInsertRows
    beginResetModel();
    endResetModel();
}

void BlobTableModel::onStorageItemRemoved(int itemId) {
    Q_UNUSED(itemId);
    // Full model reset is simplest but we could optimize with beginRemoveRows
    beginResetModel();
    endResetModel();
}

void BlobTableModel::onStorageItemChanged(int itemId) {
    // Find the row for this item ID
    int row = -1;
    for (int i = 0; i < m_storage->getItemCount(); ++i) {
        if (m_storage->getItemByIndex(i).id == itemId) {
            row = i;
            break;
        }
    }
    
    if (row >= 0) {
        QModelIndex topLeft = index(row, 0);
        QModelIndex bottomRight = index(row, columnCount() - 1);
        emit dataChanged(topLeft, bottomRight);
    }
}

void BlobTableModel::onStorageItemVisibilityChanged(int itemId, bool visible) {
    // Forward the signal
    emit itemVisibilityChanged(itemId, visible);
    
    // Update header checkbox state
    emit headerDataChanged(Qt::Horizontal, Column::Show, Column::Show);
    
    // Find the row and update the checkbox cell
    int row = -1;
    for (int i = 0; i < m_storage->getItemCount(); ++i) {
        if (m_storage->getItemByIndex(i).id == itemId) {
            row = i;
            break;
        }
    }
    
    if (row >= 0) {
        QModelIndex checkboxIndex = index(row, Column::Show);
        emit dataChanged(checkboxIndex, checkboxIndex, {Qt::CheckStateRole});
    }
}

void BlobTableModel::onStorageItemColorChanged(int itemId, const QColor& color) {
    // Forward the signal
    emit itemColorChanged(itemId, color);
    
    // Find the row and update the color cell
    int row = -1;
    for (int i = 0; i < m_storage->getItemCount(); ++i) {
        if (m_storage->getItemByIndex(i).id == itemId) {
            row = i;
            break;
        }
    }
    
    if (row >= 0) {
        QModelIndex colorIndex = index(row, Column::Color);
        emit dataChanged(colorIndex, colorIndex, {Qt::DisplayRole, Qt::EditRole, Qt::DecorationRole});
    }
}

void BlobTableModel::onStorageAllDataChanged() {
    // Full model reset
    beginResetModel();
    endResetModel();
    
    // Forward the signal to connected components
    emit itemsChanged(m_storage->getAllItems());
}

void BlobTableModel::onStorageGlobalMetricsUpdated(double minArea, double maxArea,
                                                double minAspectRatio, double maxAspectRatio,
                                                const QSizeF& fixedRoiSize) {
    // Forward the signal
    emit globalMetricsUpdated(minArea, maxArea, minAspectRatio, maxAspectRatio, fixedRoiSize);
}

