// WormTableModel.cpp
#include "wormtablemodel.h" // Lowercase include
#include <stdexcept> // For std::out_of_range

WormTableModel::WormTableModel(QObject *parent)
    : QAbstractTableModel(parent), m_nextId(1) // Start IDs from 1
{
}

QVariant WormTableModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (role != Qt::DisplayRole || orientation != Qt::Horizontal) {
        return QVariant();
    }

    switch (static_cast<Column>(section)) {
    case Column::ID:
        return "ID";
    case Column::Type:
        return "Type";
    //case Column::CentroidX:
    //    return "Centroid X";
    //case Column::CentroidY:
    //    return "Centroid Y";
    //case Column::Frame:
    //    return "Frame";
    default:
        return QVariant();
    }
}

int WormTableModel::rowCount(const QModelIndex &parent) const {
    if (parent.isValid()) { // This model is not hierarchical
        return 0;
    }
    return m_items.count();
}

int WormTableModel::columnCount(const QModelIndex &parent) const {
    if (parent.isValid()) {
        return 0;
    }
    // ID, Type, CentroidX, CentroidY, Frame
    return 5;
}

QVariant WormTableModel::data(const QModelIndex &index, int role) const {
    if (!index.isValid() || index.row() >= m_items.count() || index.row() < 0) {
        return QVariant();
    }

    const TrackedItem &item = m_items.at(index.row());

    if (role == Qt::DisplayRole || role == Qt::EditRole) {
        switch (static_cast<Column>(index.column())) {
        case Column::ID:
            return item.id;
        case Column::Type:
            // For EditRole, we might want to return the index for a QComboBox delegate
            // For DisplayRole, return the string representation
            return itemTypeToString(item.type);
        //case Column::CentroidX:
        //    return QString::number(item.initialCentroid.x(), 'f', 2); // Format to 2 decimal places
        //case Column::CentroidY:
        //    return QString::number(item.initialCentroid.y(), 'f', 2);
        //case Column::Frame:
        //    return item.frameOfSelection;
        default:
            return QVariant();
        }
    }
    return QVariant();
}

bool WormTableModel::setData(const QModelIndex &index, const QVariant &value, int role) {
    if (!index.isValid() || role != Qt::EditRole || index.row() >= m_items.count() || index.row() < 0) {
        return false;
    }

    TrackedItem &item = m_items[index.row()]; // Get non-const reference

    bool isdataChanged = false;
    switch (static_cast<Column>(index.column())) {
    case Column::Type: {
        QString typeStr = value.toString();
        ItemType newType = stringToItemType(typeStr);
        if (item.type != newType) {
            item.type = newType;
            isdataChanged = true;
        }
        break;
    }
    // ID, Centroid, Frame are not directly editable by user in this basic setup
    case Column::ID:
    //case Column::CentroidX:
    //case Column::CentroidY:
    //case Column::Frame:
    //    return false; // These are not editable through setData for now
    default:
        return false;
    }

    if (isdataChanged) {
        emit dataChanged(index, index, {role});
        return true;
    }
    return false;
}

Qt::ItemFlags WormTableModel::flags(const QModelIndex& index) const {
    if (!index.isValid()) {
        return Qt::NoItemFlags;
    }

    Qt::ItemFlags defaultFlags = QAbstractTableModel::flags(index);

    if (static_cast<Column>(index.column()) == Column::Type) {
        return defaultFlags | Qt::ItemIsEditable;
    }

    return defaultFlags;
}

bool WormTableModel::addItem(const QPointF& centroid, const QRectF& boundingBox, int frameNumber, ItemType type) {
    beginInsertRows(QModelIndex(), rowCount(), rowCount());
    TrackedItem newItem;
    newItem.id = m_nextId++;
    newItem.type = type;
    newItem.initialCentroid = centroid;
    newItem.initialBoundingBox = boundingBox;
    newItem.frameOfSelection = frameNumber;
    m_items.append(newItem);
    endInsertRows();
    return true;
}

bool WormTableModel::removeRows(int position, int rows, const QModelIndex &parent) {
    Q_UNUSED(parent);
    if (position < 0 || position + rows > m_items.count() || rows <= 0) {
        return false;
    }

    beginRemoveRows(QModelIndex(), position, position + rows - 1);
    for (int i = 0; i < rows; ++i) {
        m_items.removeAt(position);
    }
    endRemoveRows();
    return true;
}

const TrackedItem& WormTableModel::getItem(int row) const {
    if (row < 0 || row >= m_items.count()) {
        throw std::out_of_range("Row index out of range in WormTableModel::getItem");
    }
    return m_items.at(row);
}

const QList<TrackedItem>& WormTableModel::getAllItems() const {
    return m_items;
}

