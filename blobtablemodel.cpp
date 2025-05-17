#include "blobtablemodel.h"
#include <stdexcept> // For std::out_of_range
#include <QDebug>

BlobTableModel::BlobTableModel(QObject *parent)
    : QAbstractTableModel(parent), m_nextId(1), m_currentColorIndex(0)
{
    initializeColors();
}

void BlobTableModel::initializeColors() {
    m_predefinedColors << QColor(Qt::red).lighter(120)
    << QColor(Qt::green).lighter(120)
    << QColor(Qt::blue).lighter(120)
    << QColor(Qt::cyan).lighter(120)
    << QColor(Qt::magenta).lighter(120)
    << QColor(Qt::yellow).lighter(120)
    << QColor(Qt::gray).lighter(120)
    << QColor(Qt::darkRed).lighter(130)
    << QColor(Qt::darkGreen).lighter(130)
    << QColor(Qt::darkBlue).lighter(130)
    << QColor(Qt::darkCyan).lighter(130)
    << QColor(Qt::darkMagenta).lighter(130)
    << QColor(Qt::darkYellow).lighter(130);
    // Add more distinct colors if needed
}

QColor BlobTableModel::getNextColor() {
    if (m_predefinedColors.isEmpty()) {
        return QColor(Qt::black); // Fallback
    }
    QColor color = m_predefinedColors.at(m_currentColorIndex);
    m_currentColorIndex = (m_currentColorIndex + 1) % m_predefinedColors.count();
    return color;
}

QVariant BlobTableModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (role != Qt::DisplayRole || orientation != Qt::Horizontal) {
        return QVariant();
    }

    switch (static_cast<Column>(section)) {
    case Column::ID:        return "ID";
    case Column::Color:     return "Color";
    case Column::Type:      return "Type";
    case Column::Frame:     return "Frame";
    case Column::CentroidX: return "Centroid X";
    case Column::CentroidY: return "Centroid Y";
    default:                return QVariant();
    }
}

int BlobTableModel::rowCount(const QModelIndex &parent) const {
    return parent.isValid() ? 0 : m_items.count();
}

int BlobTableModel::columnCount(const QModelIndex &parent) const {
    return parent.isValid() ? 0 : 6; // ID, Color, Type, Frame, CentroidX, CentroidY
}

QVariant BlobTableModel::data(const QModelIndex &index, int role) const {
    if (!index.isValid() || index.row() >= m_items.count() || index.row() < 0) {
        return QVariant();
    }

    const TrackedItem &item = m_items.at(index.row());

    if (role == Qt::DisplayRole || role == Qt::EditRole) {
        switch (static_cast<Column>(index.column())) {
        case Column::ID:
            return item.id;
        case Column::Color:
            return item.color; // Delegate handles QColor directly for display and edit
        case Column::Type:
            return itemTypeToString(item.type); // For display, delegate might use enum for edit
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
    return QVariant();
}

bool BlobTableModel::setData(const QModelIndex &index, const QVariant &value, int role) {
    if (!index.isValid() || role != Qt::EditRole || index.row() >= m_items.count() || index.row() < 0) {
        return false;
    }

    TrackedItem &item = m_items[index.row()];
    bool dataWasChanged = false;

    switch (static_cast<Column>(index.column())) {
    case Column::Color:
        if (value.canConvert<QColor>()) {
            QColor newColor = value.value<QColor>();
            if (item.color != newColor) {
                item.color = newColor;
                dataWasChanged = true;
                emit itemColorChanged(item.id, newColor); // Emit specific color change
            }
        }
        break;
    case Column::Type: {
        // Assuming value is QString from ItemTypeDelegate or direct edit
        QString typeStr = value.toString();
        ItemType newType = stringToItemType(typeStr); // Convert string from delegate to enum
        if (item.type != newType) {
            item.type = newType;
            dataWasChanged = true;
        }
        break;
    }
    // ID, Frame, CentroidX, CentroidY are not typically editable directly by user this way
    case Column::ID:
    case Column::Frame:
    case Column::CentroidX:
    case Column::CentroidY:
    default:
        return false; // Not editable or unhandled column
    }

    if (dataWasChanged) {
        emit dataChanged(index, index, {Qt::DisplayRole, Qt::EditRole, Qt::DecorationRole});
        emit itemsChanged(m_items); // Emit that the full list might need refreshing elsewhere
        return true;
    }
    return false;
}

Qt::ItemFlags BlobTableModel::flags(const QModelIndex& index) const {
    if (!index.isValid()) {
        return Qt::NoItemFlags;
    }
    Qt::ItemFlags defaultFlags = QAbstractTableModel::flags(index);
    if (static_cast<Column>(index.column()) == Column::Type || static_cast<Column>(index.column()) == Column::Color) {
        return defaultFlags | Qt::ItemIsEditable;
    }
    return defaultFlags; // Other columns are not editable by default
}

bool BlobTableModel::addItem(const QPointF& centroid, const QRectF& boundingBox, int frameNumber, ItemType type) {
    beginInsertRows(QModelIndex(), rowCount(), rowCount());
    TrackedItem newItem;
    newItem.id = m_nextId++;
    newItem.color = getNextColor();
    newItem.type = type;
    newItem.initialCentroid = centroid;
    newItem.initialBoundingBox = boundingBox;
    newItem.frameOfSelection = frameNumber;
    m_items.append(newItem);
    endInsertRows();

    emit itemColorChanged(newItem.id, newItem.color); // For specific color update listeners
    emit itemsChanged(m_items); // For listeners needing the whole list (like VideoLoader display)
    qDebug() << "BlobTableModel: Added item ID" << newItem.id << "with color" << newItem.color.name();
    return true;
}

bool BlobTableModel::removeRows(int position, int rows, const QModelIndex &parent) {
    Q_UNUSED(parent);
    if (position < 0 || position + rows > m_items.count() || rows <= 0) {
        return false;
    }
    beginRemoveRows(QModelIndex(), position, position + rows - 1);
    for (int i = 0; i < rows; ++i) {
        m_items.removeAt(position);
    }
    endRemoveRows();
    emit itemsChanged(m_items); // Update listeners
    return true;
}

const TrackedItem& BlobTableModel::getItem(int row) const {
    if (row < 0 || row >= m_items.count()) {
        throw std::out_of_range("Row index out of range in BlobTableModel::getItem");
    }
    return m_items.at(row);
}

const QList<TrackedItem>& BlobTableModel::getAllItems() const {
    return m_items;
}
