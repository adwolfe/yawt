#include "blobtablemodel.h"
#include <stdexcept> // For std::out_of_range
#include <QDebug>
#include <QtMath> // For qMax, qMin, qSqrt, etc.
#include <limits> // For std::numeric_limits

// Define a default small ROI size for when no worms are present or dimensions are zero
const QSizeF DEFAULT_ROI_SIZE(20.0, 20.0); // Example: 20x20 pixels
// ROI_SIZE_MULTIPLIER replaced with m_roiSizeMultiplier member variable

BlobTableModel::BlobTableModel(QObject *parent)
    : QAbstractTableModel(parent),
    m_nextId(1),
    m_currentColorIndex(0),
    m_minObservedArea(std::numeric_limits<double>::max()),
    m_maxObservedArea(0.0),
    m_minObservedAspectRatio(std::numeric_limits<double>::max()),
    m_maxObservedAspectRatio(0.0),
    m_currentFixedRoiSize(DEFAULT_ROI_SIZE), // Initialize with a default
    m_roiSizeMultiplier(1.5) // Default ROI size multiplier
{
    initializeColors();
    // Initial call to set up ROI even if no items yet, or to reset if loading an empty state
    // recalculateGlobalMetricsAndROIs(); // Not strictly needed here if no items, but good for consistency
}

void BlobTableModel::initializeColors() {
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

QColor BlobTableModel::getNextColor() {
    if (m_predefinedColors.isEmpty()) {
        return QColor(Qt::gray); // Fallback
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
    case Column::CentroidX: return "X";
    case Column::CentroidY: return "Y";
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

    const TableItems::ClickedItem &item = m_items.at(index.row());

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

    TableItems::ClickedItem &item = m_items[index.row()];
    bool dataWasChanged = false;
    bool typeChanged = false;

    switch (static_cast<Column>(index.column())) {
    case Column::Color:
        if (value.canConvert<QColor>()) {
            QColor newColor = value.value<QColor>();
            if (item.color != newColor) {
                item.color = newColor;
                dataWasChanged = true;
                emit itemColorChanged(item.id, newColor);
            }
        }
        break;
    case Column::Type: {
        QString typeStr = value.toString();
        TableItems::ItemType newType = TableItems::stringToItemType(typeStr);
        if (item.type != newType) {
            item.type = newType;
            dataWasChanged = true;
            typeChanged = true; // Mark that type specifically changed for recalculation
        }
        break;
    }
    case Column::ID:
    case Column::Frame:
    case Column::CentroidX:
    case Column::CentroidY:
    default:
        return false;
    }

    if (dataWasChanged) {
        emit dataChanged(index, index, {Qt::DisplayRole, Qt::EditRole, Qt::DecorationRole});
        // itemsChanged will be emitted by recalculateGlobalMetricsAndROIs if type changed
        // or if other changes necessitate a full refresh.
        // If only color changed, we don't need to recalculate global ROIs.
        if (typeChanged) {
            recalculateGlobalMetricsAndROIs();
        } else {
            // If only color changed, we still need to inform VideoLoader to update if it uses m_itemsToDisplay directly
            // However, VideoLoader also connects to itemColorChanged.
            // To be safe and ensure VideoLoader always has the latest full list if any part of an item changes:
            emit itemsChanged(m_items);
        }
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
    return defaultFlags;
}

bool BlobTableModel::addItem(const QPointF& centroid, const QRectF& boundingBox, int frameNumber, TableItems::ItemType type) {
    beginInsertRows(QModelIndex(), rowCount(), rowCount());
    TableItems::ClickedItem newItem;
    newItem.id = m_nextId++;
    newItem.color = getNextColor();
    newItem.type = type;
    newItem.initialCentroid = centroid;
    newItem.originalClickedBoundingBox = boundingBox; // Store the original clicked bounding box
    // newItem.initialBoundingBox will be set by recalculateGlobalMetricsAndROIs
    newItem.frameOfSelection = frameNumber;
    m_items.append(newItem);
    endInsertRows();

    // Recalculate global metrics and update all item ROIs
    recalculateGlobalMetricsAndROIs();
    // itemColorChanged and itemsChanged are emitted by recalculateGlobalMetricsAndROIs

    qDebug() << "BlobTableModel: Added item ID" << newItem.id << "Original BBox:" << boundingBox;
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

    // If all items were removed, reset the ID counter
    if (m_items.isEmpty()) {
        m_nextId = 1;
    } else {
        // Renumber all remaining items sequentially
        QModelIndex topLeft = index(0, 0);
        QModelIndex bottomRight = index(m_items.count() - 1, 0);
        
        // Update IDs to be sequential
        for (int i = 0; i < m_items.count(); ++i) {
            m_items[i].id = i + 1;
        }
        
        // Set m_nextId to be one more than the highest ID
        m_nextId = m_items.count() + 1;
        
        // Notify views that the ID column data has changed
        emit dataChanged(topLeft, bottomRight, {Qt::DisplayRole});
    }

    // Recalculate global metrics and update remaining item ROIs
    recalculateGlobalMetricsAndROIs();
    return true;
}

const TableItems::ClickedItem& BlobTableModel::getItem(int row) const {
    if (row < 0 || row >= m_items.count()) {
        throw std::out_of_range("Row index out of range in BlobTableModel::getItem");
    }
    return m_items.at(row);
}

const QList<TableItems::ClickedItem>& BlobTableModel::getAllItems() const {
    return m_items;
}

// --- New Public Getters for Metrics ---
double BlobTableModel::getMinObservedArea() const {
    return m_minObservedArea;
}

double BlobTableModel::getMaxObservedArea() const {
    return m_maxObservedArea;
}

double BlobTableModel::getMinObservedAspectRatio() const {
    return m_minObservedAspectRatio;
}

double BlobTableModel::getMaxObservedAspectRatio() const {
    return m_maxObservedAspectRatio;
}

QSizeF BlobTableModel::getCurrentFixedRoiSize() const {
    return m_currentFixedRoiSize;
}

double BlobTableModel::getRoiSizeMultiplier() const {
    return m_roiSizeMultiplier;
}

void BlobTableModel::updateRoiSizeMultiplier(double newMultiplier) {
    if (!qFuzzyCompare(m_roiSizeMultiplier, newMultiplier)) {
        m_roiSizeMultiplier = newMultiplier;
        recalculateGlobalMetricsAndROIs();
    }
}

// --- Private Helper Methods ---
void BlobTableModel::recalculateGlobalMetricsAndROIs() {
    double newMinArea = std::numeric_limits<double>::max();
    double newMaxArea = 0.0;
    double newMinAspectRatio = std::numeric_limits<double>::max();
    double newMaxAspectRatio = 0.0;
    double maxObservedDimensionL = 0.0;
    int wormCount = 0;

    for (const TableItems::ClickedItem &item : qAsConst(m_items)) {
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
        qDebug() << "BlobTableModel: Global metrics updated."
                 << "Area (min/max):" << m_minObservedArea << "/" << m_maxObservedArea
                 << "Aspect (min/max):" << m_minObservedAspectRatio << "/" << m_maxObservedAspectRatio
                 << "Fixed ROI Size:" << m_currentFixedRoiSize;
        emit globalMetricsUpdated(m_minObservedArea, m_maxObservedArea,
                                  m_minObservedAspectRatio, m_maxObservedAspectRatio,
                                  m_currentFixedRoiSize);
    }

    // Always emit itemsChanged if ROIs were updated, or if items were added/removed (covered by caller)
    // or if metrics that affect display (like ROI size) changed.
    // The initial add/remove calls will trigger this function, and it will emit itemsChanged.
    // If called from setData (type change), this ensures the update.
    if (itemROIsChanged || metricsChanged) { // If ROIs changed OR other metrics changed (which implies ROI might have changed)
        emit itemsChanged(m_items);
        qDebug() << "BlobTableModel: itemsChanged emitted due to ROI or metric updates.";
    }
    // If an item was just added or removed, the model's structure changed,
    // so itemsChanged should definitely be emitted.
    // The beginInsertRows/endInsertRows and beginRemoveRows/endRemoveRows
    // handle the basic model update notifications. This itemsChanged(m_items)
    // is for the VideoLoader to get the *full list* with potentially updated ROIs.
}
