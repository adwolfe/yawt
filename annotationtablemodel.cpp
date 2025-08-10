#include "annotationtablemodel.h"
#include "trackingdatastorage.h"
#include <QDebug>
#include <algorithm>

AnnotationTableModel::AnnotationTableModel(TrackingDataStorage* storage, QObject* parent)
    : QAbstractTableModel(parent), m_storage(nullptr)
{
    setStorage(storage);
}

void AnnotationTableModel::setStorage(TrackingDataStorage* storage) {
    if (m_storage == storage) {
        return; // No change
    }
    
    beginResetModel();
    
    // Disconnect from old storage
    disconnectStorageSignals();
    
    m_storage = storage;
    m_annotations.clear();
    
    // Connect to new storage and populate data
    if (m_storage) {
        connectStorageSignals();
        populateLostAnnotations();
    }
    
    endResetModel();
}

void AnnotationTableModel::connectStorageSignals() {
    if (!m_storage) return;
    
    connect(m_storage, &TrackingDataStorage::trackAdded, 
            this, &AnnotationTableModel::onTrackingDataChanged);
    connect(m_storage, &TrackingDataStorage::trackRemoved, 
            this, &AnnotationTableModel::onTrackingDataChanged);
    connect(m_storage, &TrackingDataStorage::allDataChanged, 
            this, &AnnotationTableModel::onAllDataChanged);
}

void AnnotationTableModel::disconnectStorageSignals() {
    if (!m_storage) return;
    
    disconnect(m_storage, &TrackingDataStorage::trackAdded, 
               this, &AnnotationTableModel::onTrackingDataChanged);
    disconnect(m_storage, &TrackingDataStorage::trackRemoved, 
               this, &AnnotationTableModel::onTrackingDataChanged);
    disconnect(m_storage, &TrackingDataStorage::allDataChanged, 
               this, &AnnotationTableModel::onAllDataChanged);
}

void AnnotationTableModel::refreshAnnotations() {
    if (!m_storage) {
        clearAnnotations();
        return;
    }
    
    beginResetModel();
    m_annotations.clear();
    populateLostAnnotations();
    endResetModel();
}

void AnnotationTableModel::clearAnnotations() {
    beginResetModel();
    m_annotations.clear();
    endResetModel();
}

void AnnotationTableModel::populateLostAnnotations() {
    if (!m_storage) return;
    
    // Get all worm IDs that have tracks
    QSet<int> wormIds = m_storage->getItemsWithTracks();
    
    for (int wormId : wormIds) {
        // Get lost tracking segments for this worm
        QList<QPair<int, int>> lostSegments = m_storage->getLostTrackingSegments(wormId);
        
        for (const auto& segment : lostSegments) {
            m_annotations.append(AnnotationEntry(wormId, AnnotationType::Lost, 
                                               segment.first, segment.second));
        }
    }
    
    // Sort annotations by worm ID, then by start frame
    std::sort(m_annotations.begin(), m_annotations.end());
    
    qDebug() << "AnnotationTableModel: Populated" << m_annotations.size() << "lost tracking annotations";
}

// QAbstractTableModel interface implementation
int AnnotationTableModel::rowCount(const QModelIndex& parent) const {
    Q_UNUSED(parent)
    return m_annotations.size();
}

int AnnotationTableModel::columnCount(const QModelIndex& parent) const {
    Q_UNUSED(parent)
    return ColumnCount;
}

QVariant AnnotationTableModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() >= m_annotations.size()) {
        return QVariant();
    }
    
    const AnnotationEntry& annotation = m_annotations.at(index.row());
    
    switch (role) {
    case Qt::DisplayRole:
        switch (index.column()) {
        case ID:
            return annotation.wormId;
        case Type:
            return annotationTypeToString(annotation.type);
        case Frames:
            return formatFrameRange(annotation.startFrame, annotation.endFrame);
        default:
            return QVariant();
        }
        
    case Qt::TextAlignmentRole:
        switch (index.column()) {
        case ID:
            return Qt::AlignCenter;
        case Type:
            return Qt::AlignCenter;
        case Frames:
            return Qt::AlignCenter;
        default:
            return Qt::AlignLeft;
        }
        
    case Qt::ToolTipRole:
        switch (index.column()) {
        case ID:
            return QString("Worm/Track ID: %1\nClick row to navigate to frame %2")
                   .arg(annotation.wormId).arg(annotation.startFrame);
        case Type:
            return QString("Type of tracking annotation: %1\nClick row to navigate to frame %2")
                   .arg(annotationTypeToString(annotation.type)).arg(annotation.startFrame);
        case Frames:
            if (annotation.startFrame == annotation.endFrame) {
                return QString("Lost tracking on frame %1\nClick to navigate to this frame")
                       .arg(annotation.startFrame);
            } else {
                return QString("Lost tracking from frame %1 to %2 (%3 frames)\nClick to navigate to frame %1")
                       .arg(annotation.startFrame)
                       .arg(annotation.endFrame)
                       .arg(annotation.endFrame - annotation.startFrame + 1);
            }
        default:
            return QVariant();
        }
    case Qt::BackgroundRole:
        // Provide subtle background colors for different annotation types
        switch (annotation.type) {
        case AnnotationType::Lost:
            // Light red background for lost tracking
            return QColor(255, 240, 240); // Very light red
        default:
            return QVariant();
        }
        
    default:
        return QVariant();
    }
}

QVariant AnnotationTableModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (orientation != Qt::Horizontal) {
        return QVariant();
    }
    
    switch (role) {
    case Qt::DisplayRole:
        switch (section) {
        case ID:
            return "ID";
        case Type:
            return "Type";
        case Frames:
            return "Frames";
        default:
            return QVariant();
        }
        
    case Qt::ToolTipRole:
        switch (section) {
        case ID:
            return "Worm/Track identifier";
        case Type:
            return "Type of annotation (Lost, Merge, etc.)";
        case Frames:
            return "Frame range where the annotation applies";
        default:
            return QVariant();
        }
        
    default:
        return QVariant();
    }
}

// Data access methods
const AnnotationTableModel::AnnotationEntry* AnnotationTableModel::getAnnotationAtRow(int row) const {
    if (row < 0 || row >= m_annotations.size()) {
        return nullptr;
    }
    return &m_annotations.at(row);
}

int AnnotationTableModel::getTotalLostFrames() const {
    int total = 0;
    for (const auto& annotation : m_annotations) {
        if (annotation.type == AnnotationType::Lost) {
            total += (annotation.endFrame - annotation.startFrame + 1);
        }
    }
    return total;
}

int AnnotationTableModel::getAnnotationCountForWorm(int wormId) const {
    int count = 0;
    for (const auto& annotation : m_annotations) {
        if (annotation.wormId == wormId) {
            ++count;
        }
    }
    return count;
}

// Slots for storage updates
void AnnotationTableModel::onTrackingDataChanged() {
    refreshAnnotations();
}

void AnnotationTableModel::onAllDataChanged() {
    refreshAnnotations();
}

// Helper methods
QString AnnotationTableModel::formatFrameRange(int startFrame, int endFrame) const {
    if (startFrame == endFrame) {
        return QString::number(startFrame);
    } else {
        return QString("%1-%2").arg(startFrame).arg(endFrame);
    }
}

QString AnnotationTableModel::annotationTypeToString(AnnotationType type) const {
    switch (type) {
    case AnnotationType::Lost:
        return "Lost";
    default:
        return "Unknown";
    }
}