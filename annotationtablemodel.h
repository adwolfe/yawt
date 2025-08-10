#ifndef ANNOTATIONTABLEMODEL_H
#define ANNOTATIONTABLEMODEL_H

#include <QAbstractTableModel>
#include <QList>
#include <QPair>
#include <QString>

// Forward declarations
class TrackingDataStorage;

class AnnotationTableModel : public QAbstractTableModel {
    Q_OBJECT

public:
    enum Column {
        ID = 0,
        Type,
        Frames,
        ColumnCount
    };

    enum class AnnotationType {
        Lost
        // Future: Merge, Split, etc.
    };

    struct AnnotationEntry {
        int wormId;
        AnnotationType type;
        int startFrame;
        int endFrame;
        
        AnnotationEntry(int id, AnnotationType t, int start, int end)
            : wormId(id), type(t), startFrame(start), endFrame(end) {}
            
        // For sorting by worm ID first, then by start frame
        bool operator<(const AnnotationEntry& other) const {
            if (wormId != other.wormId) {
                return wormId < other.wormId;
            }
            return startFrame < other.startFrame;
        }
    };

    explicit AnnotationTableModel(TrackingDataStorage* storage = nullptr, QObject* parent = nullptr);
    
    // Storage management
    void setStorage(TrackingDataStorage* storage);
    void refreshAnnotations();
    void clearAnnotations();
    
    // QAbstractTableModel interface
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    
    // Data access
    const AnnotationEntry* getAnnotationAtRow(int row) const;
    int getTotalLostFrames() const;
    int getAnnotationCountForWorm(int wormId) const;

public slots:
    void onTrackingDataChanged();
    void onAllDataChanged();

private:
    void populateLostAnnotations();
    void connectStorageSignals();
    void disconnectStorageSignals();
    QString formatFrameRange(int startFrame, int endFrame) const;
    QString annotationTypeToString(AnnotationType type) const;
    
    TrackingDataStorage* m_storage;
    QList<AnnotationEntry> m_annotations;
};

#endif // ANNOTATIONTABLEMODEL_H