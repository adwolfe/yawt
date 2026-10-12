#ifndef REFERENCEANNOTATIONPANEL_H
#define REFERENCEANNOTATIONPANEL_H

#include <QWidget>
#include <QUndoStack>
#include "../../data/referenceannotations.h"

class VideoLoader;
class QComboBox;
class QSpinBox;
class QLineEdit;
class QLabel;
class QTableWidget;
class QShortcut;

class ReferenceAnnotationPanel : public QWidget {
    Q_OBJECT
public:
    explicit ReferenceAnnotationPanel(VideoLoader* video, QWidget* parent = nullptr);
    void setActive(bool active);
    bool flushBeforeLeaving();

private:
    enum Tool { Pan, Position, EndA, EndB, Centerline };
    void openVideo(const QString& path, int frames, double fps, QSize size);
    void refresh();
    void refreshOverlay();
    void commit(const ReferenceAnnotations::Document& before, const QString& label);
    void apply(const ReferenceAnnotations::Document& document);
    bool save();
    void saveAs();
    void loadFile();
    void editSourceMapping();
    void changeLabel();
    void clearTool();
    void deleteFrame();
    void swapEnds();
    void pointerPressed(QPointF p, bool remove);
    void pointerMoved(QPointF p);
    void finishDrag();
    void navigate(int direction, bool annotated);
    void selectTool();
    void updateShortcuts();
    int frameNumber() const;

    VideoLoader* m_video;
    ReferenceAnnotations::Document m_document;
    ReferenceAnnotations::Document m_dragBefore;
    QUndoStack m_undo;
    QString m_path;
    QString m_error;
    bool m_dirty = false;
    bool m_active = false;
    bool m_refreshing = false;
    bool m_dragging = false;
    int m_dragFrame = -1;
    int m_dragWorm = 1;
    int m_dragVertex = -1;
    int m_savedMode = 0;
    QSpinBox* m_worm;
    QSpinBox* m_step;
    QComboBox* m_tool;
    QComboBox* m_orientation;
    QComboBox* m_visibility;
    QComboBox* m_event;
    QComboBox* m_review;
    QLineEdit* m_notes;
    QLabel* m_status;
    QLabel* m_frameLabel;
    QTableWidget* m_table;
    QList<QShortcut*> m_shortcuts;
};
#endif
