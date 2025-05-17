#ifndef COLORDELEGATE_H
#define COLORDELEGATE_H

#include <QStyledItemDelegate>
#include <QColor>
#include <QPainter>
#include <QColorDialog>
#include <QApplication> // Required for QStyle::alignedRect
#include <QMouseEvent>  // Required for QMouseEvent

class ColorDelegate : public QStyledItemDelegate {
    Q_OBJECT

public:
    explicit ColorDelegate(QObject *parent = nullptr);

    // --- Rendering ---
    /**
     * @brief Renders the delegate, drawing a color swatch.
     */
    void paint(QPainter *painter, const QStyleOptionViewItem &option,
               const QModelIndex &index) const override;

    /**
     * @brief Returns the size hint for the delegate.
     */
    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const override;

    // --- Editing ---
    /**
     * @brief Creates the editor widget (not used directly, QColorDialog is modal).
     */
    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                          const QModelIndex &index) const override;

    /**
     * @brief Sets the editor's data from the model.
     */
    void setEditorData(QWidget *editor, const QModelIndex &index) const override;

    /**
     * @brief Sets the model's data from the editor.
     */
    void setModelData(QWidget *editor, QAbstractItemModel *model,
                      const QModelIndex &index) const override;

    /**
     * @brief Handles events for the editor, specifically mouse clicks to open QColorDialog.
     */
    bool editorEvent(QEvent *event, QAbstractItemModel *model,
                     const QStyleOptionViewItem &option, const QModelIndex &index) override;

private:
    int m_swatchPadding; // Padding around the color swatch
    int m_swatchSize;    // Size of the color swatch (height, width will be larger)
};

#endif // COLORDELEGATE_H
