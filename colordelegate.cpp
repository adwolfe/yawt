#include "colordelegate.h"
#include <QDebug>
#include <QColorDialog> // Ensure QColorDialog is included
#include <QMouseEvent>  // Ensure QMouseEvent is included
#include <QApplication> // Required for QStyle::alignedRect (though not directly used here, good for style-aware components)

ColorDelegate::ColorDelegate(QObject *parent)
    : QStyledItemDelegate(parent), m_swatchPadding(2), m_swatchSize(16) // Adjust padding and size as needed
{
}

void ColorDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option,
                          const QModelIndex &index) const {
    painter->save();

    if (option.state & QStyle::State_Selected) {
        painter->fillRect(option.rect, option.palette.highlight());
    }

    // Get the color from the model data (expected to be a QColor)
    QColor color = index.model()->data(index, Qt::EditRole).value<QColor>(); // Use EditRole as it holds the QColor
    if (!color.isValid()) {
        color = Qt::transparent; // Fallback for invalid color
    }

    // Calculate rect for the color swatch
    // Center the swatch within the cell option.rect
    int h = m_swatchSize;
    int w = option.rect.width() - 2 * m_swatchPadding; // Make it wider
    if (w < m_swatchSize) w = m_swatchSize; // Ensure minimum width

    QRect swatchRect = QStyle::alignedRect(
        Qt::LeftToRight,
        Qt::AlignCenter, // Align center within the cell
        QSize(w, h),
        option.rect
        );

    // Draw the color swatch
    painter->fillRect(swatchRect, color);
    painter->setPen(option.palette.color(QPalette::Mid)); // Border color
    painter->drawRect(swatchRect);

    painter->restore();
}

QSize ColorDelegate::sizeHint(const QStyleOptionViewItem &option,
                              const QModelIndex &index) const {
    // Provide a reasonable size hint, considering the swatch size and padding
    QSize hint = QStyledItemDelegate::sizeHint(option, index);
    hint.setHeight(m_swatchSize + 2 * m_swatchPadding + 4); // A bit taller for padding
    hint.setWidth(m_swatchSize * 2 + 2 * m_swatchPadding); // Wider
    return hint;
}


// We don't create a persistent editor widget in the table cell itself.
// Instead, editorEvent will trigger a modal QColorDialog.
QWidget *ColorDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                                     const QModelIndex &index) const {
    Q_UNUSED(parent);
    Q_UNUSED(option);
    Q_UNUSED(index);
    return nullptr; // No persistent editor needed
}

void ColorDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
    // This would be used if createEditor returned a widget.
    // For QColorDialog, we pass the current color directly when creating it.
    Q_UNUSED(editor);
    Q_UNUSED(index);
}

void ColorDelegate::setModelData(QWidget *editor, QAbstractItemModel *model,
                                 const QModelIndex &index) const {
    // This is called by QAbstractItemView when the editor (QColorDialog in our case, indirectly)
    // is finished. However, we will call model->setData directly in editorEvent.
    // If QColorDialog was a QWidget returned by createEditor, this would be more relevant.
    Q_UNUSED(editor);
    Q_UNUSED(model);
    Q_UNUSED(index);
}

bool ColorDelegate::editorEvent(QEvent *event, QAbstractItemModel *model,
                                const QStyleOptionViewItem &option, const QModelIndex &index) {
    if (event->type() == QEvent::MouseButtonRelease) {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() == Qt::LeftButton) {
            // Get the current color from the model
            QColor currentColor = index.model()->data(index, Qt::EditRole).value<QColor>();

            // Open the QColorDialog
            // Pass nullptr as parent; the dialog will still be application-modal.
            QColorDialog dialog(currentColor, nullptr);
            dialog.setWindowTitle("Select Worm Color");

            // Set options for the dialog if needed (e.g., NoButtons, ShowAlphaChannel)
            // dialog.setOption(QColorDialog::ShowAlphaChannel);

            if (dialog.exec() == QDialog::Accepted) {
                QColor newColor = dialog.selectedColor();
                if (newColor.isValid() && newColor != currentColor) {
                    // Set the new color back to the model
                    // The model's setData should handle emitting dataChanged
                    model->setData(index, newColor, Qt::EditRole);
                }
            }
            return true; // Event handled
        }
    }
    return QStyledItemDelegate::editorEvent(event, model, option, index);
}
