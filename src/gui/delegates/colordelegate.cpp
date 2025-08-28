#include "colordelegate.h"
#include <QDebug>
#include <QColorDialog>
#include <QMouseEvent>
#include <QApplication> // For style helpers
#include <QStyle>

ColorDelegate::ColorDelegate(QObject *parent)
    : QStyledItemDelegate(parent),
      m_swatchPadding(2),
      m_swatchSize(16)
{
}

void ColorDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option,
                          const QModelIndex &index) const
{
    painter->save();

    // Draw selection background if selected
    if (option.state & QStyle::State_Selected) {
        painter->fillRect(option.rect, option.palette.highlight());
    }

    // Attempt to read a QColor from the model (EditRole commonly stores QColor)
    QVariant var = index.model()->data(index, Qt::EditRole);
    QColor color = var.canConvert<QColor>() ? var.value<QColor>() : QColor();

    if (!color.isValid()) {
        // Fallback to a transparent/placeholder color if invalid
        color = Qt::transparent;
    }

    // Compute swatch rectangle centered within the cell rect
    int swatchHeight = m_swatchSize;
    int swatchWidth = qMax(m_swatchSize, option.rect.width() - 2 * m_swatchPadding);
    QRect swatchRect = QStyle::alignedRect(
        option.direction,
        Qt::AlignCenter,
        QSize(swatchWidth, swatchHeight),
        option.rect
    );

    // Draw the swatch
    painter->setPen(option.palette.color(QPalette::Mid));
    painter->fillRect(swatchRect, color);
    painter->drawRect(swatchRect);

    painter->restore();
}

QSize ColorDelegate::sizeHint(const QStyleOptionViewItem &option,
                              const QModelIndex &index) const
{
    Q_UNUSED(option);
    Q_UNUSED(index);

    QSize hint = QStyledItemDelegate::sizeHint(option, index);
    hint.setHeight(m_swatchSize + 2 * m_swatchPadding + 4);
    hint.setWidth(m_swatchSize * 2 + 2 * m_swatchPadding);
    return hint;
}

QWidget *ColorDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                                     const QModelIndex &index) const
{
    Q_UNUSED(parent);
    Q_UNUSED(option);
    Q_UNUSED(index);
    // We do not create a persistent editor widget in-place; use a modal QColorDialog on click.
    return nullptr;
}

void ColorDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
    Q_UNUSED(editor);
    Q_UNUSED(index);
    // No persistent editor to populate.
}

void ColorDelegate::setModelData(QWidget *editor, QAbstractItemModel *model,
                                 const QModelIndex &index) const
{
    Q_UNUSED(editor);
    Q_UNUSED(model);
    Q_UNUSED(index);
    // Model data is set directly in editorEvent when dialog accepted.
}

bool ColorDelegate::editorEvent(QEvent *event, QAbstractItemModel *model,
                                const QStyleOptionViewItem &option, const QModelIndex &index)
{
    // Handle mouse release to open color dialog
    if (event->type() == QEvent::MouseButtonRelease) {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() == Qt::LeftButton) {
            // Read current color (EditRole preferred)
            QVariant currentVar = index.model()->data(index, Qt::EditRole);
            QColor currentColor = currentVar.canConvert<QColor>() ? currentVar.value<QColor>() : QColor(Qt::white);

            QColorDialog dialog(currentColor, nullptr);
            dialog.setWindowTitle("Select Color");
            dialog.setOption(QColorDialog::ShowAlphaChannel, true);

            if (dialog.exec() == QDialog::Accepted) {
                QColor newColor = dialog.selectedColor();
                if (newColor.isValid() && newColor != currentColor) {
                    // Use EditRole so model is aware it's a QColor
                    model->setData(index, newColor, Qt::EditRole);
                }
            }
            return true; // event handled
        }
    }

    return QStyledItemDelegate::editorEvent(event, model, option, index);
}