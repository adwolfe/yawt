#include "itemtypedelegate.h"
#include "../../data/trackingcommon.h"
#include <QComboBox>
#include <QStringList>

ItemTypeDelegate::ItemTypeDelegate(QObject *parent)
    : QStyledItemDelegate(parent) {}

QWidget *ItemTypeDelegate::createEditor(QWidget *parent,
                                        const QStyleOptionViewItem &option,
                                        const QModelIndex &index) const {
    Q_UNUSED(option);

    // Create a QComboBox as the editor
    QComboBox *editor = new QComboBox(parent);

    const QString currentTypeStr = index.model()->data(index, Qt::EditRole).toString();
    const TableItems::ItemType currentType = TableItems::stringToItemType(currentTypeStr);

    QStringList types;
    if (currentType == TableItems::ItemType::StartPoint ||
        currentType == TableItems::ItemType::EndPoint) {
        types << itemTypeToString(TableItems::ItemType::StartPoint)
              << itemTypeToString(TableItems::ItemType::EndPoint);
    } else if (currentType == TableItems::ItemType::Worm ||
               currentType == TableItems::ItemType::Fix) {
        types << itemTypeToString(TableItems::ItemType::Worm)
              << itemTypeToString(TableItems::ItemType::Fix);
    } else {
        types << currentTypeStr;
    }

    editor->addItems(types);

    return editor;
}

void ItemTypeDelegate::setEditorData(QWidget *editor,
                                     const QModelIndex &index) const {
    // Get the current value from the model (which is a string from data() method)
    QString currentTypeStr = index.model()->data(index, Qt::EditRole).toString();

    QComboBox *comboBox = static_cast<QComboBox*>(editor);
    if (comboBox) {
        // Find the index of the current type string in the combo box and set it
        int comboIndex = comboBox->findText(currentTypeStr);
        if (comboIndex >= 0) {
            comboBox->setCurrentIndex(comboIndex);
        } else {
            // Fallback if the string is not found (e.g., "Unknown" or corrupted data)
            // You might want to set it to a default, like "Undefined"
            int undefinedIndex = comboBox->findText(itemTypeToString(TableItems::ItemType::Undefined));
            if (undefinedIndex >=0) {
                comboBox->setCurrentIndex(undefinedIndex);
            }
        }
    }
}

void ItemTypeDelegate::setModelData(QWidget *editor, QAbstractItemModel *model,
                                    const QModelIndex &index) const {
    QComboBox *comboBox = static_cast<QComboBox*>(editor);
    if (comboBox) {
        // Get the selected string from the QComboBox
        QString selectedTypeStr = comboBox->currentText();
        // The model's setData method expects the string representation for the Type column
        model->setData(index, selectedTypeStr, Qt::EditRole);
    }
}

void ItemTypeDelegate::updateEditorGeometry(QWidget *editor,
                                            const QStyleOptionViewItem &option,
                                            const QModelIndex &index) const {
    Q_UNUSED(index);
    // Set the editor's geometry to be the same as the cell it's editing
    editor->setGeometry(option.rect);
}
