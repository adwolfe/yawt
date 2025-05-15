// itemtypedelegate.h
#ifndef ITEMTYPEDELEGATE_H
#define ITEMTYPEDELEGATE_H

#include <QStyledItemDelegate> // QStyledItemDelegate is often preferred over QItemDelegate
#include "trackeditemdata.h"   // For ItemType and helper functions

// This is here to allow for a dropdown in the wormTableView


class ItemTypeDelegate : public QStyledItemDelegate {
    Q_OBJECT

public:
    explicit ItemTypeDelegate(QObject *parent = nullptr);

    // Override functions for custom editing
    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                          const QModelIndex &index) const override;

    void setEditorData(QWidget *editor, const QModelIndex &index) const override;
    void setModelData(QWidget *editor, QAbstractItemModel *model,
                      const QModelIndex &index) const override;

    void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option,
                              const QModelIndex &index) const override;
};

#endif // ITEMTYPEDELEGATE_H
