#ifndef ITEMTYPEFILTERPROXYMODEL_H
#define ITEMTYPEFILTERPROXYMODEL_H

#include <QSortFilterProxyModel>
#include <QSet>
#include "../data/trackingcommon.h"

class ItemTypeFilterProxyModel : public QSortFilterProxyModel {
    Q_OBJECT

public:
    explicit ItemTypeFilterProxyModel(QObject *parent = nullptr);

    void setAllowedTypes(const QSet<TableItems::ItemType>& types);

protected:
    bool filterAcceptsRow(int source_row, const QModelIndex& source_parent) const override;

private:
    QSet<TableItems::ItemType> m_allowedTypes;
};

#endif // ITEMTYPEFILTERPROXYMODEL_H
