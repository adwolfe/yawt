#include "itemtypefilterproxymodel.h"
#include "blobtablemodel.h"

ItemTypeFilterProxyModel::ItemTypeFilterProxyModel(QObject *parent)
    : QSortFilterProxyModel(parent) {}

void ItemTypeFilterProxyModel::setAllowedTypes(const QSet<TableItems::ItemType>& types) {
    beginFilterChange();
    m_allowedTypes = types;
    endFilterChange();
}

bool ItemTypeFilterProxyModel::filterAcceptsRow(int source_row, const QModelIndex& source_parent) const {
    if (m_allowedTypes.isEmpty()) {
        return true;
    }
    if (!sourceModel()) {
        return false;
    }
    const int typeColumn = static_cast<int>(BlobTableModel::Column::Type);
    QModelIndex typeIdx = sourceModel()->index(source_row, typeColumn, source_parent);
    QString typeStr = sourceModel()->data(typeIdx, Qt::DisplayRole).toString();
    TableItems::ItemType type = TableItems::stringToItemType(typeStr);
    return m_allowedTypes.contains(type);
}
