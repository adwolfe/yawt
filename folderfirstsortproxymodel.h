#ifndef FOLDERFIRSTSORTPROXYMODEL_H
#define FOLDERFIRSTSORTPROXYMODEL_H

#include <QSortFilterProxyModel>
#include <QFileSystemModel> // Required for QFileSystemModel type
#include <QFileInfo>        // Required for QFileInfo

class FolderFirstSortProxyModel : public QSortFilterProxyModel {
    Q_OBJECT

public:
    explicit FolderFirstSortProxyModel(QObject *parent = nullptr);

protected:
    bool lessThan(const QModelIndex &source_left, const QModelIndex &source_right) const override;
};

#endif // FOLDERFIRSTSORTPROXYMODEL_H
