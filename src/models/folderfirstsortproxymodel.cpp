#include "folderfirstsortproxymodel.h"
#include <QFileSystemModel>
#include <QFileInfo>
#include <QDebug> // Optional: for debugging

FolderFirstSortProxyModel::FolderFirstSortProxyModel(QObject *parent)
    : QSortFilterProxyModel(parent) {
}

bool FolderFirstSortProxyModel::lessThan(const QModelIndex &source_left, const QModelIndex &source_right) const {
    QFileSystemModel *model = qobject_cast<QFileSystemModel*>(sourceModel());
    if (!model) {
        // Fallback to default if source model is not a QFileSystemModel
        return QSortFilterProxyModel::lessThan(source_left, source_right);
    }

    QFileInfo leftFileInfo(model->filePath(source_left));
    QFileInfo rightFileInfo(model->filePath(source_right));

    // Primary sort: Folders before files
    if (leftFileInfo.isDir() && !rightFileInfo.isDir()) {
        return true; // Left (folder) comes before right (file)
    }
    if (!leftFileInfo.isDir() && rightFileInfo.isDir()) {
        return false; // Left (file) comes after right (folder)
    }

    // Secondary sort: If both are dirs or both are files, sort by the current sort column's data
    // QSortFilterProxyModel handles the actual data comparison based on sortColumn() and sortOrder()
    // and its own lessThan implementation for standard types if we don't override further.
    // For basic alphabetical sorting by name (column 0) when types are the same:
    if (leftFileInfo.isDir() == rightFileInfo.isDir()) { // Both are folders or both are files
        // Use QFileInfo::fileName() for case-insensitive comparison if desired,
        // or rely on the base class to compare based on the model's data for the sort column.
        // The default QSortFilterProxyModel::lessThan will compare the data provided by
        // QFileSystemModel::data() for the sortColumn().
        // For QFileSystemModel, column 0 (Name) data is QString.
        return QSortFilterProxyModel::lessThan(source_left, source_right);
    }

    // Fallback for any other unexpected case (should not be reached with above logic)
    return QSortFilterProxyModel::lessThan(source_left, source_right);
}
