#ifndef VIDEOFILETREEVIEW_H
#define VIDEOFILETREEVIEW_H

#include <QTreeView>
#include <QStringList>
#include <QDir>
#include <QModelIndex>

// Forward declarations
class QFileSystemModel;
class FolderFirstSortProxyModel; // <<< Added forward declaration

/**
 * @brief The VideoFileTreeView class provides a view for Browse directory structures,
 * displaying only subdirectories and specified video files.
 * Sorts folders before files.
 * Emits a signal when a video file is double-clicked.
 */
class VideoFileTreeView : public QTreeView {
    Q_OBJECT

public:
    explicit VideoFileTreeView(QWidget *parent = nullptr);
    ~VideoFileTreeView();

    void setRootDirectory(const QString &path);

signals:
    void videoFileDoubleClicked(const QString &filePath);

private slots:
    void onItemDoubleClicked(const QModelIndex &index);

private:
    QFileSystemModel *fileSystemModel;         // Source model
    FolderFirstSortProxyModel *proxyModel;     // Proxy model for custom sorting
    QStringList videoFileFilters;
};

#endif // VIDEOFILETREEVIEW_H
