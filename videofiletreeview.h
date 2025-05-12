#ifndef VIDEOFILETREEVIEW_H
#define VIDEOFILETREEVIEW_H

#include <QTreeView>
#include <QFileSystemModel> // Required for the file system model
#include <QStringList>      // For storing video file filters
#include <QDir>             // For QDir constants and QDir::homePath()
#include <QModelIndex>      // For signal parameters

// Forward declaration
class QFileSystemModel;

/**
 * @brief The VideoFileTreeView class provides a view for browsing directory structures,
 * displaying only subdirectories and specified video files.
 * All subdirectories are expanded by default upon setting a root directory.
 * Emits a signal when a video file is double-clicked.
 */
class VideoFileTreeView : public QTreeView {
    Q_OBJECT

public:
    /**
     * @brief Constructs a VideoFileTreeView widget.
     * @param parent The parent widget, default is nullptr.
     */
    explicit VideoFileTreeView(QWidget *parent = nullptr);

    /**
     * @brief Destructor.
     */
    ~VideoFileTreeView();

    /**
     * @brief Sets the root directory to be displayed in the tree view.
     * If the path is invalid or does not exist, it defaults to the user's home directory.
     * After setting the root path, all directories will be expanded.
     * @param path The absolute path to the root directory.
     */
    void setRootDirectory(const QString &path);

signals:
    /**
     * @brief This signal is emitted when a file (not a directory) in the tree view
     * is double-clicked.
     * @param filePath The absolute path to the double-clicked file.
     */
    void videoFileDoubleClicked(const QString &filePath);

private slots:
    /**
     * @brief Handles the doubleClicked signal from the QTreeView.
     * Checks if the item is a file and emits videoFileDoubleClicked if so.
     * @param index The model index of the item that was double-clicked.
     */
    void onItemDoubleClicked(const QModelIndex &index);

private:
    QFileSystemModel *fileSystemModel; // Model to interact with the file system
    QStringList videoFileFilters;      // List of extensions for video files (e.g., "*.mp4", "*.avi")
};

#endif // VIDEOFILETREEVIEW_H
