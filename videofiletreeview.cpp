#include "videofiletreeview.h"
#include <QHeaderView>     // For customizing the header (e.g., resizing columns)
#include <QTimer>          // For QTimer::singleShot to delay expandAll
#include <QDebug>          // For warning messages and debugging output

VideoFileTreeView::VideoFileTreeView(QWidget *parent) : QTreeView(parent) {
    // Initialize the file system model
    fileSystemModel = new QFileSystemModel(this);

    // Define common video file extensions
    videoFileFilters << "*.mp4" << "*.avi" << "*.mov" << "*.mkv"
                     << "*.wmv" << "*.flv" << "*.webm" << "*.mpg"
                     << "*.mpeg" << "*.m4v" << "*.3gp";

    // Configure the model:
    fileSystemModel->setFilter(QDir::AllDirs | QDir::Files | QDir::NoDotAndDotDot);
    fileSystemModel->setNameFilters(videoFileFilters);
    fileSystemModel->setNameFilterDisables(false);

    setModel(fileSystemModel);

    // --- UI Customizations ---
    header()->setSectionResizeMode(0, QHeaderView::Stretch);
    setAnimated(false);
    setSortingEnabled(true);
    sortByColumn(0, Qt::AscendingOrder);

    // --- Connect signals ---
    // Connect the QTreeView's doubleClicked signal to our custom slot
    connect(this, &QTreeView::doubleClicked, this, &VideoFileTreeView::onItemDoubleClicked);
}

VideoFileTreeView::~VideoFileTreeView() {
    // fileSystemModel is child of this, Qt handles deletion
}

void VideoFileTreeView::setRootDirectory(const QString &path) {
    QString effectivePath = path;
    if (path.isEmpty() || !QDir(path).exists()) {
        qWarning() << "VideoFileTreeView: Root path '" << path
                   << "' is invalid or does not exist. Defaulting to home directory.";
        effectivePath = QDir::homePath();
    }

    QModelIndex rootModelIndex = fileSystemModel->setRootPath(effectivePath);
    setRootIndex(rootModelIndex);


    // Expands directories
    //QTimer::singleShot(0, this, [this]() {
    //    this->expandAll();
    //    if (this->model() && this->model()->rowCount(this->rootIndex()) > 0) {
    //        this->scrollTo(this->model()->index(0, 0, this->rootIndex()), QAbstractItemView::PositionAtTop);
    //    }
    //});
}

void VideoFileTreeView::onItemDoubleClicked(const QModelIndex &index) {
    // Check if the model and index are valid
    if (!index.isValid() || !fileSystemModel) {
        return;
    }

    // Check if the double-clicked item is a directory
    bool isDir = fileSystemModel->isDir(index);
    if (isDir) {
        // If it's a directory, we don't want to emit the signal.
        // The default QTreeView behavior (expand/collapse) will occur.
        return;
    }

    // If it's not a directory, it's a file. Get its path.
    QString filePath = fileSystemModel->filePath(index);

    // Check if the file path is not empty (it shouldn't be for a valid file index)
    if (!filePath.isEmpty()) {
        qDebug() << "File double-clicked:" << filePath; // For debugging
        emit videoFileDoubleClicked(filePath); // Emit the signal
    }
}
