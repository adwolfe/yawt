#include "videofiletreeview.h"
#include "folderfirstsortproxymodel.h" // <<< Include your new proxy model
#include <QHeaderView>
#include <QTimer>
#include <QDebug>
#include "../../utils/loggingcategories.h"
#include <QFileSystemModel> // <<< Make sure this is included
#include <QFileInfo>

VideoFileTreeView::VideoFileTreeView(QWidget *parent) : QTreeView(parent) {
    // Initialize the source file system model
    fileSystemModel = new QFileSystemModel(this); // Parent 'this' for auto-deletion

    videoFileFilters << "*.mp4" << "*.avi" << "*.mov" << "*.mkv"
                     << "*.wmv" << "*.flv" << "*.webm" << "*.mpg"
                     << "*.mpeg" << "*.m4v" << "*.3gp";

    fileSystemModel->setFilter(QDir::AllDirs | QDir::Files | QDir::NoDotAndDotDot);
    fileSystemModel->setNameFilters(videoFileFilters);
    fileSystemModel->setNameFilterDisables(false); // Apply filters to files, not just disable non-matching dirs

    // Initialize the proxy model
    proxyModel = new FolderFirstSortProxyModel(this); // Parent 'this' for auto-deletion
    proxyModel->setSourceModel(fileSystemModel);

    // Set the proxy model to the view
    setModel(proxyModel);

    // --- UI Customizations ---
    header()->setSectionResizeMode(0, QHeaderView::Stretch); // For the first visible column
    setAnimated(false); // Preference
    setIndentation(15); // Adjust indentation as needed

    // Enable sorting on the TreeView - proxyModel will handle the logic
    setSortingEnabled(true);
    sortByColumn(0, Qt::AscendingOrder); // Sort by name (column 0)

    // Hide columns (indices are for the view, after proxy)
    // QFileSystemModel default columns: 0:Name, 1:Size, 2:Type, 3:Date Modified
    // We want to show only Name, so hide Size, Type, Date Modified
    setColumnHidden(1, true); // Hide Size
    setColumnHidden(2, true); // Hide Type/Kind
    setColumnHidden(3, true); // Hide Date Modified

    // --- Connect signals ---
    connect(this, &QTreeView::doubleClicked, this, &VideoFileTreeView::onItemDoubleClicked);
}

VideoFileTreeView::~VideoFileTreeView() {
    // No need to delete fileSystemModel or proxyModel explicitly
    // as they are children of this QWidget and Qt will manage their memory.
}

void VideoFileTreeView::setRootDirectory(const QString &path) {
    QString effectivePath = path;
    QDir dir(path);
    if (path.isEmpty() || !dir.exists()) {
        YAWT_WARN(lcGuiVideoLoader) << "VideoFileTreeView: Root path '" << path
                                    << "' is invalid or does not exist. Defaulting to home directory.";
        effectivePath = QDir::homePath();
    }

    // Set root path on the source model
    QModelIndex sourceRootIndex = fileSystemModel->setRootPath(effectivePath);

    // Map the source model's root index to the proxy model and set it on the view
    if (proxyModel) { // Ensure proxyModel is initialized
        setRootIndex(proxyModel->mapFromSource(sourceRootIndex));
    } else {
        YAWT_WARN(lcGuiVideoLoader) << "VideoFileTreeView: Proxy model not initialized. Cannot set root index.";
        // Fallback or error handling if proxy model somehow isn't set
        // This shouldn't happen if constructor logic is correct.
        setRootIndex(QModelIndex()); // Set an invalid index
    }


    // Optional: Expand all items after setting root path.
    // Consider performance implications for very large directories.
    // QTimer::singleShot(0, this, &QTreeView::expandAll);
    // if (proxyModel && proxyModel->rowCount(rootIndex()) > 0) {
    //     scrollTo(proxyModel->index(0, 0, rootIndex()), QAbstractItemView::PositionAtTop);
    // }
}

void VideoFileTreeView::onItemDoubleClicked(const QModelIndex &proxyIndex) {
    // The 'proxyIndex' is from the proxyModel (the view's model)
    if (!proxyIndex.isValid() || !proxyModel || !fileSystemModel) {
        return;
    }

    // Map the proxy index to the source model index (QFileSystemModel)
    QModelIndex sourceIndex = proxyModel->mapToSource(proxyIndex);

    if (!sourceIndex.isValid()) { // Check if mapping was successful
        return;
    }

    // Now query properties using the sourceIndex and fileSystemModel
    bool isDir = fileSystemModel->isDir(sourceIndex);
    if (isDir) {
        QString dirPath = fileSystemModel->filePath(sourceIndex);
        QFileInfo dirInfo(dirPath);
        if (dirInfo.fileName().startsWith("PROC_")) {
            QDir procDir(dirPath);
            QString wormsPath = procDir.absoluteFilePath("worms.json");
            QString thresholdPath = procDir.absoluteFilePath("thresholding.json");
            QStringList trackFiles = procDir.entryList(QStringList() << "*_tracks.csv" << "*_tracks.xlsx", QDir::Files);
            if (QFileInfo::exists(wormsPath) && QFileInfo::exists(thresholdPath) && !trackFiles.isEmpty()) {
                YAWT_INFO(lcGuiVideoLoader) << "PROC directory double-clicked:" << dirPath;
                emit runDirectoryDoubleClicked(dirPath);
            }
        }
        return;
    }

    QString filePath = fileSystemModel->filePath(sourceIndex);
    if (!filePath.isEmpty()) {
        YAWT_INFO(lcGuiVideoLoader) << "File double-clicked (via proxy):" << filePath;
        emit videoFileDoubleClicked(filePath);
    }
}
