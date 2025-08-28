QT += core gui widgets svg

TARGET = yawt
VERSION = 0.9.1

CONFIG += c++17
CONFIG += warn_on
CONFIG += thread
CONFIG -= app_bundle  # optional: skip .app bundle on macOS

# Add new include paths for refactored source layout
INCLUDEPATH += include
INCLUDEPATH += src
INCLUDEPATH += src/gui
INCLUDEPATH += src/gui/widgets
INCLUDEPATH += src/gui/delegates
INCLUDEPATH += src/models
INCLUDEPATH += src/core
INCLUDEPATH += src/core/processing
INCLUDEPATH += src/data
INCLUDEPATH += src/utils

# Include OpenCV
INCLUDEPATH += /opt/homebrew/include/opencv4
LIBS += -L/opt/homebrew/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio

SOURCES += \
    main.cpp \
    \
    # Models
    src/models/annotationtablemodel.cpp \
    src/models/blobtablemodel.cpp \
    src/models/folderfirstsortproxymodel.cpp \
    \
    # GUI (top-level)
    src/gui/mainwindow.cpp \
    src/gui/trackingprogressdialog.cpp \
    \
    # GUI widgets
    src/gui/widgets/mergeviewer.cpp \
    src/gui/widgets/miniloader.cpp \
    src/gui/widgets/cachestatuswidget.cpp \
    src/gui/widgets/videofiletreeview.cpp \
    src/gui/widgets/videoloader.cpp \
    \
    # GUI delegates
    src/gui/delegates/colordelegate.cpp \
    src/gui/delegates/itemtypedelegate.cpp \
    \
    # Core tracking
    src/core/trackingmanager.cpp \
    src/core/wormtracker.cpp \
    \
    # Core processing
    src/core/processing/videoprocessor.cpp \
    src/core/processing/frameloader_internal.cpp \
    \
    # Data
    src/data/trackingdatastorage.cpp \
    src/data/trackingcommon.cpp \
    src/data/wormobject.cpp \
    \
    # Utils
    src/utils/debugutils.cpp \
    src/utils/thresholdingutils.cpp

HEADERS += \
    \
    # Models
    src/models/annotationtablemodel.h \
    src/models/blobtablemodel.h \
    src/models/folderfirstsortproxymodel.h \
    \
    # GUI
    src/gui/mainwindow.h \
    src/gui/trackingprogressdialog.h \
    \
    # GUI widgets
    src/gui/widgets/mergeviewer.h \
    src/gui/widgets/miniloader.h \
    src/gui/widgets/cachestatuswidget.h \
    src/gui/widgets/videofiletreeview.h \
    src/gui/widgets/videoloader.h \
    \
    # GUI delegates
    src/gui/delegates/colordelegate.h \
    src/gui/delegates/itemtypedelegate.h \
    \
    # Core
    src/core/trackingmanager.h \
    src/core/wormtracker.h \
    \
    # Core processing
    src/core/processing/videoprocessor.h \
    src/core/processing/frameloader_internal.h \
    \
    # Data
    src/data/trackingdatastorage.h \
    src/data/trackingcommon.h \
    src/data/wormobject.h \
    \
    # Utils
    src/utils/debugutils.h \
    src/utils/thresholdingutils.h

FORMS += \
    src/gui/mainwindow.ui \
    src/gui/trackingprogressdialog.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

# Optional: enable debug info
CONFIG += debug

RESOURCES += \
    resources.qrc

RC_ICONS = resources/icon_tr.ico
