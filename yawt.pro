QT += core gui widgets svg

CONFIG += c++17
CONFIG += warn_on
CONFIG += thread
CONFIG -= app_bundle  # optional: skip .app bundle on macOS

INCLUDEPATH += $$PWD/widgets
INCLUDEPATH += $$PWD/tracking
INCLUDEPATH += $$PWD/utilities

# Include OpenCV
INCLUDEPATH += /opt/homebrew/include/opencv4
LIBS += -L/opt/homebrew/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    tracking/trackingcommon.cpp \
    tracking/trackingmanager.cpp \
    tracking/wormobject.cpp \
    tracking/wormtracker.cpp \
    utilities/itemtypedelegate.cpp \
    utilities/folderfirstsortproxymodel.cpp \
    utilities/videoprocessor.cpp \
    utilities/wormtablemodel.cpp \
    widgets/trackingprogressdialog.cpp \
    widgets/videofiletreeview.cpp \
    widgets/videoloader.cpp

HEADERS += \
    mainwindow.h \
<<<<<<< HEAD
    tracking/trackdata.h \
    tracking/trackeditemdata.h \
    tracking/trackingcommon.h \
    tracking/trackingmanager.h \
    tracking/wormobject.h \
    tracking/wormtracker.h \
    utilities/folderfirstsortproxymodel.h \
    utilities/itemtypedelegate.h \
    utilities/videoprocessor.h \
    utilities/wormtablemodel.h \
    widgets/trackingprogressdialog.h \
    widgets/videofiletreeview.h \
    widgets/videoloader.h
=======
    trackingcommon.h \
    trackingmanager.h \
    trackingprogressdialog.h \
    videofiletreeview.h \
    videoloader.h \
    videoprocessor.h \
    wormobject.h \
    wormtablemodel.h \
    wormtracker.h
>>>>>>> devel

FORMS += \
    mainwindow.ui \
    widgets/trackingprogressdialog.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


# Optional: enable debug info
CONFIG += debug

RESOURCES += \
    resources.qrc
