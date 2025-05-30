QT += core gui widgets svg

CONFIG += c++17
CONFIG += warn_on
CONFIG += thread
CONFIG -= app_bundle  # optional: skip .app bundle on macOS

INCLUDEPATH += include

# Include OpenCV
INCLUDEPATH += /opt/homebrew/include/opencv4
LIBS += -L/opt/homebrew/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio

SOURCES += \
    blobtablemodel.cpp \
    colordelegate.cpp \
    folderfirstsortproxymodel.cpp \
    itemtypedelegate.cpp \
    main.cpp \
    mainwindow.cpp \
    trackingcommon.cpp \
    trackingmanager.cpp \
    trackingprogressdialog.cpp \
    videofiletreeview.cpp \
    videoloader.cpp \
    videoprocessor.cpp \
    wormobject.cpp \
    wormtracker.cpp

HEADERS += \
    blobtablemodel.h \
    colordelegate.h \
    folderfirstsortproxymodel.h \
    itemtypedelegate.h \
    mainwindow.h \
    trackingcommon.h \
    trackingmanager.h \
    trackingprogressdialog.h \
    videofiletreeview.h \
    videoloader.h \
    videoprocessor.h \
    wormobject.h \
    wormtracker.h

FORMS += \
    mainwindow.ui \
    trackingprogressdialog.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


# Optional: enable debug info
CONFIG += debug

RESOURCES += \
    resources.qrc
