cmake_minimum_required(VERSION 3.16)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(yawt LANGUAGES CXX VERSION 0.9.7)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable automatic processing of Qt files
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Make sure the top-level source dir and new src/ tree are visible to the compiler
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/gui
    ${CMAKE_SOURCE_DIR}/src/gui/widgets
    ${CMAKE_SOURCE_DIR}/src/gui/delegates
    ${CMAKE_SOURCE_DIR}/src/models
    ${CMAKE_SOURCE_DIR}/src/core
    ${CMAKE_SOURCE_DIR}/src/core/processing
    ${CMAKE_SOURCE_DIR}/src/data
    ${CMAKE_SOURCE_DIR}/src/utils
)

# Find Qt modules
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Svg)

# Include OpenCV
# Prefer standard OpenCV discovery; allow overriding via -DOpenCV_DIR
find_package(OpenCV REQUIRED)

# Optional: Basler Pylon SDK
# Install the SDK from https://www.baslerweb.com/en/downloads/software-downloads/
# Point CMake at it with:
#   cmake -Dpylon_DIR=/Library/Frameworks/pylon.framework/Versions/Current/lib/cmake/pylon ..
find_package(pylon QUIET)
if(pylon_FOUND)
    message(STATUS "Pylon SDK found — Basler camera support enabled")
else()
    message(STATUS "Pylon SDK not found — Basler camera support disabled")
endif()

# List of sources (updated to reflect new organization under src/)
set(SOURCES
    main.cpp

    # Models
    src/models/annotationtablemodel.cpp
    src/models/blobtablemodel.cpp
    src/models/folderfirstsortproxymodel.cpp
    src/models/itemtypefilterproxymodel.cpp

    # GUI (top-level)
    src/gui/mainwindow.cpp
    src/gui/analysisdialog.cpp
    src/gui/analysisgroupwidgets.cpp
    src/gui/analysissessionmodel.cpp
    src/gui/analysispanel.cpp
    src/gui/scaledialog.cpp
    src/gui/trackingprogressdialog.cpp

    # GUI widgets and delegates
    src/gui/widgets/mergeviewer.cpp
    src/gui/widgets/miniloader.cpp
    src/gui/widgets/wormtimeline.cpp
   # src/gui/widgets/cachestatuswidget.cpp
    src/gui/widgets/videofiletreeview.cpp
    src/gui/widgets/frameloader.cpp
    src/gui/widgets/videoloader.cpp
    src/gui/widgets/liveview.cpp
    src/gui/widgets/opencvcamerasource.cpp
    src/gui/widgets/pyloncamerasource.cpp
    src/gui/widgets/cameraenumerator.cpp
    src/gui/widgets/captureworker.cpp
    src/gui/widgets/capturepanel.cpp

    src/gui/delegates/colordelegate.cpp
    src/gui/delegates/itemtypedelegate.cpp

    # Core tracking
    src/core/trackingmanager.cpp
    src/core/wormtracker.cpp
    src/core/centerlineworker.cpp
    src/core/centerlineprocessor.cpp
    src/core/centerlinegeometry.cpp
    src/core/appcontroller.cpp

    # Core processing
    src/core/processing/videoprocessor.cpp
 #   src/core/processing/frameloader_internal.cpp

    # Data
    src/data/trackingdatastorage.cpp
    src/data/trackingcommon.cpp
    src/data/wormobject.cpp
    src/data/videometadatastore.cpp

    # Debug
    src/debug/debugdatastore.cpp
    src/debug/debugexporter.cpp

    # Utils
    src/utils/debugutils.cpp
    src/utils/thresholdingutils.cpp
)

# List of headers (updated to new locations)
set(HEADERS
    # Models
    src/models/annotationtablemodel.h
    src/models/blobtablemodel.h
    src/models/folderfirstsortproxymodel.h
    src/models/itemtypefilterproxymodel.h

    # GUI
    src/gui/mainwindow.h
    src/gui/analysisdialog.h
    src/gui/analysisgroupwidgets.h
    src/gui/analysissessionmodel.h
    src/gui/analysispanel.h
    src/gui/scaledialog.h
    src/gui/trackingprogressdialog.h

    # GUI widgets / delegates
    src/gui/widgets/mergeviewer.h
    src/gui/widgets/miniloader.h
    src/gui/widgets/wormtimeline.h
  #  src/gui/widgets/cachestatuswidget.h
    src/gui/widgets/videofiletreeview.h
    src/gui/widgets/frameloader.h
    src/gui/widgets/videoloader.h
    src/gui/widgets/liveview.h
    src/gui/widgets/icamerasource.h
    src/gui/widgets/opencvcamerasource.h
    src/gui/widgets/pyloncamerasource.h
    src/gui/widgets/cameraenumerator.h
    src/gui/widgets/captureworker.h
    src/gui/widgets/capturepanel.h

    src/gui/delegates/colordelegate.h
    src/gui/delegates/itemtypedelegate.h

    # Core
    src/core/trackingmanager.h
    src/core/wormtracker.h
    src/core/centerlinetypes.h
    src/core/centerlineworker.h
    src/core/centerlineprocessor.h
    src/core/centerlinegeometry.h
    src/core/appcontroller.h

    # Core processing
    src/core/processing/videoprocessor.h
#    src/core/processing/frameloader_internal.h

    # Data
    src/data/trackingdatastorage.h
    src/data/trackingcommon.h
    src/data/wormobject.h
    src/data/videometadatastore.h

    # Debug
    src/debug/debugrecords.h
    src/debug/debugdatastore.h
    src/debug/debugexporter.h

    # Utils
    src/utils/debugutils.h
    src/utils/thresholdingutils.h
)

# UI files (moved)
set(UIS
    src/gui/mainwindow.ui
    src/gui/trackingprogressdialog.ui
)

# Resources
set(RESOURCES
    resources.qrc
)

# macOS app bundle icon (keep same behavior as before)
set(MACOSX_BUNDLE_ICON_FILE icon.icns)
set_source_files_properties(${CMAKE_SOURCE_DIR}/resources/icon.icns PROPERTIES
    MACOSX_PACKAGE_LOCATION "Resources")

set(APP_ICON_MAC ${CMAKE_SOURCE_DIR}/resources/icon.icns)

# Create the executable
if(WIN32)
    set(APP_ICON_RESOURCE_WINDOWS resources/icon_tr.ico)
    add_executable(${PROJECT_NAME} WIN32 ${SOURCES} ${HEADERS} ${UIS} ${RESOURCES} ${APP_ICON_RESOURCE_WINDOWS})
else()
    add_executable(${PROJECT_NAME} MACOSX_BUNDLE
        ${SOURCES} ${HEADERS} ${UIS} ${RESOURCES} ${APP_ICON_MAC}
    )
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_INFO_PLIST ${CMAKE_SOURCE_DIR}/Info.plist
)

# Link Qt and OpenCV
target_link_libraries(${PROJECT_NAME}
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Svg
    ${OpenCV_LIBS}
)

# Link Pylon if found
if(pylon_FOUND)
    target_compile_definitions(${PROJECT_NAME} PRIVATE YAWT_HAS_PYLON)
    target_link_libraries(${PROJECT_NAME} pylon::pylon)
endif()

# Generate version header into the build directory
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/version.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/version.h"
    @ONLY
)
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# Optional: install target path (similar to .pro file behavior)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# Custom deployment target for macOS app bundle
if(APPLE)
    add_custom_target(deploy
        COMMAND ${CMAKE_SOURCE_DIR}/scripts/package_mac.sh -n --no-archive --no-macdeployqt
        DEPENDS ${PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Deploying macOS app bundle with all dependencies"
    )

    # Additional bundle properties for better deployment
    set_target_properties(${PROJECT_NAME} PROPERTIES
        MACOSX_BUNDLE_BUNDLE_NAME "YAWT"
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
        MACOSX_BUNDLE_COPYRIGHT "Copyright © 2024"
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.sciwolfe.yawt"
    )
endif()
