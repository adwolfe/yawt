cmake_minimum_required(VERSION 3.16)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(yawt LANGUAGES CXX VERSION 0.9.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable automatic processing of Qt files
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)



# Add source directory includes
include_directories(${CMAKE_SOURCE_DIR})

# Find Qt modules
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Svg)

# Include OpenCV
find_package(OpenCV REQUIRED PATHS /opt/homebrew)

# List of sources
set(SOURCES
    annotationtablemodel.cpp
    blobtablemodel.cpp
    colordelegate.cpp
    debugutils.cpp
    folderfirstsortproxymodel.cpp
    #framecachemanager.cpp
    itemtypedelegate.cpp
    main.cpp
    mainwindow.cpp
    minivideoloader.cpp
    thresholdingutils.cpp
    trackingcommon.cpp
    trackingdatastorage.cpp
    trackingmanager.cpp
    trackingprogressdialog.cpp
    videofiletreeview.cpp
    videoloader.cpp
    videoprocessor.cpp
    wormobject.cpp
    wormtracker.cpp
)

# List of headers
set(HEADERS
    annotationtablemodel.h
    blobtablemodel.h
    colordelegate.h
    folderfirstsortproxymodel.h
    #framecachemanager.h
    itemtypedelegate.h
    mainwindow.h
    minivideoloader.h
    thresholdingutils.h
    trackingcommon.h
    trackingdatastorage.h
    trackingmanager.h
    trackingprogressdialog.h
    videofiletreeview.h
    videoloader.h
    videoprocessor.h
    wormobject.h
    wormtracker.h
)

# UI files
set(UIS
    mainwindow.ui
    trackingprogressdialog.ui
)

# Resources
set(RESOURCES
    resources.qrc
)
# macOS app bundle icon
set(MACOSX_BUNDLE_ICON_FILE icon.icns)
set_source_files_properties(${CMAKE_SOURCE_DIR}/resources/icon.icns PROPERTIES
    MACOSX_PACKAGE_LOCATION "Resources")

set(APP_ICON_MAC ${CMAKE_SOURCE_DIR}/resources/icon.icns)
# Optional: Windows .ico icon resource
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
        COMMAND ${CMAKE_SOURCE_DIR}/deploy_mac.sh
        DEPENDS ${PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
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
