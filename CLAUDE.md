# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

YAWT (Yet Another Worm Tracker) is a Qt6-based desktop application for tracking C. elegans worm behavior in video recordings. The application processes chemotaxis experiments by starting from a keyframe where all worms are visible and tracking them both forward and backward through the video timeline.

## Build System

This project supports both CMake and qmake build systems:

**CMake (Primary):**
```bash
# Debug build using convenience script
./zed_build_debug.sh

# Manual CMake build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=$(qtpaths --install-prefix)
make -j$(sysctl -n hw.logicalcpu)
```

**qmake (Alternative):**
```bash
# Using qtbuildtool script
./qtbuildtool.sh yawt.pro

# Manual qmake build
cd build
qmake ../yawt.pro
make
```

**Dependencies:**
- Qt6 (Core, Gui, Widgets, Svg)
- OpenCV (installed at /opt/homebrew on macOS)
- C++17 standard

## Core Architecture

The application follows a layered architecture with these key components:

**Main GUI Layer:**
- `MainWindow` - Primary UI controller handling video display, user interactions, and tool modes
- `mainwindow.ui` - Qt Designer UI definition
- `TrackingProgressDialog` - Progress monitoring during tracking operations

**Video Processing Layer:**
- `VideoLoader` - Handles video file loading, frame display, and user interaction modes (pan, ROI, crop, edit)
- `VideoProcessor` - Processes video chunks for thresholding and frame preparation
- `VideoFileTreeView` - Custom tree view for video file selection

**Tracking Engine:**
- `TrackingManager` - Coordinates the overall tracking process, manages multiple worm trackers
- `WormTracker` - Individual tracker for a single worm, handles forward/backward tracking from keyframe
- `WormObject` - Represents a tracked worm with its complete trajectory over time

**Computer Vision:**
- `thresholdingutils.h/.cpp` - OpenCV-based thresholding algorithms for blob detection
- `trackingcommon.h` - Shared data structures (`DetectedBlob`, `InitialWormInfo`, `TrackerState`)

**UI Components:**
- `BlobTableModel` - Model for displaying detected blobs in table views
- `ColorDelegate`, `ItemTypeDelegate` - Custom table cell renderers
- `FolderFirstSortProxyModel` - File system sorting utilities

## Key Data Flow

1. User selects video and sets keyframe where all worms are visible
2. User marks initial worm positions as ROIs on the keyframe
3. `TrackingManager` creates forward and backward `WormTracker` instances for each worm
4. Each `WormTracker` processes frames in its direction, maintaining tracking state
5. Trackers handle merge/split events when worms interact
6. Final tracks are consolidated and can be exported to CSV

## Tracking States

The system handles complex worm interactions through state management:
- `TrackingNormally` - Standard single-worm tracking
- `TrackingAsMerged` - Worm is merged with others, tracking the combined blob
- `PausedForSplit` - Merged worms have split, awaiting target selection
- `Lost` - Tracking failed, attempting recovery

## Development Notes

- The project uses Qt's signal-slot system extensively for component communication
- Threading is used for video processing and individual worm tracking
- OpenCV is used for all computer vision operations (thresholding, blob detection)
- The application supports both macOS app bundles and cross-platform builds
- Frame-specific merge/split handling uses unique blob IDs and IoU calculations