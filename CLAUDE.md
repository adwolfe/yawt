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
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=$(qtpaths --install-prefix)
make -j$(sysctl -n hw.logicalcpu)
```

**qmake (Alternative):**
```bash
# Using qtbuildtool script
./qtbuildtool.sh yawt.pro

# Manual qmake build
mkdir -p build
cd build
qmake ../yawt.pro
make
```

**Dependencies:**
- Qt6 (Core, Gui, Widgets, Svg)
- OpenCV (installed at /opt/homebrew on macOS)
- C++17 standard

**Build Scripts:**
- `zed_build_debug.sh` - Debug build convenience script for Zed editor
- `zed_launch.sh` - Launch script for debug mode
- `qtbuildtool.sh` - qmake build convenience script

## Core Architecture

The application follows a layered architecture with these key components. Recent refactors introduced an application controller to isolate non-UI logic from the view layer.

**Main GUI Layer:**
- `MainWindow` - Primary UI, owns widgets and view-specific behavior (video display, menus, tool modes, dialogs parenting). After the refactor MainWindow delegates core state and tracking orchestration to `AppController`.
- `mainwindow.ui` - Qt Designer UI definition
- `TrackingProgressDialog` - Progress/preview dialog for tracking operations. This dialog can be created and owned by the `AppController` (controller-owned modal or modeless flows are supported) so UI wiring and orchestration are handled outside MainWindow.

**Application Controller (new):**
- `AppController` (located in `src/core`) — Non-UI controller introduced to own and coordinate core components:
  - Creates and owns `TrackingDataStorage`, `TrackingManager`, `BlobTableModel`, and `AnnotationTableModel`.
  - Exposes model accessors for views to bind to (so MainWindow can set view models from the controller).
  - Provides high-level APIs for model manipulation and tracking orchestration (e.g., `beginTrackingFromModel`, `requestStartTracking`, `showTrackingDialog`, `cancelTracking`).
  - Owns or orchestrates the `TrackingProgressDialog` (optional; MainWindow may still parent the dialog if desired), wires dialog signals to the `TrackingManager`, and forwards progress/status signals for UI updates.

**Video Processing Layer:**
- `VideoLoader` - Handles video file loading, frame display, and user interaction modes (pan, ROI, crop, edit)
- `VideoProcessor` - Processes video chunks for thresholding and frame preparation
- `VideoFileTreeView` - Custom tree view for video file selection

**Tracking Engine:**
- `TrackingManager` - Coordinates the overall tracking process and manages multiple worm trackers; remains responsible for multi-threaded tracking runs
- `WormTracker` - Individual tracker for a single worm, handles forward/backward tracking from the keyframe
- `WormObject` - Represents a tracked worm with its trajectory over time
- `TrackingDataStorage` - Central data storage for all blob and track data, serves as the single source of truth (now primarily created/owned by `AppController`)

**Computer Vision:**
- `thresholdingutils.h/.cpp` - OpenCV-based thresholding algorithms for blob detection
- `trackingcommon.h/.cpp` - Shared data structures (`DetectedBlob`, `InitialWormInfo`, `TrackerState`)

**UI Components:**
- `BlobTableModel` - Model for displaying detected blobs in table views (now created by `AppController`)
- `AnnotationTableModel` - Model for annotations derived from storage (created by `AppController`)
- `ColorDelegate`, `ItemTypeDelegate` - Custom table cell renderers
- `FolderFirstSortProxyModel` - File system sorting utilities

## Key Data Flow

0. The UI (MainWindow) obtains models and the central storage from `AppController`. The controller owns the core models and the `TrackingManager`.
1. User selects a video and sets the keyframe where all worms are visible.
2. User marks initial worm positions as ROIs on the keyframe; these are stored in `TrackingDataStorage` via `BlobTableModel`.
3. The UI requests tracking via `AppController` (either by calling `beginTrackingFromModel(...)`, `requestStartTracking(...)`, or by asking the controller to `showTrackingDialog(...)`). The controller builds the initial worm list from the blob model (optionally filtering already-tracked worms).
4. `TrackingManager` (owned or created by `AppController`) creates forward and backward `WormTracker` instances for each selected worm and starts the multi-threaded tracking process.
5. Each `WormTracker` processes frames in its direction, maintaining per-worm tracking state and emitting progress/status/tracking events.
6. Trackers detect and handle merge/split events when worms interact; `TrackingManager` emits consolidated tracks as they are available.
7. `AppController` stores received tracks into `TrackingDataStorage` and re-emits `tracksUpdated` so UI components (VideoLoader, annotation views, mini-loaders) can refresh from the single source of truth.
8. Final tracks can be exported to CSV or used to populate overlays and annotation tables.

## Tracking States

The system handles complex worm interactions through state management:
- `TrackingNormally` - Standard single-worm tracking
- `TrackingAsMerged` - Worm is merged with others, tracking the combined blob
- `PausedForSplit` - Merged worms have split, awaiting target selection
- `Lost` - Tracking failed, attempting recovery

## Development Notes

- The project uses Qt's signal-slot system extensively for component communication; many cross-thread signals rely on registered Qt meta-types.
- Threading is used for video processing and for per-worm tracking tasks (the `TrackingManager` coordinates worker threads).
- OpenCV is used for all computer vision operations (thresholding, blob detection).
- The application supports both macOS app bundles and cross-platform builds.
- Frame-specific merge/split handling uses unique blob IDs and IoU calculations; merge/split histories and resolution are stored in `TrackingDataStorage`.
- Central data storage (`TrackingDataStorage`) remains the single source of truth for blobs, ROIs, tracks, merge groups, and annotations. `AppController` now creates and owns this storage (unless an external storage is injected).
- AppController responsibilities (recent refactor):
  - Creates and owns `TrackingDataStorage`, `TrackingManager`, `BlobTableModel`, and `AnnotationTableModel`.
  - Exposes model getters so the GUI (MainWindow) can bind views directly to models supplied by the controller.
  - Provides high-level APIs: `requestStartTracking(...)`, `beginTrackingFromModel(...)`, `cancelTracking()`, `showTrackingDialog(...)`, and model-manipulation helpers like `addBlobFromVideo(...)`, `addRoi(...)`, `removeAllBlobs()`, and `setRoiSizeMultiplier(...)`.
  - Forwards `TrackingManager` signals (progress, status, finished, failed, cancelled) as controller-level signals suitable for UI consumption.
  - Optionally owns the `TrackingProgressDialog` and performs the dialog orchestration (create, connect, execute, and start tracking when the dialog requests it). This reduces MainWindow responsibilities and keeps non-UI logic centralized.
- MainWindow responsibilities after refactor:
  - Own and manage UI widgets (VideoLoader, MiniLoader instances, table views, delegates) and remain the top-level parent for dialog windows if desired.
  - Request high-level operations from `AppController` and bind UI views to models provided by the controller.
  - Keep only view-layer logic (selection handling, auto-centering, playback controls, window/layout behavior).
- Meta-type registration: `TrackingManager` registers required meta-types at construction time; ensure any cross-thread signal emission occurs after those registrations. The `AppController` creates the manager early so the registration is performed before GUI wiring that relies on queued signals.
- Testing and future work:
  - Add unit tests for `AppController` by injecting a fake/subclassed `TrackingManager` or `TrackingDataStorage` to validate orchestration, filtering of "only missing" items, and signal forwarding.
  - Consider making `TrackingProgressDialog` modeless and letting `AppController` close it when tracking completes (better UX for long runs).
  - Extract `MiniLoaderCoordinator` to encapsulate mini-loader polling and visible-worm aggregation logic.
  - Audit ownership and parent-child relationships for all core objects to ensure deterministic lifetime management.
- Version information is managed through CMake template (`version.h.in`)
- Current project version: 0.9.0

## How to use the AppController API

The `AppController` centralizes non-UI orchestration (storage, models, TrackingManager, and optionally the tracking dialog). Below are short, copy-pasteable C++ usage examples showing common patterns: obtaining models, showing the controller-owned dialog, starting tracking directly, and wiring signals.

1) Create the controller and bind models to views (typical MainWindow constructor)
```cpp
// In MainWindow::MainWindow(...)
m_appController = new AppController(this);

// Bind models to views
ui->wormTableView->setModel(m_appController->blobTableModel());
ui->annoTableView->setModel(m_appController->annotationTableModel());

// Optionally keep a local pointer for legacy code paths
m_blobTableModel = m_appController->blobTableModel();
m_trackingDataStorage = m_appController->trackingDataStorage();
```

2) Show the controller-owned tracking dialog (controller will orchestrate start when user clicks Begin)
```cpp
// Prepare parameters from VideoLoader
QString videoPath = ui->videoLoader->getCurrentVideoPath();
int keyFrame = ui->videoLoader->getCurrentFrameNumber();
auto settings = ui->videoLoader->getCurrentThresholdSettings();
int totalFrames = ui->videoLoader->getTotalFrames();
bool onlyTrackMissing = true; // or false, depending on user preference

// Show the dialog (controller creates, wires, and runs it parented to 'this')
m_appController->showTrackingDialog(videoPath, keyFrame, settings, onlyTrackMissing, totalFrames, this);
```

3) Start tracking directly from MainWindow (controller will build initial worm list from the blob model)
```cpp
// Start tracking using the blob model contents as initial worms
m_appController->beginTrackingFromModel(videoPath, keyFrame, settings, /*onlyTrackMissing=*/true, totalFrames);
```

4) If you already have InitialWormInfo populated, call the lower-level API:
```cpp
std::vector<Tracking::InitialWormInfo> initialWorms = /* build from model or other source */;
m_appController->requestStartTracking(videoPath, keyFrame, settings, initialWorms, totalFrames);
```

5) Signal wiring examples (connect controller signals to dialog/UI)
```cpp
// Forward progress/status to a dialog or status widget
connect(m_appController, &AppController::trackingProgress,
        m_trackingProgressDialog, &TrackingProgressDialog::updateOverallProgress);
connect(m_appController, &AppController::trackingStatusMessage,
        m_trackingProgressDialog, &TrackingProgressDialog::updateStatusMessage);

// Listen for final tracks in MainWindow
connect(m_appController, &AppController::tracksUpdated,
        this, &MainWindow::acceptTracksFromManager);

// Handle finished/failed/cancelled
connect(m_appController, &AppController::trackingFinished, this, [](){ /* UI update */ });
connect(m_appController, &AppController::trackingFailed, this, [](const QString& reason){ /* UI error */ });
connect(m_appController, &AppController::trackingCancelled, this, [](){ /* Cancellation UI */ });
```

Notes and best practices
- Ownership: `AppController` parents storage/models/manager to itself by default so lifetime is tied to the controller. Create the controller with `new AppController(this)` from a long-lived UI owner (e.g., MainWindow).
- Dialog modality: `showTrackingDialog(...)` currently creates and exec()s a modal dialog. Consider requesting a modeless dialog by updating the controller if you want non-blocking background tracking.
- Meta-types and threading: `TrackingManager` registers required Qt meta-types on construction. Ensure the controller creates the manager early (it does by default) before you rely on queued, cross-thread signals.
- Testing: For unit tests, inject or subclass a fake `TrackingManager` or `TrackingDataStorage` into `AppController` (alternate constructor exists) to observe and assert orchestration behavior and signal forwarding.
