# CLAUDE.md

This file provides guidance to code agents when working with this repository.

## Project Overview

YAWT (Yet Another Worm Tracker) is a Qt6-based desktop application for tracking C. elegans worm behavior in video recordings. The application processes chemotaxis experiments by starting from a keyframe where all worms are visible and tracking them both forward and backward through the video timeline.

- Repository type: single app (not a monorepo)
- Language/stack: C++17, Qt6 (Core/Gui/Widgets/Svg), OpenCV
- Current version: 0.9.1 (set in CMakeLists.txt; version header generated from version.h.in)

Top-level layout:
- src/ — Application code (core, data, gui, models, utils)
- scripts/ — Build/run/deploy helper scripts
- resources/ — Icons and SVG assets (referenced by resources.qrc)
- CMakeLists.txt — Primary build configuration
- main.cpp, info.plist, resources.qrc, version.h.in

## Environment & Setup

Required tools and libraries:
- C++17-capable compiler (Clang/AppleClang on macOS)
- CMake ≥ 3.16
- Qt 6 modules: Core, Gui, Widgets, Svg
- OpenCV (Homebrew default path /opt/homebrew on Apple Silicon)
- macOS app bundle tooling (for local runs on macOS)

Recommended macOS setup using Homebrew:
    brew install cmake qt opencv

Notes:
- The build uses: -DCMAKE_PREFIX_PATH=$(qtpaths --install-prefix). If your system exposes qtpaths6 instead, substitute qtpaths6.
- OpenCV is discovered via find_package(OpenCV REQUIRED PATHS /opt/homebrew). If OpenCV is elsewhere, pass -DOpenCV_DIR=<path> to cmake.
- No databases or external network services are used.

## Build & Run (CMake only)

Debug build from repository root:
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=$(qtpaths --install-prefix)
    make -j"$(sysctl -n hw.logicalcpu)"
    open -n yawt.app --args --debug-mode

Release build:
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$(qtpaths --install-prefix)
    make -j"$(sysctl -n hw.logicalcpu)"

Helper scripts:
- scripts/zed_build_debug.sh — Builds Debug and launches the app bundle
- scripts/zed_launch.sh — Launches an already-built app (expects ZED_WORKTREE_ROOT; otherwise use open -n build/yawt.app)

Important:
- qmake is not supported in this repository (no .pro file present). The scripts/qtbuildtool.sh script is legacy and requires a .pro that does not exist here.
- CMake defines a macOS deploy target that references deploy_mac.sh, which is not present. Treat that target as non-functional unless added.

## Testing

- No test suite is currently configured in this repository.
- Safe agent additions: unit tests for pure logic (e.g., src/data, src/utils), or Qt-less components. If introducing a test framework (e.g., QTest or GoogleTest), keep changes minimal, self-contained, and avoid altering production code paths unless necessary.

## Coding Standards and Conventions

- Language level: C++17
- Qt patterns: signal/slot, QObject parent/child ownership for lifetime management
- Layering:
  - Non-UI core: src/core/ (processing in src/core/processing/)
  - Data structures and storage: src/data/
  - GUI: src/gui/ (widgets, dialogs, delegates, .ui)
  - Models: src/models/
  - Utilities: src/utils/
- Keep GUI-only includes out of non-UI headers (e.g., AppController headers avoid GUI includes).
- Register Qt meta-types before any queued cross-thread signal usage (TrackingManager constructor handles this).
- Linters/formatters: none configured. If adding, prefer a minimal .clang-format at repo root and scope formatting to touched lines only.

## Core Architecture

Main GUI Layer:
- MainWindow — Primary UI; owns widgets and view behavior (video display, menus, tool modes, dialog parenting). Delegates orchestration to AppController.
- mainwindow.ui — Qt Designer UI
- TrackingProgressDialog — Progress/preview dialog; can be owned and wired by AppController.

Application Controller:
- AppController (src/core) — Owns and coordinates:
  - TrackingDataStorage, TrackingManager, BlobTableModel, AnnotationTableModel
  - Exposes model getters for the GUI
  - High-level APIs: beginTrackingFromModel, requestStartTracking, showTrackingDialog, cancelTracking
  - Forwards progress/status signals and can manage the tracking dialog lifecycle

Video Processing Layer:
- VideoLoader — Video file loading, frame display, and interactions (pan, ROI, crop, edit)
- VideoProcessor — Processes video chunks for thresholding and preparation
- VideoFileTreeView — Custom tree view for video selection

Tracking Engine:
- TrackingManager — Orchestrates multi-worm tracking, multi-threaded runs, merge/split handling
- WormTracker — Per-worm tracker (forward/backward from keyframe)
- WormObject — Track representation for a worm
- TrackingDataStorage — Central source of truth for blobs, tracks, annotations

Computer Vision:
- thresholdingutils.h/.cpp — OpenCV-based thresholding algorithms
- trackingcommon.h/.cpp — Shared data structures (DetectedBlob, InitialWormInfo, TrackerState)

UI Components:
- BlobTableModel — Model for detected blobs (created by controller)
- AnnotationTableModel — Annotations model (created by controller)
- ColorDelegate, ItemTypeDelegate — Custom cell renderers
- FolderFirstSortProxyModel — File system sorting utilities

## Key Data Flow

0. The UI obtains models and storage from AppController.
1. User selects a video and sets the keyframe.
2. User marks initial worm positions (ROIs) on the keyframe; stored in TrackingDataStorage via BlobTableModel.
3. UI requests tracking via AppController (beginTrackingFromModel, requestStartTracking, or showTrackingDialog).
4. TrackingManager creates forward/backward WormTracker instances and starts multi-threaded tracking.
5. Trackers process frames, maintaining state and emitting progress/status/tracking events.
6. Merge/split events are handled frame-atomically; consolidated tracks are emitted.
7. AppController stores tracks in TrackingDataStorage and re-emits tracksUpdated for UI updates.
8. Final tracks can be exported to CSV or used to populate overlays and annotation tables.

## Tracking States

- TrackingNormally — Standard single-worm tracking
- TrackingAsMerged — Tracking combined blob when worms are merged
- PausedForSplit — Merged worms have split; awaiting target selection
- Lost — Tracking failed; attempting recovery

## Development Notes

- Extensive cross-thread Qt signals/slots; meta-types registered in TrackingManager on construction.
- Video processing and per-worm tracking run in threads coordinated by TrackingManager.
- OpenCV performs thresholding and blob detection.
- macOS app bundle support; cross-platform builds supported if Qt/OpenCV available.
- Merge/split handling uses unique blob IDs, IoU, and histories stored in TrackingDataStorage.
- Versioning: CMake project version is 0.9.1; version header is generated from version.h.in.

Known gaps/limitations:
- No test suite present
- The CMake deploy target references a missing deploy_mac.sh
- Legacy qmake script exists but no .pro file

## How to use the AppController API (examples)

Create the controller and bind models in MainWindow:
    // In MainWindow::MainWindow(...)
    m_appController = new AppController(this);

    // Bind models to views
    ui->wormTableView->setModel(m_appController->blobTableModel());
    ui->annoTableView->setModel(m_appController->annotationTableModel());

    // Optional local pointers for legacy paths
    m_blobTableModel = m_appController->blobTableModel();
    m_trackingDataStorage = m_appController->trackingDataStorage();

Show the controller-owned tracking dialog:
    QString videoPath = ui->videoLoader->getCurrentVideoPath();
    int keyFrame = ui->videoLoader->getCurrentFrameNumber();
    auto settings = ui->videoLoader->getCurrentThresholdSettings();
    int totalFrames = ui->videoLoader->getTotalFrames();
    bool onlyTrackMissing = true;

    m_appController->showTrackingDialog(videoPath, keyFrame, settings, onlyTrackMissing, totalFrames, this);

Start tracking directly from the blob model:
    m_appController->beginTrackingFromModel(videoPath, keyFrame, settings, /*onlyTrackMissing=*/true, totalFrames);

Start tracking with prebuilt InitialWormInfo:
    std::vector<Tracking::InitialWormInfo> initialWorms = /* ... */;
    m_appController->requestStartTracking(videoPath, keyFrame, settings, initialWorms, totalFrames);

Wire progress and completion signals:
    connect(m_appController, &AppController::trackingProgress,
            m_trackingProgressDialog, &TrackingProgressDialog::updateOverallProgress);
    connect(m_appController, &AppController::trackingStatusMessage,
            m_trackingProgressDialog, &TrackingProgressDialog::updateStatusMessage);

    connect(m_appController, &AppController::tracksUpdated,
            this, &MainWindow::acceptTracksFromManager);

    connect(m_appController, &AppController::trackingFinished, this, [](){ /* UI update */ });
    connect(m_appController, &AppController::trackingFailed, this, [](const QString& reason){ /* UI error */ });
    connect(m_appController, &AppController::trackingCancelled, this, [](){ /* Cancellation UI */ });

Best practices:
- Parent AppController to a long-lived UI owner (e.g., MainWindow) so it owns storage/models/manager.
- Construct TrackingManager early so meta-types are registered before cross-thread signals.
- Current dialog is modal; a future modeless variant could improve UX on long runs.

## Agent Behavior Guidelines

Do:
- Use CMake as the sole build system; do not introduce qmake unless explicitly requested along with a .pro.
- Keep non-UI logic in src/core and src/data; avoid GUI header leakage into non-UI headers.
- Maintain QObject parent/child ownership to prevent leaks and ensure deterministic lifetimes.
- Register any new Qt meta-types used across threads in a central constructor (e.g., TrackingManager).
- Prefer small, localized refactors with clear comments and minimal surface area.
- Use targeted searches in src/ subtrees; do not guess file paths.

Don’t:
- Modify tracking thresholds or merge/split algorithmic constants (e.g., PHYSICAL_BLOB_IOU_THRESHOLD) without human approval.
- Introduce long-running watchers/servers in scripts; builds must terminate.
- Reorganize project structure broadly or change public APIs (AppController, TrackingManager) without coordination.
- Add dependencies on external services or networked APIs.

Fragile/Risky areas:
- Threading/cross-thread signals in TrackingManager and WormTracker
- Processed frame memory usage and cleanup paths
- macOS deployment target referencing a missing script (non-blocking; avoid relying on it)

Safe operations:
- Add focused unit tests for pure logic (src/data, src/utils)
- Improve error messages/logging via src/utils/debugutils.*
- Fix trivial bugs, null checks, and race mitigations with clear comments
- Add non-invasive documentation and comments; minor refactors that preserve behavior