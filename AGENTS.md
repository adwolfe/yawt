# AGENTS.md

This file is the general agent guide for the YAWT repository.

## Project Summary
YAWT (Yet Another Worm Tracker) is a C++17 Qt6 desktop application for tracking *C. elegans* worms in chemotaxis videos. The workflow starts from a keyframe where all worms are visible, then tracks forward and backward through the timeline.

## Tech Stack
- Language: C++17
- GUI: Qt 6 (Core/Gui/Widgets/Svg)
- Vision: OpenCV
- Build: CMake (only)

## Repository Layout
- `src/` — application code
  - `src/gui/` — Qt UI and widgets (MainWindow, dialogs, delegates)
  - `src/core/` — controller and tracking orchestration
  - `src/core/processing/` — video processing workers
  - `src/data/` — tracking data structures and storage
  - `src/models/` — Qt models for tables/views
  - `src/utils/` — logging, debug helpers, thresholding utilities
- `resources/` — icons and SVG assets
- `scripts/` — build/run/deploy helpers
- `CMakeLists.txt` — build definition (current version: 0.9.5)
- `version.h.in` — template for generated version header

## Architecture (MVC)
- **View:** `MainWindow` and UI widgets in `src/gui` handle user interaction and display.
- **Controller:** `AppController` (`src/core`) owns storage, models, and `TrackingManager`, and exposes high-level APIs to the UI.
- **Model:** `TrackingDataStorage` (`src/data`) plus Qt models in `src/models` that adapt data to views.

## Data Flow (High-Level)
1. UI selects a video and keyframe; ROIs/initial worms are added via the blob model.
2. UI calls `AppController` to start tracking (from model or explicit initial worms).
3. `TrackingManager` spawns background workers for video processing and per-worm tracking (QThreads).
4. Progress and status are emitted via Qt signals/slots and forwarded to the UI.
5. When done, tracks are persisted to storage and emitted via `tracksUpdated`.
6. Final outputs include CSV track export and frame-atomic JSON state.

## Threading Notes
- `TrackingManager` coordinates cross-thread work and progress aggregation.
- Custom types used across threads must be registered (handled in `TrackingManager::registerMetaTypes`).
- Use queued signals/slots for cross-thread communication.

## Build & Run
### Requirements
- CMake >= 3.16
- Qt 6 (Core, Gui, Widgets, Svg)
- OpenCV

### Typical macOS setup (Homebrew)
```
brew install cmake qt opencv
```

### Debug build
```
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=$(qtpaths --install-prefix)
make -j"$(sysctl -n hw.logicalcpu)"
open -n yawt.app
```

### Release build
```
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$(qtpaths --install-prefix)
make -j"$(sysctl -n hw.logicalcpu)"
```

Notes:
- If your system exposes `qtpaths6` instead of `qtpaths`, substitute it.
- OpenCV is discovered via `find_package(OpenCV REQUIRED)`; use `-DOpenCV_DIR=<path>` if needed.
- The macOS deploy target runs `scripts/deploy_simple_working.sh`.

## Testing
- No test suite is configured.
- Safe additions: unit tests for pure logic in `src/data` or `src/utils`.

## Agent Guidelines
Do:
- Use CMake only; do not add qmake or `.pro` files unless explicitly requested.
- Keep GUI headers out of non-UI headers.
- Maintain QObject parent/child ownership for deterministic lifetimes.
- Keep changes small and localized unless asked for refactors.

Don’t:
- Change tracking algorithm constants (e.g., thresholds/merge rules) without explicit approval.
- Add network services or external dependencies without approval.
- Introduce long-running background processes in scripts.

## Fragile Areas
- Cross-thread signaling and thread cleanup in `TrackingManager` and `WormTracker`.
- Processed-frame memory usage and cleanup paths.
- Merge/split handling logic and frame-atomic state persistence.

## Safe Enhancements
- Logging and error messages (see `src/utils/debugutils.*`).
- UI polishing and small UX improvements.
- Documentation improvements.
