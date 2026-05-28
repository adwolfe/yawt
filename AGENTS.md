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
    - `appcontroller.{h,cpp}` — high-level controller, owns storage/manager/models
    - `trackingmanager.{h,cpp}` — multi-worm tracking orchestration and worker threads
    - `wormtracker.{h,cpp}` — per-worm forward/backward tracker
    - `centerlineworker.{h,cpp}` — post-tracking background centerline computation
  - `src/core/processing/` — video processing workers
  - `src/data/` — tracking data structures and storage
    - `trackingcommon.{h,cpp}` — shared types (`DetectedBlob`, `WormTrackPoint`, `AllWormTracks`) plus blob detection, skeletonization, and centerline helpers
  - `src/models/` — Qt models for tables/views
  - `src/utils/` — logging, debug helpers, thresholding utilities
- `resources/` — icons and SVG assets
- `scripts/` — build/run/deploy helpers
- `CMakeLists.txt` — build definition (current version: 0.9.6)
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
5. When all per-worm trackers finish, `TrackingManager` emits `trackingFinishedSuccessfully` and launches `CenterlineWorker` on a background thread for the post-tracking centerline phase.  The progress dialog shows two phases (tracking, then centerlines); the Close button is gated on `centerlineFinished`.
6. When centerlines are done, tracks are persisted to storage and emitted via `tracksUpdated`.
7. Final outputs include CSV track export, per-worm centerline export, and frame-atomic JSON state.

## Coiled-Worm (Ring) Topology
When a worm coils tightly enough to touch itself, the thresholded blob becomes a donut/ring shape rather than a filled disk.  YAWT detects and propagates ring topology end-to-end so downstream code can reason about the actual body shape:

- **Detection** (`Tracking::findAllPlausibleBlobsInRoi` in `trackingcommon.cpp`): uses `cv::RETR_CCOMP` with a 2-level hierarchy (outer + holes).  For each outer contour, hole areas are subtracted from the outer area to give the true body area; hole contours are stored on `DetectedBlob::holeContourPoints` (a `std::vector<std::vector<cv::Point>>`) alongside the outer `contourPoints`.
- **Storage / serialization**: `FrameSpecificPhysicalBlob` and the per-frame JSON include `holeContourPoints`; storing/reading a blob via `TrackingDataStorage::setDetectedBlobForFrame` preserves both.
- **Rendering**: `miniloader.cpp` and the videoloader skeleton overlay draw blobs as `QPainterPath::subtracted(holePath)` so coiled worms render as rings rather than filled disks.
- **Real-time ROI placement** (`WormTracker::updateTrackingState`): for ring blobs the tracker would otherwise anchor on the geometric centroid, which falls inside the hole.  Instead, when `holeContourPoints` is non-empty, `m_lastKnownPosition` is snapped to the nearest outer-contour point.  The stored `WormTrackPoint.position` is unchanged — it remains the true blob centroid for export consistency.
- **Mask construction for skeletonization** (`Tracking::populateCenterlineFromContour`): the mask is filled from the outer contour, then hole contours are erased so the skeleton runs on the actual ring shape (not a filled disk).  A 3×3 morphological close is applied when holes are present to seal pixel-thin self-touch gaps that fluctuate frame-to-frame.

## Centerline Pipeline
Centerlines (a.k.a. skeleton, "CL") are an ordered polyline running from one body end to the other.  They are computed via `Tracking::populateCenterlineFromContour`: Zhang–Suen thinning produces a 1-pixel skeleton, the skeleton's pixel-graph is built, and Dijkstra finds the longest simple path between two degree-1 endpoints.

Centerline data lives on `DetectedBlob::centerlinePoints` (`std::vector<cv::Point2f>`) and is persisted in storage; it is not duplicated on `WormTrackPoint`.  Convenience accessors:
- nose (head end): `centerlinePoints.front()`
- tail (other end): `centerlinePoints.back()`
- centerline-centroid: `centerlinePoints[size/2]`

`Tracking::extractResampledCenterlinePoints(blob, N)` returns `N` evenly-spaced points along the centerline as a `QList<QPointF>`.

### Post-tracking correction (`CenterlineWorker`)
After all per-worm trackers finish, `CenterlineWorker` runs on a background QThread to refine centerlines.  It does **not** modify `WormTrackPoint.position`; only `DetectedBlob::centerlinePoints` is updated.  The algorithm is per-worm and two-pass:

**Pass 1 — first-pass centerlines and reference body length.**  Compute a centerline for every non-Merged/Lost frame.  Collect arc lengths from non-ring frames only (where the skeleton is most reliable) and take the median as the per-worm `refLength`.

**Pass 2 — keyframe-outward orientation and repair.**  Process the worm's keyframe (`ClickedItem::frameOfSelection`) first — guaranteed to contain a clean separated blob — to seed orientation.  Then propagate forward (keyframe → end) and backward (keyframe → start) carrying a `CenterlineState` ({ resampled 20-point polyline, valid flag }) into each next frame.  Per frame:

1. Resample the centerline to 20 points (~5% body-length spacing — fine enough for shape correspondence to discriminate flips during tight coils).
2. **Orient by shape correspondence**: compare the sum of pointwise distances for forward vs reversed alignment with the previous frame's points; reverse if the reversed sum is smaller.  This is more robust than endpoint-distance matching because nose and tail can collapse spatially during coiling, but the order along the polyline still distinguishes them.
3. If the path is too short (`arcLen < 0.5 × refLength`), apply a fallback:
   - **Fallback A — ring blobs**: walk the outer contour from the nearest point to `prev.nose` for `refLength` pixels, in both directions; pick the direction whose endpoint is closer to `prev.tail`.  (For a ring, the outer contour traces the body itself.)
   - **Fallback B — non-ring blobs**: translate the previous frame's centerline by `(currentBlobCentroid − prevCentroid)` and snap any point that falls outside the current blob to the nearest outer contour point.  (For an uncoiled worm the outer contour wraps ~2× the body length, so walking it would yield an edge-trace; the previous shape is a better template.)
4. Adopt the fallback only if it covers more arc length than the original skeleton path.
5. Merged/Lost frames invalidate the propagated state; orientation resumes when the next clean frame is processed.

### Centerline rendering
`videoloader.cpp` renders the centerline polyline (toggle: skeleton/CL button) and three coloured dots:
- cyan = nose (`centerlinePoints.front()`)
- red = tail (`centerlinePoints.back()`)
- yellow = centerline-centroid (`centerlinePoints[size/2]`)

The blob centroid is rendered separately and is **never** moved onto the centerline.  The yellow centerline-centroid is the visual indicator for the body's mid-point in cases where the geometric centroid falls in a hole.

## Threading Notes
- `TrackingManager` coordinates cross-thread work and progress aggregation.
- Background worker threads owned/managed by `TrackingManager`:
  - One QThread per chunked `VideoProcessor` worker.
  - One QThread per `WormTracker` (forward/backward pair per conceptual worm).
  - Optional `VideoSaverWorker` thread for thresholded-video output.
  - `CenterlineWorker` thread launched after `trackingFinishedSuccessfully`; `cleanupThreadsAndObjects` is deferred via queued connection until `centerlineFinished` arrives.
- Custom types used across threads must be registered (handled in `TrackingManager::registerMetaTypes`).
- Use queued signals/slots for cross-thread communication.
- `AppController` relays per-phase signals (`overallTrackingProgress`, `trackingFinished*`, `centerlineProgress`, `centerlineFinished`) from `TrackingManager` to `TrackingProgressDialog`, which renders a two-phase progress UI (tracking → centerlines → enable Close).

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
- Change tracking algorithm constants (e.g., thresholds/merge rules, the centerline `kMinArcLengthFraction` or `kCenterlinePoints`) without explicit approval.
- Modify `WormTrackPoint.position` from the centerline pipeline.  The blob centroid must remain the authoritative position, even when it falls inside a coiled-worm hole; the centerline-centroid is exposed via the rendered overlay, not by overwriting `position`.
- Apply the contour-walk fallback to non-ring blobs.  The outer contour of an uncoiled worm wraps ~2× body length; walking it produces an edge-trace, not a centerline.  Use the previous-centerline template fallback (Fallback B) instead.
- Add network services or external dependencies without approval.
- Introduce long-running background processes in scripts.

## Fragile Areas
- Cross-thread signaling and thread cleanup in `TrackingManager`, `WormTracker`, and `CenterlineWorker`.  The post-tracking handoff (defer cleanup until `centerlineFinished`) is easy to break by adding new terminal paths that don't go through the centerline phase.
- Processed-frame memory usage and cleanup paths.
- Merge/split handling logic and frame-atomic state persistence.
- Ring/donut topology propagation: `holeContourPoints` must be carried alongside `contourPoints` at every blob assignment site (worker outputs, `FrameSpecificPhysicalBlob`, JSON serialization, rendering).  Dropping it anywhere causes coiled worms to render as filled disks and breaks centerline mask construction.
- Centerline orientation continuity: shape-correspondence orientation in `CenterlineWorker` depends on having a valid `prev.points` of matching length.  A reset (Merged/Lost gap, or first frame) restarts orientation freely; downstream frames inherit whichever orientation the next clean frame decides.

## Safe Enhancements
- Logging and error messages (see `src/utils/debugutils.*`).
- UI polishing and small UX improvements.
- Documentation improvements.
