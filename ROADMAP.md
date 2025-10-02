# YAWT Roadmap (Local Task Tracker)

This file is the single source of truth for local planning and status. Each task has:
- An ID (T-###)
- A checkbox for status ([ ] open, [x] done)
- Owner, milestone, area, priority, optional size
- Optional target and notes

Conventions
- Milestones: 0.9.x Hardening, 1.0.0 UX + Features, Testing, Build & Packaging, Documentation & Process, Backlog/Ideas
- Areas: core, gui, data, utils, build, ci, docs
- Priorities: P0 (urgent), P1, P2
- Sizes: S, M, L
- Owners: use GitHub handle or @you

Legend
- [ ] open
- [x] done
- [-] blocked

---

## Milestone: 0.9.x Hardening

- [ ] T-001 Threads: deterministic shutdown of worker threads
  - owner: @you
  - milestone: 0.9.x Hardening
  - area: core
  - priority: P0
  - size: M
  - target: TBD
  - notes:
    - Join/quit video processor threads, tracker threads, and video saver thread on cancel and app quit.
    - Ensure no dangling QThreads; use QPointer guards and robust cleanup.

- [ ] T-002 Wire app shutdown to cleanup
  - owner: @you
  - milestone: 0.9.x Hardening
  - area: core
  - priority: P0
  - size: S
  - target: TBD
  - notes:
    - Connect QCoreApplication::aboutToQuit to AppController::cancelTracking() and TrackingManager::cleanupThreadsAndObjects().

- [ ] T-003 Memory: streaming/chunked save of processed frames
  - owner: @you
  - milestone: 0.9.x Hardening
  - area: core
  - priority: P0
  - size: M
  - target: TBD
  - notes:
    - Avoid retaining all processed frames in memory.
    - Clear processed frame vectors promptly on save/cancel/fail.

- [ ] T-004 Error normalization and cleanup consistency
  - owner: @you
  - milestone: 0.9.x Hardening
  - area: core
  - priority: P0
  - size: S
  - target: TBD
  - notes:
    - Ensure all failure paths emit trackingFailed(QString) with actionable messages and run cleanup.

- [ ] T-005 Status messaging granularity
  - owner: @you
  - milestone: 0.9.x Hardening
  - area: core
  - priority: P1
  - size: S
  - target: TBD
  - notes:
    - Emit informative trackingStatusUpdate: “Processing chunk x/y”, “Saving video n%”.

- [ ] T-006 Meta-type registration audit
  - owner: @you
  - milestone: 0.9.x Hardening
  - area: core
  - priority: P1
  - size: S
  - target: TBD
  - notes:
    - Verify Q_DECLARE_METATYPE and qRegisterMetaType for all types used across threads.

- [ ] T-007 Remove hard-coded OpenCV path; make discovery configurable
  - owner: @you
  - milestone: 0.9.x Hardening
  - area: build
  - priority: P1
  - size: S
  - target: TBD
  - notes:
    - Prefer find_package(OpenCV REQUIRED) with optional -DOpenCV_DIR override.
    - Document in CLAUDE.md.

- [ ] T-008 Fix or remove CMake “deploy” target
  - owner: @you
  - milestone: 0.9.x Hardening
  - area: build
  - priority: P1
  - size: S
  - target: TBD
  - notes:
    - Either add deploy_mac.sh (macdeployqt + OpenCV bundling) or remove the target to avoid confusion.

- [ ] T-009 Deprecate legacy qmake script
  - owner: @you
  - milestone: 0.9.x Hardening
  - area: build
  - priority: P2
  - size: S
  - target: TBD
  - notes:
    - scripts/qtbuildtool.sh references a .pro which does not exist; remove or move to archive.

- [ ] T-010 Sample data for smoke testing
  - owner: @you
  - milestone: 0.9.x Hardening
  - area: ci
  - priority: P2
  - size: M
  - target: TBD
  - notes:
    - Include tiny synthetic video (or generator) to exercise a short run and verify stability locally.

---

## Milestone: 1.0.0 UX + Features

- [ ] T-020 Modeless TrackingProgressDialog
  - owner: @you
  - milestone: 1.0.0 UX + Features
  - area: gui
  - priority: P1
  - size: M
  - target: TBD
  - notes:
    - Make dialog modeless; AppController owns lifecycle; auto-close on finish/cancel/fail.

- [ ] T-021 Extract MiniLoaderCoordinator
  - owner: @you
  - milestone: 1.0.0 UX + Features
  - area: gui
  - priority: P2
  - size: M
  - target: TBD
  - notes:
    - Encapsulate mini-loader polling and visible-worm aggregation; reduce MainWindow coupling.

- [ ] T-022 ROI/annotation UX improvements
  - owner: @you
  - milestone: 1.0.0 UX + Features
  - area: gui
  - priority: P2
  - size: M
  - target: TBD
  - notes:
    - Expose setRoiSizeMultiplier in UI; better auto-centering; clearer selection/status messages.

- [ ] T-023 Export pipeline: JSON/HDF5 and predictable paths
  - owner: @you
  - milestone: 1.0.0 UX + Features
  - area: data
  - priority: P2
  - size: M
  - target: TBD
  - notes:
    - JSON export for threshold settings, initial ROIs, final tracks.
    - Optional HDF5 for large runs; per-video directories with QStandardPaths.

- [ ] T-024 Advanced config for merge/split heuristics
  - owner: @you
  - milestone: 1.0.0 UX + Features
  - area: core
  - priority: P2
  - size: S
  - target: TBD
  - notes:
    - Expose tuning (e.g., IoU threshold) behind a safe, power-user setting.

---

## Milestone: Testing

- [ ] T-030 Unit tests: src/utils/thresholdingutils.*
  - owner: @you
  - milestone: Testing
  - area: utils
  - priority: P1
  - size: S
  - target: TBD
  - notes:
    - Parameter boundary tests; deterministic outputs where applicable.

- [ ] T-031 Unit tests: src/data/trackingdatastorage.*
  - owner: @you
  - milestone: Testing
  - area: data
  - priority: P1
  - size: M
  - target: TBD
  - notes:
    - Add/get semantics; merge history persistence; basic invariants.

- [ ] T-032 AppController orchestration tests
  - owner: @you
  - milestone: Testing
  - area: core
  - priority: P1
  - size: M
  - target: TBD
  - notes:
    - Inject fake TrackingManager/TrackingDataStorage; verify “only track missing” filtering and signal forwarding.

- [ ] T-033 Integration-lite synthetic run
  - owner: @you
  - milestone: Testing
  - area: core
  - priority: P2
  - size: M
  - target: TBD
  - notes:
    - Short synthetic/tiny video to exercise progress, cancellation, and error paths headlessly.

- [ ] T-034 Regression tests for merge/split decisions
  - owner: @you
  - milestone: Testing
  - area: core
  - priority: P2
  - size: M
  - target: TBD
  - notes:
    - Canned contours/rects; verify selection logic and state transitions.

---

## Milestone: Build & Packaging

- [ ] T-040 CMake options for OpenCV/Qt discovery
  - owner: @you
  - milestone: Build & Packaging
  - area: build
  - priority: P1
  - size: S
  - target: TBD
  - notes:
    - Add USE_SYSTEM_OPENCV and hints for Homebrew paths; document overrides in CLAUDE.md.

- [ ] T-041 deploy_mac.sh (macdeployqt + OpenCV)
  - owner: @you
  - milestone: Build & Packaging
  - area: build
  - priority: P2
  - size: M
  - target: TBD
  - notes:
    - Bundle Qt and OpenCV libs; produce a distributable .app.

---

## Milestone: Documentation & Process

- [x] T-050 Update CLAUDE.md for agents (CMake-only, env, behavior)
  - owner: @you
  - milestone: Documentation & Process
  - area: docs
  - priority: P0
  - size: S
  - target: N/A
  - notes:
    - Corrected build info; removed qmake; added agent guidelines.

- [x] T-051 Add local roadmap with IDs (this file)
  - owner: @you
  - milestone: Documentation & Process
  - area: docs
  - priority: P0
  - size: S
  - target: N/A
  - notes:
    - Converted ROADMAP.md to local tracking format with IDs and metadata.

- [x] T-052 Add PR/Issue templates
  - owner: @you
  - milestone: Documentation & Process
  - area: docs
  - priority: P1
  - size: S
  - target: N/A
  - notes:
    - .github/ISSUE_TEMPLATE/task.md and .github/PULL_REQUEST_TEMPLATE.md added.

- [ ] T-053 Expand README quick start and troubleshooting
  - owner: @you
  - milestone: Documentation & Process
  - area: docs
  - priority: P2
  - size: S
  - target: TBD
  - notes:
    - Build/run steps; how to set keyframe, add ROIs, start tracking; common issues.

- [ ] T-054 CONTRIBUTING.md (branching, labels, PR checks)
  - owner: @you
  - milestone: Documentation & Process
  - area: docs
  - priority: P2
  - size: S
  - target: TBD
  - notes:
    - Define label scheme, milestones, and PR requirements; reference CLAUDE.md.

---

## Backlog / Ideas

- [ ] T-060 Cross-platform packaging (Windows/Linux)
  - owner: @you
  - milestone: Backlog/Ideas
  - area: build
  - priority: P3
  - size: L
  - target: TBD
  - notes:
    - Windows: windeployqt; Linux: AppImage/flatpak; CI matrix later.

- [ ] T-061 Enhanced logging and diagnostics
  - owner: @you
  - milestone: Backlog/Ideas
  - area: utils
  - priority: P3
  - size: M
  - target: TBD
  - notes:
    - Log levels and optional file logging under QStandardPaths::AppDataLocation.

- [ ] T-062 Import/replay of saved runs
  - owner: @you
  - milestone: Backlog/Ideas
  - area: data
  - priority: P3
  - size: M
  - target: TBD
  - notes:
    - Load settings, ROIs, and tracks into UI for retracking/inspection.

- [ ] T-063 Performance review: chunk sizing and thread pools
  - owner: @you
  - milestone: Backlog/Ideas
  - area: core
  - priority: P3
  - size: M
  - target: TBD
  - notes:
    - Review QThread vs QThreadPool usage; minimize cv::Mat copies; preallocation.

---