# Disk access and responsiveness

VideoLoader accepts video-open requests asynchronously. Its `loadVideo()` return
value means the request was accepted; `videoLoaded` or `videoLoadFailed` reports
completion. FrameLoader owns the only playback decoder and performs opening,
seeking, sequential prefetch, and frame reads on its worker thread. Requests and
results carry a video generation so switching videos cannot display old frames.
Uncached seeks retain the current display until the requested frame arrives.
Dragging the frame slider samples the latest target every 40 ms and requests the
exact target immediately on release. Playback pauses during the drag and resumes
on release if it was previously playing. Playback ticks cannot replace a pending
seek. Explicit navigation clears obsolete queued prefetch work; an OpenCV read
already in progress must still finish. Duplicate requests for that in-flight
frame are suppressed.

The raw-frame cache retains at most 500 frames and targets a 256 MiB memory
budget (one oversized frame is retained so it remains usable). Tracking completion
does not reduce its capacity. Playback prefetches ten frames ahead. The crop
preview reuses the current display image, including its thresholding, without
requesting neighboring frames or processing a full frame again.

AnalysisSessionModel discovers directories and loads metadata on workers. Model
changes happen on the GUI thread. Superseded scans stop between directories and
their results are discarded. Refreshes retain current group assignments/checks
and reuse tracks when the run path, source size, and modification time match.
Full tracks load asynchronously only for checked runs requested by an open plot.
Plots refresh through `analysisDataReady` when those tracks arrive.

`worms.json.index.json` is an optional, small discovery sidecar containing worm
IDs and the source file's size and modification time. New exports write it;
legacy files gain it on their first successful discovery when the directory is
writable. Missing, stale, or malformed indexes fall back to the original file.
An index write failure never prevents loading or exporting a run.

Run import reads and parses its files once on a worker, then applies the prepared
snapshot after the corresponding video opens. Plugin discovery and debug image
export also run on workers. Debug jobs capture in-memory inputs on the GUI thread
and never read live tracking storage from another thread; pending refreshes are
coalesced. Debug previews use images already loaded by the worker.

Analysis-state writes are debounced, serialized, and atomic. GUI scale/calibration
saves use a serial metadata writer; metadata readers on workers wait for earlier
queued writes. Metadata writes are atomic and read/modify/write operations are
serialized to preserve other fields.

Run `python3 scripts/test_disk_access.py build` with a configured CMake Unix
Makefiles build. The regression executable checks asynchronous video opening,
latest-request seeks, failed opens, folder switching, preserved state/caches,
metadata write ordering, and index invalidation. On Unix it also blocks a file
read with a FIFO whose writer is released by a GUI timer: a synchronous scan
would prevent that timer from firing and fail the test's timeout.

Run `python3 scripts/test_disk_access.py build --playback-only` to isolate video
open/seek and playback-navigation regressions. These check control synchronization,
pending seeks during playback ticks, drag coalescing, exact release positioning,
and pause/resume during dragging.

Playback speed throughput experiment:

- `python3 scripts/test_disk_access.py build --benchmark-speed` generates
  high-detail deterministic noise MJPEG fixtures at 640x480 and 1920x1080.
- Add `--light` to use inexpensive smooth-gradient MJPEG fixtures instead.
- Each fixture has 240 frames at 25 fps. Each run opens a fresh MainWindow/cache,
  measures advancement from frame 0 to 239 with the Qt event loop, and alternates
  requested speeds in the order 10x, 20x, 20x, 10x. The runner uses Qt offscreen;
  results measure application frame advancement, not physical screen refresh.

Local results (two runs per speed, rounded):

| Fixture | Requested 10x: achieved | Requested 20x: achieved |
| --- | --- | --- |
| Smooth gradient, 640x480 | 9.98x | 19.38x |
| Smooth gradient, 1920x1080 | 9.97x | 15.54x |
| High detail, 640x480 | 9.42x | 9.34x |
| High detail, 1920x1080 | 1.49x | 1.50x |

These synthetic fixtures show that 20x can improve throughput when there is
headroom, but cannot overcome a saturated playback pipeline. They do not isolate
decoder time from GUI work or predict performance for a particular user video.
The supported speed range now includes 20x; previously the UI offered 20x while
VideoLoader clamped it to 10x. Playback continues to preserve every source frame.


Fractional-FPS investigation (2026-10-08):

`python3 scripts/test_disk_access.py build --benchmark-fps` compares equal
four-second source intervals at 3.75 and 7.5 fps with identical pixel content and
MJPEG encoding. It checks fractional FPS metadata and normal-speed duration,
and reports playback and standalone decode throughput at 1x, 10x, and 20x.

`python3 scripts/test_disk_access.py build --benchmark-file /absolute/video.avi`
benchmarks an existing video. The input is copied without transcoding into a
temporary directory, isolating metadata and leaving the original video untouched.
Add `--run /absolute/run/directory --overlays tracks,centerlines,threshold` to
import a temporary copy of tracking results and select overlays. Run benchmarks
serially to avoid competing for CPU. Real-file tests sample four wall-clock
seconds at each speed; run tests start at source time 100 seconds.

The supplied AVI files have consistent duration and fractional frame-rate metadata:
S4_N2-0_0001_cropped is H.264, 1814x1716, 3.75 fps, 2250 frames, 600 seconds;
S2_PFk-50_0002_cropped is H.264, 1886x1892, 7.5 fps, 4500 frames, 600 seconds.
OpenCV's FFmpeg backend decoded the first 150 frames at approximately 538 and
738 fps respectively. Sequential frame-position checks showed no mismatches.

| Playback configuration | Achieved at requested 10x | Achieved at requested 20x |
| --- | --- | --- |
| 3.75 fps AVI, plain | 9.86x | 20.46x |
| 7.5 fps AVI, plain | 10.23x | 19.03x |
| 3.75 fps AVI, tracking + tracks/centerlines/threshold | 9.82x | 11.34x |
| 7.5 fps AVI, tracking + tracks/centerlines/threshold | 3.58x | 3.56x |
| 7.5 fps AVI, tracking + tracks only | 3.44x | 3.44x |
| 7.5 fps AVI, tracking + threshold only (saved blur enabled) | 10.23x | 19.03x |

Tracking comparisons used PROC_2026-10-08-123422 for S4 and
PROC_2026-10-08-124030 for S2. The full track-path overlay reproduces the large
slowdown; removing it restores throughput with the same data and threshold
settings. VideoLoader::paintEvent rebuilds each visible worm's entire
QPainterPath from all recorded track points and strokes it on every repaint.
The higher input FPS also requires twice as many presented frames per source
second, amplifying a per-frame rendering bottleneck. Blur and raw decode are
not the limiting work in these measured configurations.

Normal-speed frame cadence remained approximately 267 ms for S4 and 133 ms for
S2, including with results loaded. The reported plain-video/1x slowdown was not
reproduced. These tests use Qt offscreen rendering with fresh MainWindows;
physical display, existing session state, and other UI modes can differ.

Track-overlay caching (2026-10-11):

VideoLoader now caches each worm's complete QPainterPath in source-video
coordinates. Replacing/clearing tracks, changing the centroid/midpoint display
mode, or refreshing blob geometry invalidates those paths. Midpoint lookup is
lazy and uses individual stored blobs rather than copying each frame's blob map.

A second cache stores the rasterized track overlay as a transparent, view-sized
image at the widget's device pixel ratio. It is reused across frame presentation
and seeking. Track styles/visibility, geometry, view bounds (zoom/pan), widget
size, or display pixel ratio refresh the image. View changes reuse the source
paths, with a constant two-logical-pixel stroke. Dynamic markers and current-frame
centerlines remain separate. Active panning/following still rasterizes a new view;
there is no frame-rate cap or frame skipping in this change.

The playback regression now compares complete rendered images against direct
path drawing, allowing three channel levels of edge rounding for transparent
composition. It covers frame changes, transparent colors, item and selected-track
visibility, centroid/midpoint switching, blob refresh, zoom, pan, resize,
replacement and clearing of tracks. The suite passes at default and 2x scaling
(`QT_SCALE_FACTOR=2 python3 scripts/test_disk_access.py build --playback-only`).

Repeating the real-file benchmark with the same saved runs, 100-second starting
position, and tracks/centerlines/threshold enabled produced:

| Video | Previous 10x request | Cached 10x request | Previous 20x request | Cached 20x request |
| --- | --- | --- | --- | --- |
| S4, 3.75 fps | 9.82x | 9.86x | 11.34x | 20.25x |
| S2, 7.5 fps | 3.58x | 10.14x | 3.56x | 19.03x |

These are achieved source-time speeds in four-second Qt offscreen measurements,
compared with the earlier recorded measurements; they are not physical display
refresh rates. Cache misses after view/data changes still include one full
rasterization, while steady playback only composites the cached image.

Mini preview investigation (2026-10-11):

Add `--benchmark-mini` to `--benchmark-file ... --run ... --overlays
tracks,centerlines,threshold` to select the first worm and compare an active crop
preview. This runs six four-second trials at requested 20x, alternating preview
off, image only, and image with blob/centerline overlays, then reversing that
order. Main-video overlays, worm selection and widget layout stay the same.
The off condition disconnects crop refresh signals and disables preview painting;
simply hiding the widget would leave crop updates running. Inputs and tracking
runs remain isolated temporary copies. Earlier real-file benchmarks did not
select a worm, so they did not measure an active mini crop.

| Video / scaling | Preview off | Image only | Preview with overlays |
| --- | --- | --- | --- |
| S4, 3.75 fps / 1x | 20.45–20.46x | 20.46x | 20.46x |
| S2, 7.5 fps / 1x | 19.03x | 19.03x | 18.86–19.03x |
| S2, 7.5 fps / 2x | 18.82–19.04x | 19.02x | 19.02–19.04x |

At the 100-second starting frame, 300 repeated isolated crop updates averaged
0.002 ms. Repeated preview rendering, including overlays, averaged 0.12 ms for
S4 and 0.14 ms for S2 at 1x scaling; S2 took 0.22 ms at 2x scaling. These use
57x57 and 63x63 source crops and approximately 198x102 logical-pixel widgets
(189x102 at 2x). Blob geometry was confirmed visible in the overlay paint probe.
Paint probes reuse that starting frame; the throughput trials advance normally.

MiniLoader does not decode or threshold independently. MainWindow crops the
already prepared display image; MiniLoader scales it and draws current-frame
blob/centerline geometry on the GUI thread. Its contour/hole clipping is repeated
on every overlay repaint, but it does not redraw entire track histories. The
older multi-frame crop helper has no current callers. These offscreen tests show
no meaningful throughput penalty from the active preview in these configurations;
they do not measure physical display refresh or every crop/contour complexity.
