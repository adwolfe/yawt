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
