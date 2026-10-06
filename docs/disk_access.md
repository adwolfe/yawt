# Disk access and responsiveness

VideoLoader accepts video-open requests asynchronously. Its `loadVideo()` return
value means the request was accepted; `videoLoaded` or `videoLoadFailed` reports
completion. FrameLoader owns the only playback decoder and performs opening,
seeking, sequential prefetch, and frame reads on its worker thread. Requests and
results carry a video generation so switching videos cannot display old frames.
Uncached seeks retain the current display until the requested frame arrives.
The crop preview requests neighbors asynchronously and refreshes as they arrive.

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
