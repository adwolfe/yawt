# Reference annotations

Start YAWT with `--debug`, open a video, and select **Processing → Reference**.
The tab uses the existing player and shows the raw video with human annotations.
Tracking, threshold, and computed-centerline overlays are hidden while the tab is active.
Reference IDs and geometry are independent of the tracking results and worm-selection table.

## Annotating a frame

1. Select a persistent **Worm ID**. Keep that ID for the same animal throughout the clip.
2. Choose **Position**, **End A**, **End B**, or **Centerline**.
3. Click the video to place a point. Click and drag to adjust it.
   For a centerline, click successive vertices from the A side toward the B side.
   Drag an existing vertex to move it; right-click a vertex to remove it.
4. Set the head assignment only when known. Otherwise keep **Head unknown**:
   A/B still allow tests of physical-end continuity without assuming anatomy.
5. Set visibility and an event label when useful. Notes belong to the current worm and frame.
6. Choose **Verified** after checking the annotation, or **Uncertain** when it cannot be trusted.
   Geometry and label edits reset the review state to **Unreviewed**.

An approximate position click is not a measured blob centroid or an arc-length body midpoint.
Do not use it for precise comparisons with either measurement. Endpoints and the traced body line
are separate fields; drawing a line does not invent endpoint observations.
Unknown endpoints are omitted rather than stored as `(0, 0)` or predicted through an overlap.
A one-vertex line is a draft; a centerline comparison needs at least two vertices.

**Clear tool** removes the selected kind of geometry. **Delete** removes the current
worm/frame record. **Swap A/B** exchanges endpoint labels, reverses the centerline, and
preserves the anatomical head assignment. Undo and redo also update the saved file.

**Step** controls the previous/next frame buttons. The **Mark** buttons jump to
annotated frames for the selected worm. Clicking a table row selects its worm and frame.
Navigation respects the player's selected start/stop frame bounds.
There is no interpolation: unannotated frames remain unannotated.

The mouse wheel zooms as usual. Choose **Pan / zoom** to pan the video. Keyboard shortcuts
are enabled while Reference is active, except while editing text, spinboxes, or combo boxes:

| Key | Action |
| --- | --- |
| P / A / B / C | Position / End A / End B / Centerline |
| Esc | Pan / zoom |
| Comma / period | Previous / next frame by the selected step |
| Shift + comma / period | Previous / next marked frame |
| Platform Undo / Redo | Undo / redo a reference edit |

## Files and source mapping

Completed edits autosave atomically to `<video filename>.reference.json`, beside the video.
The full save path is available in the save-status tooltip. Reopening the video loads this
file automatically. **Save as** writes another JSON file and makes it the autosave destination
for the current session. **Load** opens an existing reference JSON for the loaded video.
For automatic loading in a later session, use the default sidecar filename.

Unreadable or incompatible sidecars are preserved. The status shows the error; use **Load**
to open a valid file or **Save as** to save elsewhere. Save failures remain visible, and leaving
the video or closing YAWT requires saving or explicitly discarding any unsaved edits.

Coordinates use pixels in the loaded video, with zero-based frame numbers. For a crop, choose
**Mapping** and record the original video's path, crop X/Y, and first original frame:

```
original_x = reference_x + crop_x
original_y = reference_y + crop_y
original_frame = reference_frame + first_frame
```

This mapping assumes no resizing or frame-rate conversion. A tracking time crop does not
renumber frames or change the pixel origin.

The plain JSON format is `yawt_reference_annotations`, schema version 1. It contains:

- `video`: path, width, height, frame count, FPS, and file byte size.
- `source_mapping`: original path, crop origin, and starting frame.
- `annotations`: sparse records keyed by `frame` and `worm_id`, with `provenance: manual`,
  optional `position`, `end_a`, and `end_b`, and an A-to-B `centerline` array.
- Each record's `orientation`, `visibility`, `event`, `review`, and `notes`.

The loader checks the video's filename, byte size, dimensions, frame count, and FPS, and rejects
duplicate records, invalid labels, and coordinates outside the video. These metadata checks
are not a content hash. Moving a video together with its sidecar is supported.

Accuracy tests should use only explicitly verified records and only the observations present
in those records. Uncertain passages can still provide event or visibility references without
requiring a hidden endpoint or body route to be guessed. Unreviewed and uncertain geometry
must not be treated as confirmed ground truth.

## Regression checks

Run `python3 scripts/test_reference_annotations.py build` with a configured Unix Makefiles
build. The runner exercises actual widget gestures and persistence, including zoomed video
coordinates, centerline edits, undo/redo, review invalidation, independent worm/frame records,
reopening, debug gating, and invalid-file preservation. It runs with Qt's offscreen platform
and does not open a visible desktop window.
