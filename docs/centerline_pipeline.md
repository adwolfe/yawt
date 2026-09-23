# Centerline Pipeline: Vocabulary and Structure

This page defines the code names that appear in comments and debug output for the
post-tracking centerline pass: **Phase A / B / C**, **Sweep 0 / 1**, **Step 1–5**,
**D-1 … D-4**, **S-0 / S-1**, and the retired **D-2 / D-3 / 0-tip ring cut**. The names date from the pass's incremental
development and are kept because they are embedded in log lines, the Debug tab, and
`Debug::CenterlineBranch`. This is the only place they are defined.

Code: `src/core/centerlineworker.cpp` (orchestration), `src/core/centerlineprocessor.cpp`
(per-frame work), `src/core/centerlinetypes.h` (types), `src/core/centerlinegeometry.cpp`
(geometry helpers).

## What the pass does

Tracking produces, per worm per frame, a `DetectedBlob` (contour, holes, area) and a
`TrackPoint` (centroid, search window, quality). The centerline pass runs afterwards on
a background thread (`CenterlineWorker::doWork`) and, for every non-lost frame, fills in:

- `DetectedBlob::centerline.points` — an ordered polyline from head to tail
- `DetectedBlob::centerline.tipCandidates` with `headTipIdx` / `tailTipIdx`
- `DetectedBlob::centerline.topology` — `Clean`, `SelfCrossed`, `Merged`, or `Lost`
- per-worm `TipFeatureBaseline` statistics in `TrackingDataStorage`

When it finishes, `TrackingManager::handleCenterlineFinished` calls
`TrackingDataStorage::refreshDerivedTrackData()`, which copies body length and head/tail
tip positions onto the track points, and writes `<basename>_headtail_swaps.xlsx`.

## Phases (what kind of information)

The "phases" name three layers of per-worm knowledge that were added in that order.
They are not execution stages; all three are computed inside Step 1 of every frame.

| Name | Meaning | Type | Where computed |
|---|---|---|---|
| **Phase A** | Per-worm **tip-feature baseline**: running mean and variance of tip curvature magnitude, tip width, and body length, sampled only on `Clean` frames. Used as a length prior and as a curvature threshold. | `Centerline::TipFeatureBaseline` (Welford online stats) | `TrackingDataStorage::recordTipFeatureSample` / `recordBodyLengthSample`, fed from Step 1 |
| **Phase B** | Per-frame **tip candidates**: points on the outer contour that could be a head or tail. Sources: skeleton degree-1 endpoints, contour curvature peaks, or a hypothesised hidden end chosen by S-1. | `Tracking::TipCandidate` on `DetectedBlob::centerline.tipCandidates` | `Centerline::detectEndpoints`, steps (b)–(e) |
| **Phase C** | **Head/tail assignment** and its prerequisites. | | |
| Phase C.1 | Assigning the head and tail roles to two candidates using the previous frame's positions and velocities. | `EndpointResult::headIdx` / `tailIdx` → `DetectedBlob::centerline.headTipIdx` / `tailTipIdx`; predictor state in `Centerline::HeadTailPredictor` | `detectEndpoints`, step (g) |
| Phase C.2 | **Topology classification** of the blob so that later steps know whether both tips are trustworthy. | `Tracking::TopologyState` | `detectEndpoints`, step (f) |

Topology values:

| Value | Rule |
|---|---|
| `Clean` | No hole in the mask, exactly two skeleton endpoints, not in a merge group |
| `SelfCrossed` | A hole (ring) is present, or fewer than two skeleton endpoints were found |
| `Merged` | The worm is in a merge group on this frame (`inMergeGroup` overrides the geometric result) |
| `Lost` | No valid blob |

## Sweeps (how frames are visited)

`CenterlineWorker::doWork` processes worms one at a time. For each worm:

**Sweep 0 — body-length learning.** Read-only walk over every non-ring, non-merged,
non-lost frame. A throwaway skeleton centerline is built on a temporary copy of each blob
and its arc length recorded. `refLength` is the median. Nothing is written to storage.

**Sweep 1 — keyframe-outward.** The frame the user clicked on (`AnnotationItem::frameOfSelection`)
is processed first as a **keyframe bootstrap**: there is no previous frame, so head/tail
are assigned from geometry alone and the predictor is seeded from the result. The pass then
walks forward from keyframe + 1 to the end, and separately backward from keyframe − 1 to 0,
each direction carrying its own `CenterlineSweepState` (predictor plus previous centerline).
If a frame is skipped (merged frame with `skipMergedFrames` on, or lost), the next processed
frame is treated as a fresh bootstrap.

Each frame in Sweep 1 runs `Centerline::processFrame`, which is the five steps below.

## Steps (what happens on one frame)

**Step 1 — detect endpoints.** `detectEndpoints(blob, predictor, baseline, inMergeGroup)`
is a pure function that performs, in order: (a) padded local mask and distance transform,
(b) Guo–Hall skeletonisation into a `SkeletonGraph`, (c) pruning to at most two
degree-1 endpoints by longest path, (d) signed curvature along the outer contour with
local maxima, (e) extension of each skeleton endpoint to the strongest reachable
curvature peak for non-clean frames, or a terminal-axis boundary intersection for clean frames, (f) topology classification
(Phase C.2), (g) head/tail assignment (Phase C.1). Results are written back to the blob.
On `Clean` frames the tip features are sampled into the baseline (Phase A).

Between Step 1 and Step 2, on a `SelfCrossed` frame where the predictor has velocity,
the **omega unzipper** tries cutting the mask along the predicted crossing and re-running
detection; if the cut blob classifies as `Clean` it replaces the original for this frame.

**Step 2 — build the centerline.** Dispatch on topology:

| Branch | Condition | Method |
|---|---|---|
| **D-1** clean graph path | `Clean` | Shortest path through the skeleton graph from head tip to tail tip |
| **D-1** synthetic-hole retry | D-1 path is shorter than half the body length (worm tightly coiled but no hole detected) | Punch a synthetic hole where the body overlaps, re-skeletonise, and run S-1 on it; restore the D-1 path if that fails |
| **S-1** self-crossed route selection | `SelfCrossed`, any number of visible tips | Choose among skeleton routes (below) |
| **S-0** self-crossed unresolved | S-1 found no route within the body-length window | No centerline; the visible tips and their roles are still stored |
| **D-4** fallback contour skeleton | `Merged`, or D-1 failed to produce a centerline | Legacy `Tracking::populateCenterlineFromContour` |

**S-1 route selection** (`src/core/centerlineroutes.cpp`). The skeleton is compressed into
endpoints, junction clusters, and the branches between them. Every edge-simple walk that
starts at a visible endpoint (or, with none visible, at the point nearest a predicted end)
is a candidate. A walk may end at another endpoint, at a junction, part-way along a branch
once it reaches the body length, or after retracing its last branch when the body folds
back along itself; a walk may also close on a hidden start when the two ends touch.
Candidates are resampled and rejected unless their length is within 75–115 % of the body
length. Both head/tail assignments of each survivor are scored, lowest wins:

- length deviation from the body length;
- end type: a visible tip costs nothing, a short branch (no longer than the body half-width
  plus one pixel) a little, and a hidden end more — much more when that end was observed in
  the previous frame, since a visible end rarely vanishes;
- distance of each end from its role prediction, as a likelihood whose spread grows with the
  number of frames since the role was last observed;
- loop orientation: the signed turning (head→tail) must keep the sign of the last trusted
  centerline once both are clearly curved;
- sharp turns taken inside a junction (bodies cross roughly straight).

The body length is the clean-frame baseline once it has 30 samples, otherwise the Sweep 0
median; never the previous frame's output.

**Step 3 — resample** the polyline to `CenterlineSnakeParams::nPoints` evenly spaced
points. The body-length sample for Phase A is taken from this resampled curve.

**Step 4 — snake refinement** (`Clean` frames only). An active contour initialised from
the previous frame's centerline is relaxed toward the distance-transform ridge under
tension, rigidity, and image forces (`CenterlineSnakeParams`). A finished centerline is
never reversed afterwards; orientation is enforced while S-1 chooses a route.

**Step 5 — predictor update.** Head, tail, and centre positions are stored in
`HeadTailPredictor`. Each end also carries an **age**: frames since it was last observed as
a real tip. A hypothesised hidden end updates the position but not the age, and velocity is
kept only for an end observed in both preceding frames. On the next frame the predictor is
rebuilt from stored frames in sweep order with the same rule (`loadPreviousFrameContext`),
looking back up to 60 frames for each end's last observation. The signed turning of the
latest trusted centerline (a clean frame with two tips, or an S-1 route with two visible ends
and a matching length) is kept in `CenterlineSweepState` as the orientation reference.

Head/tail assignment in `detectEndpoints` step (g) uses the same age-weighted likelihood.
On a clean frame that follows a clean frame, the previous centerline's order is a second
check on the two tips.

## Post-sweep passes (per worm, after both directions)

1. **Motion-based head/tail refinement** (`refineHeadTailByMotion`). Over windows
   of at least a few seconds of frames, compares the worm's motion with its recorded head.
   Windows in which more than `maxReversalFraction` of steps oppose the majority direction
   are treated as turning events and skipped. Produces `motionSwapped`, emitted as
   `headTailMotionSwapEvent`.
2. **Geometry-based head/tail refinement** (`refineHeadTailByGeometry`). Collects tip
   geometry features on `Clean` frames and, when the head and tail distributions separate
   significantly (Cohen's d), flips segments whose assignment disagrees. Produces
   `geoSwapped`, emitted as `headTailGeometrySwapEvent`.
3. **Net swap** = XOR of the two lists (a frame flipped by both passes is unchanged),
   emitted as `headTailSwapEvent` and written to `<basename>_headtail_swaps.xlsx`.
4. **Midpoint smoothing** (optional, `setSmoothCenterline`). A degree-2 Savitzky–Golay
   filter over a window of `2 · sgHalfWindow + 1` consecutive frames smooths the
   centerline midpoint used for the trace. On `Clean` frames, the filtered midpoint
   guides a second snake relaxation while the detected head and tail remain pinned.
   Frames with a filtered midpoint outside the blob mask keep their original centerline.

## Coordinate frames

"Video" coordinates are pixels in the source frame; every point
stored on a `DetectedBlob` or `TrackPoint` is in this frame. Inside `detectEndpoints`
the skeleton, distance transform, and `SkeletonGraph::points` are **local** to
`EndpointResult::localBounds`; add the bounds origin to convert back.

## History

The pass was rewritten from a five-pass design into the sweep/step structure above. The
design document for that rewrite (`CENTERLINE_REWRITE_PLAN.md`) was deleted from the tree
in commit `dcce759` and is not the reference for current behaviour; this page is.

### Authoritative visible-tip position and DEBUG cap views

Clean frames fit a terminal body axis to six interior skeleton samples spaced one
pixel apart along the shortest path between endpoints. The first forward
intersection of that axis with a contour segment supplies `TrueTip::point`.
Intersecting segments directly makes the position independent of contour vertex
density, including `CHAIN_APPROX_SIMPLE` compression. Neither curvature scores
nor averages of contour corners select clean tips. If the fit is degenerate or
has no local forward exit, the closest contour-segment point is used instead.
Non-clean frames retain their existing peak/snap and hidden-tip routing.

The selected point is shared by role assignment, stored candidates, D-1 endpoints,
snake pins, and predictor updates. Curvature and width remain supporting contour
features; they do not change the selected clean-frame position.

`TipCandidate::Source::AxisBoundary` identifies axis intersections (yellow in the
candidate overlay). DEBUG cap views show uniformly spaced interior samples and
the fitted ray in cyan, the raw skeleton endpoint in white, and the selected
boundary point in yellow. The log records the fit origin, samples, direction,
estimator, and reason. Existing cap image filenames are retained.

D-2 (two known tips), D-3 (one known tip, hidden prediction) and the 0-tip ring cut were
replaced by S-1. Their `Debug::CenterlineBranch` values remain so older exports still
parse; S-1 reuses the D-3 route debug fields (`04c_d3_possible_paths.png` shows the top
four candidates, the selected one first).

Run `python3 scripts/test_centerline_endpoints.py build` against a configured
CMake Unix Makefiles build. An optional second argument specifies a directory for
DEBUG exports. Regressions include the recorded worm 4 truncation contours and
worm 1 frames 1534-1541, with checks for boundary membership and invariance to
contour densification and reversal, and two worm 5 self-contact sequences (frames
543-590 and 766-791) swept backward, which check body-length consistency and that a
continuously visible end keeps its role.
