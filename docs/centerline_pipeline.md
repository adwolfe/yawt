# Centerline Pipeline: Vocabulary and Structure

This page defines the code names that appear in comments and debug output for the
post-tracking centerline pass: **Phase A / B / C**, **Sweep 0 / 1**, **Step 1–5**,
**D-1 … D-4**, and **0-tip ring cut**. The names date from the pass's incremental
development and are kept because they are embedded in log lines, the Debug tab, and
`Debug::CenterlineBranch`. This is the only place they are defined.

Code: `src/core/centerlineworker.cpp` (orchestration), `src/core/centerlineprocessor.cpp`
(per-frame work), `src/core/centerlinetypes.h` (types), `src/core/centerlinegeometry.cpp`
(geometry helpers).

## What the pass does

Tracking produces, per worm per frame, a `DetectedBlob` (contour, holes, area) and a
`WormTrackPoint` (centroid, search ROI, quality). The centerline pass runs afterwards on
a background thread (`CenterlineWorker::doWork`) and, for every non-lost frame, fills in:

- `DetectedBlob::centerlinePoints` — an ordered polyline from head to tail
- `DetectedBlob::tipCandidates` with `assignedHeadTipIdx` / `assignedTailTipIdx`
- `DetectedBlob::topologyState` — `Clean`, `SelfCrossed`, `Merged`, or `Lost`
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
| **Phase B** | Per-frame **tip candidates**: points on the outer contour that could be a nose or tail. Sources: skeleton degree-1 endpoints, contour curvature peaks, or a hypothesised hidden tip (D-3). | `Tracking::TipCandidate` on `DetectedBlob::tipCandidates` | `Centerline::detectEndpoints`, steps (b)–(e) |
| **Phase C** | **Head/tail assignment** and its prerequisites. | | |
| Phase C.1 | Assigning the head and tail roles to two candidates using the previous frame's positions and velocities. | `EndpointResult::headIdx` / `tailIdx` → `DetectedBlob::assignedHeadTipIdx` / `assignedTailTipIdx`; predictor state in `Centerline::HeadTailPredictor` | `detectEndpoints`, step (g) |
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

**Sweep 1 — keyframe-outward.** The frame the user clicked on (`ClickedItem::frameOfSelection`)
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
(b) Zhang–Suen skeletonisation into a `SkeletonGraph`, (c) pruning to at most two
degree-1 endpoints by longest path, (d) signed curvature along the outer contour with
local maxima, (e) extension of each skeleton endpoint to the strongest reachable
curvature peak, plus a bilateral cap-midpoint estimate, (f) topology classification
(Phase C.2), (g) head/tail assignment (Phase C.1). Results are written back to the blob.
On `Clean` frames the tip features are sampled into the baseline (Phase A).

Between Step 1 and Step 2, on a `SelfCrossed` frame where the predictor has velocity,
the **omega unzipper** tries cutting the mask along the predicted crossing and re-running
detection; if the cut blob classifies as `Clean` it replaces the original for this frame.

**Step 2 — build the centerline.** Dispatch on topology and on how many tips are known:

| Branch | Condition | Method |
|---|---|---|
| **D-1** clean graph path | `Clean` | Shortest path through the skeleton graph from head tip to tail tip |
| **D-1** synthetic-hole retry | D-1 path is suspiciously short relative to `refLength` (worm tightly coiled but no hole detected) | Punch a synthetic hole where the body overlaps, re-skeletonise, and retry; restore the D-1 path if that fails |
| **D-2** two known tips | `SelfCrossed`, both tips found | Trace the skeleton through the loop from the head tip to the tail tip (loop-aware arc routing) |
| **D-3** one known tip, hidden prediction | `SelfCrossed`, one tip found | Walk from the known tip through the loop junction toward a predicted position for the hidden tip; register the far end as a `HypothesizedHidden` candidate |
| **0-tip ring cut** | `SelfCrossed`, no tips found | Score candidate cuts across the ring using the predictor and pick the best resulting centerline |
| **D-4** fallback contour skeleton | Any branch above failed to produce a usable centerline | Legacy `Tracking::populateCenterlineFromContour` |

The branch taken is recorded in `Debug::CenterlineFrameDebug::branch`
(`Debug::CenterlineBranch`) together with a list of decision strings.

**Step 3 — resample** the polyline to `CenterlineSnakeParams::nPoints` evenly spaced
points. The body-length sample for Phase A is taken from this resampled curve.

**Step 4 — snake refinement** (`Clean` frames only). An active contour initialised from
the previous frame's centerline is relaxed toward the distance-transform ridge under
tension, rigidity, and image forces (`CenterlineSnakeParams`). The **right-hand-rule
veto** compares the total signed turning angle of the result with the previous frame's
and rejects candidates whose sense of rotation flipped, unless the body is nearly
straight.

**Step 5 — predictor update.** Head, tail, and centre positions and velocities are
stored in `HeadTailPredictor` for the next frame.

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
4. **Tip smoothing** (optional, `setSmoothCenterline`). A degree-2 Savitzky–Golay filter
   over a window of `2 · sgHalfWindow + 1` frames smooths head and tail tip positions,
   then `relaxCenterlineToSmoothedTips` re-runs the snake on each `Clean` frame with the
   smoothed tips pinned.

## Coordinate frames

"Video" (also written "world") coordinates are pixels in the source frame; every point
stored on a `DetectedBlob` or `WormTrackPoint` is in this frame. Inside `detectEndpoints`
the skeleton, distance transform, and `SkeletonGraph::points` are **local** to
`EndpointResult::localBounds`; add the bounds origin to convert back.

## History

The pass was rewritten from a five-pass design into the sweep/step structure above. The
design document for that rewrite (`CENTERLINE_REWRITE_PLAN.md`) was deleted from the tree
in commit `dcce759` and is not the reference for current behaviour; this page is.
