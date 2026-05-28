# Centerline Worker Rewrite — Restartable Plan

This file is the canonical plan for rewriting `src/core/centerlineworker.cpp`
and the supporting `detectEndpoints()` function in `src/data/trackingcommon.{h,cpp}`.

If this work is interrupted, restart by re-reading this file end-to-end. Each
phase is checkpointed so a partial rewrite can be resumed without losing
context.

---

## 0. Current state of the repo (verified against actual files, NOT prior summaries)

A previous agent attempt was claimed to have added `detectEndpoints()`,
`SkeletonGraph`, `TrueTip`, and `EndpointResult` to `trackingcommon.{h,cpp}`.
**That work was NOT actually committed — it does not exist in the codebase.**
Verified with `grep -n "detectEndpoints\|SkeletonGraph\|TrueTip\|EndpointResult"
src/data/trackingcommon.{h,cpp} src/core/centerlineworker.cpp` → no matches.

What IS present:
- `Tracking::TipCandidate::Source::HypothesizedHidden` enum value at
  [trackingcommon.h:185](src/data/trackingcommon.h:185) (Bug 1 fix retained).
- Orange hollow-square renderer for `HypothesizedHidden` at
  [videoloader.cpp:1126-1134](src/gui/widgets/videoloader.cpp:1126).
- Inline D-3 in `runAssignmentDirection` at
  [centerlineworker.cpp:1474-1522](src/core/centerlineworker.cpp:1474)
  (Bug 2 fix retained).
- `refineCenterlineSnakeFromOneTip()` uses farthest-mask-pixel scan
  (Bug 3 fix retained), at
  [centerlineworker.cpp:819-882](src/core/centerlineworker.cpp:819).

What is NOT present (must be added):
- `SkeletonGraph`, `TrueTip`, `EndpointResult` structs.
- `detectEndpoints()` function.
- The 2-sweep / 5-step `doWork()` structure.

The current `doWork()` is at
[centerlineworker.cpp:1146-1626](src/core/centerlineworker.cpp:1146) and uses
5 sequential passes (Pass 1 length, Pass 2 centerline+snake, Pass 3 baseline,
Pass 4a topology, Pass 4b assign + inline D-3, Pass 5 D-2).

Existing helpers in `trackingcommon.cpp` we will reuse:
- `skeletonizeBinaryMask()` — [trackingcommon.cpp:74](src/data/trackingcommon.cpp:74) (file-static, in anon namespace)
- `dijkstraSkeleton()` — [trackingcommon.cpp:107](src/data/trackingcommon.cpp:107) (file-static)
- `reconstructCenterlinePath()` — [trackingcommon.cpp:149](src/data/trackingcommon.cpp:149) (file-static)
- `buildCenterlineMask()` — [trackingcommon.cpp:181](src/data/trackingcommon.cpp:181) (file-static)
- `extractCenterlineFromMask()` — [trackingcommon.cpp:220](src/data/trackingcommon.cpp:220) (file-static)
- `findTipCandidates()` — [trackingcommon.cpp:353](src/data/trackingcommon.cpp:353) (public, contains the curvature-peak math we'll reuse)
- `classifyTopology()` — [trackingcommon.cpp:601](src/data/trackingcommon.cpp:601) (public, will be SUPERSEDED but kept as fallback)
- `assignHeadTail()` — [trackingcommon.cpp:640](src/data/trackingcommon.cpp:640) (public, kept; will keep being used by the new design for now)
- `populateCenterlineFromContour()` — [trackingcommon.cpp:920](src/data/trackingcommon.cpp:920) (public)
- `populateCenterlineFromContourWithCut()` — [trackingcommon.cpp:939](src/data/trackingcommon.cpp:939) (public)

Existing helpers in `centerlineworker.cpp` we will KEEP (file-static):
- `ptDist`, `arcLen`, `nearestContourIdx`, `nearestContourPoint`,
  `nearestHolePoint`, `resample`, `blobCentroid`, `fillBlobMask` — geometry utilities.
- `runDijkstraInMask`, `reconstructPath`, `geodesicPathInMask`, `GeodesicResult`
  — mask-Dijkstra used by D-2 / D-3.
- `refineSnakeCore`, `buildSnakeMask`, `refineCenterlineSnakeFromTips`,
  `refineCenterlineSnakeFromOneTip` — snake refinement (D-2/D-3 wrappers).
- `CenterlineState`, `totalSignedTurningAngle` — for right-hand-rule check.

What we will DELETE from `centerlineworker.cpp`:
- `processOneFrame()` ([line 887](src/core/centerlineworker.cpp:887))
- `orientByShape()` ([line 137](src/core/centerlineworker.cpp:137))
- `buildSplitRingCandidate()` ([line 225](src/core/centerlineworker.cpp:225))
- `buildSplitRingCandidateWithCut()` ([line 255](src/core/centerlineworker.cpp:255))
- `diffBasedCutHint()` ([line 302](src/core/centerlineworker.cpp:302))
- `refineCenterlineSnake()` legacy wrapper ([line 725](src/core/centerlineworker.cpp:725))
- `inconsistentWithPreviousFrame()`, `continuityScore()` ([lines 157, 186](src/core/centerlineworker.cpp:157)) — only called from `processOneFrame`.

External callers that must keep working (DO NOT BREAK):
- `trackingdatastorage.cpp:585` — calls `populateCenterlineFromContour()`.
- `trackingmanager.cpp:484` and `:2232` — call `populateCenterlineFromContour()`.
- `videoloader.cpp:1109-1152` — reads `blob.tipCandidates`, `tc.source`,
  `blob.assignedHeadTipIdx`, `blob.assignedTailTipIdx`. The `source` field
  is used to pick the rendering style (filled green = SkeletonEndpoint,
  hollow magenta = CurvaturePeak, hollow orange = HypothesizedHidden).

---

## 1. Why the prior agent attempt failed (and how to avoid it)

The user reports: **the green points (skeleton endpoint candidate dots) were
"completely messed up" both before AND after the changes**. Green dots are
rendered at [videoloader.cpp:1116](src/gui/widgets/videoloader.cpp:1116) for
candidates with `tc.source == TipCandidate::Source::SkeletonEndpoint`.

Most likely failure modes (defensively guard against each):

1. **`TrueTip → TipCandidate` conversion drops the source tag.** If the new
   code writes back `TrueTip` objects without explicitly setting
   `tc.source = SkeletonEndpoint`, default-constructed TipCandidates have
   source `SkeletonEndpoint` (value 0) — but if anyone changes the default
   the dots vanish. Be explicit about source.

2. **Endpoint pruning kills valid endpoints.** The new `detectEndpoints()`
   does pruning to enforce ≤2 endpoints. If pruning is overly aggressive on
   degenerate skeletons (e.g. a very short worm with two close endpoints +
   one spur), it can leave 0 endpoints — so no green dots. Pruning must keep
   the longest-path pair, never delete both.

3. **Curvature-peak extension snaps endpoints to the wrong peak.** The
   "extend skeleton endpoint to the nearest curvature peak within
   `DT(endpoint) × 1.5`" rule can pick a body kink instead of a true tip
   if the threshold is too generous on a coiled worm. The extended position
   becomes the rendered green dot — so a wrong extension shows up as a
   visibly misplaced green dot.

4. **Endpoint detection runs on a different mask than the original
   `findTipCandidates`.** `findTipCandidates` uses `buildCenterlineMask`
   which applies a `MORPH_CLOSE` for ring blobs at line 213-214. If
   `detectEndpoints` builds the mask differently, results disagree on
   ring-edge cases and green-dot positions shift.

5. **Per-frame storage write-back missing or out-of-order.** The blob must be
   written back to storage AFTER all per-frame mutations are done. If we
   write back after Step 1 (detectEndpoints) and again after Step 4 (snake
   refinement), the second write overwrites the first only if it includes
   the tipCandidates / assignment indices. This was a recurring gotcha in
   the existing 5-pass code (see `m_storage->setDetectedBlobForFrame` calls
   at lines 1216, 1269, 1292, 1376, 1422, 1524, 1607).

6. **`extractCenterlineFromMask` is in an anonymous namespace.** It's used
   internally by `populateCenterlineFromContour`. The new design has us
   building a `SkeletonGraph` ourselves so we don't depend on this — but
   if we change our minds later and call it directly, we will need to expose
   it.

**Mitigation strategy:** Build incrementally. After each phase, verify by
running the app and visually inspecting the green dots on a known-good
clip. Do not delete any of the existing 5-pass code until the new code
demonstrates correct green dots on the same clip.

---

## 2. Implementation phases (in order)

Each phase is a logical commit boundary. Phases 1-3 are additive (do not
break the existing pipeline). Phase 4 is the cut-over. Phases 5-6 are
cleanup.

### Phase 1 — Add new structs to `trackingcommon.h` ✅ DONE

**Goal:** declare types so `detectEndpoints()` can be implemented in Phase 2.

**File:** `src/data/trackingcommon.h`

**Insert location:** immediately after the `TipCandidate` struct definition
(currently ends at [line 188](src/data/trackingcommon.h:188)), before the
`enum class TopologyState` block at [line 201](src/data/trackingcommon.h:201).

**Add (in `namespace Tracking`, in this exact order so each can refer to
prior):**

```cpp
/**
 * @brief Skeleton graph extracted from a binary worm mask.
 *
 * The skeleton is the Zhang-Suen 1-pixel-wide medial-axis representation;
 * each foreground pixel is a graph node, connected to its 8-neighbour
 * skeleton pixels (orthogonal weight 1, diagonal √2). Endpoint indices
 * are the degree-1 nodes — body tips when the topology is unobstructed.
 *
 * `indexImage` maps mask coords → graph index (-1 outside skeleton). All
 * coords are LOCAL to the bounding box used to build the mask; the caller
 * adds the box origin when converting back to video coords.
 */
struct SkeletonGraph {
    std::vector<cv::Point>        points;          // mask-local coords
    std::vector<std::vector<int>> adjacency;
    std::vector<int>              endpointIndices; // degree-1 nodes
    cv::Mat                       indexImage;      // CV_32SC1: pixel → index
    cv::Mat                       skeleton;        // CV_8UC1 binary, 0 or 255
};

/**
 * @brief A high-confidence body tip — skeleton endpoint optionally extended
 *        to the matching outer-contour curvature peak.
 *
 * `point` is the best-estimate world position used for downstream geodesic
 * paths, snake pinning, and head/tail assignment. `skelPoint` is the raw
 * skeleton degree-1 node in world coords (handy when the extension is
 * disputed). `extended == true` iff `point` came from a curvature peak,
 * not the skeleton endpoint itself.
 */
struct TrueTip {
    cv::Point2f point;       // world coords; either curvature peak or skel pt
    cv::Point2f skelPoint;   // world coords; raw skeleton degree-1 node
    float       curvature = 0.f;
    float       width     = 0.f;
    bool        extended  = false;
};

/**
 * @brief Combined output of `detectEndpoints()`.
 *
 * Bundles the skeleton graph, distance transform, true tips, topology
 * classification, and head/tail role assignment. Designed to be computed
 * once per frame in the new pipeline and reused for every downstream step
 * (centerline build, snake refinement, predictor update).
 *
 * Coordinates: `tips`, `headIdx`, `tailIdx` are indices into `tips`; the
 * `point` and `skelPoint` fields inside each TrueTip are in WORLD (video)
 * coords. The skeleton graph and distTransform are LOCAL to `localBounds`.
 */
struct EndpointResult {
    SkeletonGraph        skeleton;
    cv::Mat              distTransform;             // CV_32F, local coords
    cv::Rect             localBounds;               // origin offset (LOCAL→WORLD)
    std::vector<TrueTip> tips;                      // 0–2 entries
    TopologyState        topology = TopologyState::Unknown;
    int                  headIdx = -1;              // index into `tips`
    int                  tailIdx = -1;              // index into `tips`

    bool valid() const {
        return !skeleton.points.empty() && !skeleton.skeleton.empty();
    }
};
```

**Then add the function declaration**, after the existing `findTipCandidates`
declaration at [line 549](src/data/trackingcommon.h:549):

```cpp
/**
 * @brief Single-pass endpoint detector that replaces the old
 *        findTipCandidates + classifyTopology + assignHeadTail trio for
 *        the new centerline pipeline.
 *
 * Pure function: depends only on `blob.contourPoints`, `blob.holeContourPoints`,
 * the per-worm `predictor` (for tie-breaking head/tail), and the per-worm
 * `baseline` (for the curvature-peak threshold).
 *
 * Steps performed (see implementation):
 *   (a) Build padded local mask + distance transform.
 *   (b) Skeletonize → assemble SkeletonGraph (points, adjacency, indexImage).
 *   (c) Find degree-1 nodes; if >2, prune to the longest-path pair.
 *   (d) Compute outer-contour signed curvature; find local maxima.
 *   (e) For each surviving skeleton endpoint, search for a curvature peak
 *       within `DT(endpoint) * 1.5` pixels. If found, set `point` to that
 *       peak and mark `extended = true`. Otherwise `point = skelPoint`
 *       (the nearest contour point to the skeleton endpoint).
 *   (f) Classify topology: holes present OR <2 tips → SelfCrossed; else Clean.
 *       (Caller passes `inMergeGroup` flag to override → Merged/Lost.)
 *   (g) Assign head/tail by minimum total predicted-distance, with velocity
 *       used as tiebreaker when the cost spread is < 10%.
 *
 * The result's tips/headIdx/tailIdx are designed to be written back to
 * `blob.tipCandidates`, `blob.assignedHeadTipIdx`, `blob.assignedTailTipIdx`
 * by the caller (centerlineworker), with `tc.source = SkeletonEndpoint`
 * for every tip, so the existing renderer at videoloader.cpp:1116 keeps
 * showing them as filled green dots.
 */
EndpointResult detectEndpoints(const DetectedBlob& blob,
                               const HeadTailPredictor& predictor,
                               const TipFeatureBaseline& baseline,
                               bool inMergeGroup);
```

**Validation after Phase 1:**
- File compiles. Run `cmake --build build` (or whatever the project uses).
- No callers of `detectEndpoints` yet, so behaviour is unchanged.
- `git diff src/data/trackingcommon.h` shows ONLY additions.

---

### Phase 2 — Implement `detectEndpoints()` in `trackingcommon.cpp` ✅ DONE

**Goal:** add the new function as a peer of `findTipCandidates()`. Old
function stays untouched and still works.

**File:** `src/data/trackingcommon.cpp`

**Insert location:** immediately AFTER the closing `}` of `findTipCandidates()`
at [line 586](src/data/trackingcommon.cpp:586), BEFORE the `// ── Topology
classification (Phase C.2) ────────...` comment block at line 588.

**Critical implementation notes (read before writing code):**

- **Reuse `buildCenterlineMask()`** at [trackingcommon.cpp:181](src/data/trackingcommon.cpp:181)
  for the mask. This function lives in the anonymous namespace within the
  same file, so `detectEndpoints` (also defined in this TU) can call it
  directly. ✅ no header changes needed.

- **Reuse `skeletonizeBinaryMask()`** at [trackingcommon.cpp:74](src/data/trackingcommon.cpp:74) — same anonymous namespace.

- **Reuse `dijkstraSkeleton()`** at [trackingcommon.cpp:107](src/data/trackingcommon.cpp:107) — same anonymous namespace.

- **Build the SkeletonGraph identically to `extractCenterlineFromMask`**
  ([line 220-280](src/data/trackingcommon.cpp:220)). That function builds
  a (points, adjacency, indexImage) trio; lift the same code into our
  new struct. This guarantees green dots match what users have seen.

- **Curvature math: COPY from `findTipCandidates()`**
  ([line 425-441](src/data/trackingcommon.cpp:425)) verbatim. Same window
  size, same sign convention, same threshold. The peak-detection loop
  ([lines 451-463](src/data/trackingcommon.cpp:451)) is also lifted.

- **Curvature threshold for the extension match (step e)**:
  - When `baseline.isReliable()` (≥30 samples): use
    `max(0.5f * baseline.meanAbsCurvature, 0.04f)` as the floor.
    The 0.5× factor matches the prior summary's "baseline.meanAbsCurvature × 0.5".
  - Otherwise: use the same default `minCurvatureMagnitude = 0.08f` that
    `findTipCandidates` defaults to.

- **Extension search (step e) per skeleton endpoint:**
  1. Convert skel endpoint local coords → world.
  2. Find nearest outer-contour point → fallback `skelPoint`.
  3. Get `DT(skelEndpoint)` from the local distTransform.
  4. Search radius = `max(DT * 1.5f, 4.0f)` world pixels.
  5. Among curvature peaks within that radius, pick the one with the
     largest `|curvature|` (NOT the closest — we want the strongest tip
     signal within reach, otherwise we snap to body kinks).
  6. If chosen, set `point = peak position`, `extended = true`,
     copy curvature + width from that peak. Otherwise `point = skelPoint`,
     `extended = false`, sample local DT/curvature at the contour point.

- **Endpoint pruning (step c):** if `endpointIndices.size() > 2`, run
  `dijkstraSkeleton` from each endpoint, find the (i, j) pair with the
  longest shortest-path distance. Drop all other indices. **NEVER drop
  below 2 if 2+ existed; never drop the only endpoint if size == 1.**

- **Topology (step f):**
  ```cpp
  if (!blob.isValid)         topology = Lost;
  else if (inMergeGroup)     topology = Merged;
  else if (!holes.empty() || tips.size() < 2)  topology = SelfCrossed;
  else                       topology = Clean;
  ```

- **Head/tail assignment (step g):**
  - 0 tips → headIdx = tailIdx = -1, return.
  - 1 tip → assign to the role whose predicted position is closer
    (via `predictor.lastHeadPos + velHead` vs `lastTailPos + velTail` if
    `hasVelocity`, else just `lastHeadPos` vs `lastTailPos`). Leave the
    OTHER role at -1 (D-3 will fill it later). If `!predictor.hasPrev`,
    arbitrary: assign to head, leave tail at -1.
  - 2 tips → enumerate (i,j), i≠j; cost = dist(tips[i], predHead) +
    dist(tips[j], predTail). Pick min-cost ordered pair. If the runner-up
    cost is within 10% of the winner AND `predictor.hasVelocity`, use
    velocity-extrapolated positions for the tiebreak (the same predicted
    points but recomputed with the spec's "lastPos + 2 × vel" rule —
    a longer extrapolation that magnifies the directional disagreement).

- **Feature sampling for `width`:** copy the `widthAt()` lambda from
  `findTipCandidates` ([lines 483-517](src/data/trackingcommon.cpp:483)) —
  the perpendicular-inward DT probe. This MUST match `findTipCandidates`
  so baseline samples are comparable across the pipeline.

- **Logging:** copy the structure of `findTipCandidates`'s debug log
  ([lines 562-583](src/data/trackingcommon.cpp:562)). Use `lcDataCommon`
  category. Log the endpoint count BEFORE pruning, AFTER pruning, and
  per-tip extension status.

**Pseudocode skeleton (write the real code following this):**

```cpp
EndpointResult detectEndpoints(const DetectedBlob& blob,
                               const HeadTailPredictor& predictor,
                               const TipFeatureBaseline& baseline,
                               bool inMergeGroup)
{
    EndpointResult r;
    if (!blob.isValid) { r.topology = TopologyState::Lost; return r; }
    if (inMergeGroup)  { r.topology = TopologyState::Merged; /* still build skeleton if useful */ }
    if (blob.contourPoints.size() < 8) return r;

    // (a) mask + DT
    cv::Mat mask;
    r.localBounds = buildCenterlineMask(blob, mask);
    if (mask.empty()) return r;
    cv::distanceTransform(mask, r.distTransform, cv::DIST_L2, 3);

    // (b) skeleton + adjacency + indexImage  (lifted from extractCenterlineFromMask)
    cv::Mat skel = skeletonizeBinaryMask(mask);
    if (cv::countNonZero(skel) < 2) return r;
    r.skeleton.skeleton = skel;
    r.skeleton.indexImage = cv::Mat(skel.size(), CV_32SC1, cv::Scalar(-1));
    // ... populate r.skeleton.points + indexImage ...
    // ... populate r.skeleton.adjacency ...
    // ... populate r.skeleton.endpointIndices (degree==1) ...

    // (c) prune to ≤2 endpoints (longest-path pair)
    if (r.skeleton.endpointIndices.size() > 2) {
        // double Dijkstra; keep best (a, b) pair
    }

    // (d) signed curvature + local maxima  (lifted from findTipCandidates)
    // ... build contourLocal, curvature[], curvaturePeakIdx ...

    // (e) for each surviving skeleton endpoint, attempt extension
    const float curvFloor = baseline.isReliable()
        ? std::max(0.5f * baseline.meanAbsCurvature, 0.04f)
        : 0.08f;
    for (int epIdx : r.skeleton.endpointIndices) {
        // ... compute world coords, DT(endpoint), search radius ...
        // ... pick strongest curvature peak within radius (or fall back) ...
        TrueTip t;
        // populate t.point, t.skelPoint, t.curvature, t.width, t.extended
        r.tips.push_back(t);
    }

    // (f) topology
    if (!inMergeGroup) {
        const bool hasRing = !blob.holeContourPoints.empty();
        r.topology = (hasRing || r.tips.size() < 2)
                       ? TopologyState::SelfCrossed
                       : TopologyState::Clean;
    }

    // (g) head/tail assignment
    // ... 0/1/2-tip branches as described above ...

    // log
    // ...

    return r;
}
```

**Validation after Phase 2:**
- File compiles.
- Still no callers, so runtime behaviour unchanged.
- (Optional) Add a temporary debug call in `centerlineworker.cpp` Pass 4a
  that calls both `findTipCandidates` and `detectEndpoints` and logs any
  disagreement — REMOVE before commit.

---

### Phase 3 — Stage the new doWork() alongside the old one ✅ DONE

**Goal:** keep the old `doWork()` callable while we build the new one,
so we can A/B test.

**File:** `src/core/centerlineworker.cpp`

**Approach:** rename the existing `doWork()` to `doWorkLegacy()`, add a
new `doWork()` that immediately calls `doWorkLegacy()`. This keeps the
public slot signature stable and gives us a sandbox for the new code.

**Steps:**

1. At [line 1146](src/core/centerlineworker.cpp:1146), rename the existing
   `void CenterlineWorker::doWork()` to
   `void CenterlineWorker::doWorkLegacy()`.

2. Add a forward declaration in `centerlineworker.h` if needed
   (private method) — actually, since it's a private static-style helper,
   we can add it as a private member of `CenterlineWorker` in the header.
   Edit `centerlineworker.h`:
   - After line 36 (`void doWork();`), add:
     `void doWorkLegacy();`
   - Inside `class CenterlineWorker` `private:` section.

3. Above the renamed `doWorkLegacy()`, add the new `doWork()`:

   ```cpp
   void CenterlineWorker::doWork()
   {
       // While the new 2-sweep architecture is being built up, keep the
       // legacy 5-pass implementation reachable. Toggle this flag when
       // the new path is ready for visual A/B testing.
       constexpr bool kUseNewPipeline = false;
       if (kUseNewPipeline) {
           doWorkNew();
       } else {
           doWorkLegacy();
       }
   }
   ```

4. Add a stub `doWorkNew()` that just emits `finished()`:

   ```cpp
   void CenterlineWorker::doWorkNew()
   {
       // TODO Phase 4: implement new 2-sweep pipeline.
       emit finished();
   }
   ```

5. Add `doWorkNew()` to `centerlineworker.h` `private:` section.

**Validation after Phase 3:**
- App still works exactly as before (kUseNewPipeline = false).
- `git diff src/core/centerlineworker.{h,cpp}` shows only the rename,
  the new dispatcher, and the empty stub.

---

### Phase 4 — Implement the new pipeline in `doWorkNew()` ✅ CODE COMPLETE (awaiting user visual validation)

**Goal:** implement the 2-sweep / 5-step structure. Keep it parallel to
the legacy until visual validation passes.

**Implementation deviations from the pseudocode in this plan, for the
record:**
- `processFrame` is a single lambda taking `(i, predictor&, prevState&,
  isKeyframeBootstrap)` rather than two separate `runDirection` calls.
  The keyframe is processed once with the bootstrap flag, then forward
  and backward are independent passes seeded from the keyframe's
  resulting predictor + state.
- Keyframe bootstrap is two-phase: detectEndpoints leaves head/tail at
  -1 (no predictor), then we arbitrarily set (0, 1) so Step 2 can build
  a centerline; after Step 4 we re-derive head/tail from the
  centerline's natural front/back via `nearestCandidateIdx`. This
  matches the legacy seedPredictor behaviour at lines 1408-1443 of the
  old `doWorkLegacy`.
- A new file-static helper `skeletonGraphPath()` was added (alongside
  `geodesicPathInMask`) — runs Dijkstra on the `SkeletonGraph`'s
  adjacency vector and emits world-coords path points. Saves having to
  expose `Tracking::dijkstraSkeleton` (which is currently file-static in
  `trackingcommon.cpp`).
- Ring-cut endpoints follow the decision recorded in §4: predicted nose
  → predicted tail when `prevState` is valid; major axis of the largest
  hole as fallback (via `cv::fitEllipse`).
- Predictor's `refDistance` is updated each frame from the running
  `refLength` so the assignment cost scales correctly on long-bodied or
  short-bodied worms.

**To enable for testing:** flip `kUseNewPipeline` to `true` at
`src/core/centerlineworker.cpp` (search for the constexpr). Build, run,
inspect green dots + cyan/red rings + centerlines.

**File:** `src/core/centerlineworker.cpp`

The new `doWorkNew()` mirrors the structure of `doWorkLegacy()` but
collapses Pass 3, 4a, 4b, 5 into the per-frame Sweep 1.

#### Sweep 0 — Learn body length (identical to legacy Pass 1)

Code is essentially [centerlineworker.cpp:1190-1227](src/core/centerlineworker.cpp:1190).
Lift it verbatim, but DO NOT call `populateCenterlineFromContour` and
write back to storage — Sweep 0's only job is to compute `refLength`.
Use a TEMPORARY local copy of the blob to skeletonize so storage isn't
mutated yet.

```cpp
// Sweep 0 — body-length sample
std::vector<float> validLengths;
for (const Tracking::WormTrackPoint& tp : sortedPoints) {
    if (tp.quality == Lost || tp.quality == Merged) continue;
    QMap<int, Tracking::DetectedBlob> frameBlobs =
        m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
    if (!frameBlobs.contains(wormId)) continue;
    Tracking::DetectedBlob blob = frameBlobs[wormId];
    if (!blob.isValid || blob.contourPoints.empty()) continue;
    if (!blob.holeContourPoints.empty()) continue;  // skip rings
    Tracking::DetectedBlob temp = blob;             // local, NOT written back
    if (Tracking::populateCenterlineFromContour(temp) &&
        temp.centerlinePoints.size() >= 2) {
        std::vector<cv::Point2f> p(temp.centerlinePoints.begin(),
                                   temp.centerlinePoints.end());
        validLengths.push_back(arcLen(p));
    }
}
float refLength = 0.f;
if (!validLengths.empty()) { /* median */ }
```

#### Sweep 1 — Per-frame loop

For each worm:
1. Find `keyframeIdx` (same as legacy [lines 1232-1241](src/core/centerlineworker.cpp:1232)).
2. Compute the keyframe's frame state to seed the predictor and
   CenterlineState (same role as legacy [lines 1279-1296](src/core/centerlineworker.cpp:1279)
   and [lines 1408-1443](src/core/centerlineworker.cpp:1408), unified).
3. Run the per-frame lambda forward then backward from the keyframe.

Per-frame lambda (5 steps, in order):

```cpp
auto runDirection = [&](int startIdx, int step,
                        Tracking::HeadTailPredictor predictor,
                        CenterlineState prevState) {
    const int n = static_cast<int>(sortedPoints.size());
    for (int i = startIdx; i >= 0 && i < n; i += step) {
        const Tracking::WormTrackPoint& tp = sortedPoints[i];
        if (tp.quality == Lost) {
            predictor = Tracking::HeadTailPredictor{};
            prevState.valid = false;
            continue;
        }

        QMap<int, Tracking::DetectedBlob> frameBlobs =
            m_storage->getDetectedBlobsForFrame(tp.frameNumberOriginal);
        if (!frameBlobs.contains(wormId)) continue;
        Tracking::DetectedBlob blob = frameBlobs[wormId];

        // ── Determine merge-group flag (same as legacy Pass 4a)
        const QList<QList<int>> mergeGroups =
            m_storage->getMergeGroupsForFrame(tp.frameNumberOriginal);
        bool inMerge = false;
        for (const QList<int>& g : mergeGroups)
            if (g.size() > 1 && g.contains(wormId)) { inMerge = true; break; }

        // ── STEP 1: detect endpoints (replaces tip-cand + topology + assign)
        Tracking::TipFeatureBaseline baseline = m_storage->getTipBaseline(wormId);
        Tracking::EndpointResult er =
            Tracking::detectEndpoints(blob, predictor, baseline, inMerge);

        // Write the EndpointResult back into blob.tipCandidates so the
        // renderer keeps showing green dots. EVERY tip gets source =
        // SkeletonEndpoint regardless of `extended` — extension just moves
        // the position; it's still a real body endpoint, not a curvature
        // peak in the rendering sense.
        blob.tipCandidates.clear();
        for (const Tracking::TrueTip& t : er.tips) {
            Tracking::TipCandidate tc;
            tc.point     = t.point;
            tc.curvature = t.curvature;
            tc.width     = t.width;
            tc.source    = Tracking::TipCandidate::Source::SkeletonEndpoint;
            blob.tipCandidates.push_back(tc);
        }
        blob.assignedHeadTipIdx = er.headIdx;
        blob.assignedTailTipIdx = er.tailIdx;
        blob.topologyState      = er.topology;

        // On Clean frames, sample baseline + length (replaces legacy Pass 3)
        if (er.topology == Tracking::TopologyState::Clean &&
            er.tips.size() == 2) {
            for (const Tracking::TrueTip& t : er.tips) {
                m_storage->recordTipFeatureSample(
                    wormId, std::abs(t.curvature), t.width);
            }
            // Body length recorded after STEP 3 below using the resampled centerline.
        }

        // ── STEP 2: build centerline based on topology
        std::vector<cv::Point2f> centerline;
        cv::Point2f overlapCenter;
        bool hasOverlap = false;
        const int nPts = std::max(4, m_snakeParams.nPoints);

        if (er.topology == Tracking::TopologyState::Clean &&
            er.headIdx >= 0 && er.tailIdx >= 0) {
            // Dijkstra on skeleton graph between the two TrueTip points.
            // Map er.tips[headIdx].skelPoint → graph index via indexImage,
            // run dijkstraSkeleton, reconstruct path, extend ends from the
            // graph path's last skeleton points to the world-coords TrueTip
            // (curvature-peak-extended) positions.
            //
            // helper: world tip → graph idx:
            //   localPt = skelPoint - localBounds.tl()
            //   idx = er.skeleton.indexImage.at<int>(localPt.y, localPt.x)
            // ... call dijkstraSkeleton(points, adjacency, headGraphIdx) ...
            // ... reconstructCenterlinePath(...) → centerline (world coords)
            // ... if extension was applied (TrueTip.extended), prepend or
            //     append the extended `point` to the path so the line
            //     reaches the curvature peak, not just the skel endpoint.
        }
        else if (er.topology == Tracking::TopologyState::SelfCrossed) {
            const bool hasHole = !blob.holeContourPoints.empty();
            const int hIdx = er.headIdx, tIdx = er.tailIdx;
            const bool hasHead = (hIdx >= 0);
            const bool hasTail = (tIdx >= 0);

            if (hasHole) {
                // Synthetic-hole punch: cut a thin line across the hole, then
                // re-skeletonize. Uses populateCenterlineFromContourWithCut.
                // Cut endpoints: predicted-nose to predicted-tail line
                // intersected with the hole, OR the simpler legacy approach
                // — use the prevState centerline midpoint as a hint.
                //
                // Fallback: if cut fails AND both tips assigned, fall through
                // to D-2 below.
                cv::Point2f cutA, cutB;
                // ... derive cutA, cutB ...
                if (Tracking::populateCenterlineFromContourWithCut(
                        blob, cutA, cutB, /*thickness*/ 3) &&
                    blob.centerlinePoints.size() >= 2) {
                    centerline.assign(blob.centerlinePoints.begin(),
                                      blob.centerlinePoints.end());
                }
            }
            if (centerline.empty() && hasHead && hasTail) {
                // D-2: geodesic head→tail through mask, then snake refine.
                refineCenterlineSnakeFromTips(blob,
                    blob.tipCandidates[hIdx].point,
                    blob.tipCandidates[tIdx].point,
                    nPts, m_snakeParams,
                    centerline, overlapCenter, hasOverlap);
            }
            else if (centerline.empty() && (hasHead || hasTail)) {
                // D-3: one tip → farthest-mask-pixel geodesic + snake.
                const int knownIdx = hasHead ? hIdx : tIdx;
                const cv::Point2f known = blob.tipCandidates[knownIdx].point;
                cv::Point2f hidden;
                if (refineCenterlineSnakeFromOneTip(blob, known, nPts,
                        m_snakeParams, centerline, hidden,
                        overlapCenter, hasOverlap)) {
                    // Append a TipCandidate for the hidden tip so the
                    // renderer shows it as orange.
                    Tracking::TipCandidate hyp;
                    hyp.point  = hidden;
                    hyp.source = Tracking::TipCandidate::Source::HypothesizedHidden;
                    blob.tipCandidates.push_back(hyp);
                    const int newIdx = static_cast<int>(blob.tipCandidates.size()) - 1;
                    if (hasHead) blob.assignedTailTipIdx = newIdx;
                    else         blob.assignedHeadTipIdx = newIdx;
                }
            }
        }

        if (centerline.empty()) {
            // D-4 fallback: legacy contour-skeleton.
            Tracking::populateCenterlineFromContour(blob);
            centerline.assign(blob.centerlinePoints.begin(),
                              blob.centerlinePoints.end());
        }

        if (centerline.size() < 2) {
            prevState.valid = false;
            continue;
        }

        // ── STEP 3: resample to nPoints
        if (static_cast<int>(centerline.size()) != nPts)
            centerline = resample(centerline, nPts);

        // Body-length sample on clean frames (was legacy Pass 3 tail).
        if (er.topology == Tracking::TopologyState::Clean &&
            er.tips.size() == 2) {
            m_storage->recordBodyLengthSample(wormId, arcLen(centerline));
        }

        // ── STEP 4: snake refinement (skeleton attraction, length pres, rigidity)
        // For Clean frames the skeleton-graph Dijkstra path IS the medial
        // axis and snake refinement is largely a no-op; running it anyway
        // gives consistent point spacing. For SelfCrossed frames the
        // centerline already came out of refineSnakeCore via the D-2/D-3
        // wrappers, so we skip the second snake pass to avoid double work.
        if (er.topology == Tracking::TopologyState::Clean) {
            cv::Mat mask; cv::Rect bounds;
            if (buildSnakeMask(blob, mask, bounds)) {
                const cv::Point2f pinH = (er.headIdx >= 0)
                    ? blob.tipCandidates[er.headIdx].point : centerline.front();
                const cv::Point2f pinT = (er.tailIdx >= 0)
                    ? blob.tipCandidates[er.tailIdx].point : centerline.back();
                cv::Point2f tmpOverlap; bool tmpHasOverlap = false;
                refineSnakeCore(blob, mask, bounds, centerline,
                                pinH, pinT, nPts, m_snakeParams,
                                tmpOverlap, tmpHasOverlap);
            }
        }

        // Right-hand-rule orientation veto (replaces legacy [lines 1109-1119]).
        const float curTurning = totalSignedTurningAngle(centerline);
        const float thr = static_cast<float>(
            std::max(0.0, m_snakeParams.orientationAngleThreshold));
        bool flipped = false;
        if (prevState.valid &&
            std::abs(prevState.turningAngle) > thr &&
            std::abs(curTurning) > thr &&
            curTurning * prevState.turningAngle < 0.f) {
            std::reverse(centerline.begin(), centerline.end());
            flipped = true;
            // Also swap the head/tail indices so the assignment matches
            // the new orientation.
            std::swap(blob.assignedHeadTipIdx, blob.assignedTailTipIdx);
        }

        // Persist centerline + overlap marker on the blob.
        blob.centerlinePoints.assign(centerline.begin(), centerline.end());
        if (hasOverlap) {
            blob.centerlineCutPoint = overlapCenter;
            blob.hasCenterlineCutPoint = true;
        } else {
            blob.hasCenterlineCutPoint = false;
        }

        // Single, final write-back (vs legacy's multiple per-frame writes).
        m_storage->setDetectedBlobForFrame(tp.frameNumberOriginal, wormId, blob);

        // ── STEP 5: update predictor for next frame
        const int hIdx2 = blob.assignedHeadTipIdx;
        const int tIdx2 = blob.assignedTailTipIdx;
        if (hIdx2 >= 0) {
            const cv::Point2f newH = blob.tipCandidates[hIdx2].point;
            predictor.velHead = newH - predictor.lastHeadPos;
            predictor.lastHeadPos = newH;
        }
        if (tIdx2 >= 0) {
            const cv::Point2f newT = blob.tipCandidates[tIdx2].point;
            predictor.velTail = newT - predictor.lastTailPos;
            predictor.lastTailPos = newT;
        }
        predictor.hasVelocity = predictor.hasPrev && (hIdx2 >= 0 || tIdx2 >= 0);
        predictor.hasPrev = true;

        // CenterlineState carry for next frame's right-hand-rule check.
        prevState.points       = centerline;
        prevState.blobCentroid = blobCentroid(blob);
        prevState.blob         = blob;
        prevState.valid        = true;
        prevState.turningAngle = flipped ? -curTurning : curTurning;
    }
};
```

**Helper to add at top of `doWorkNew()`** (lambda or local function):
- Map a world `cv::Point2f` to a graph index via the `EndpointResult`'s
  `localBounds` + `skeleton.indexImage`. Used by Clean-branch Dijkstra.

**Validation after Phase 4 — critical, BEFORE flipping the toggle:**

A. Build with `kUseNewPipeline = false`. Confirm app still runs identically
   (sanity check that nothing in legacy was broken by the rename).

B. Flip `kUseNewPipeline = true`. Build. Run on a known-good clip.
   Visually inspect:
   - Green dots present on Clean frames (filled circle). If missing →
     `tipCandidates` not populated or source not set to SkeletonEndpoint.
   - Green dots in approximately the same positions as the legacy run.
     Snapshot the legacy output first so you have a comparison.
   - Cyan ring (head) and red ring (tail) drawn on the right tips.
   - Centerline (white polyline) follows body axis.
   - Orange hollow square appears on D-3 frames (worm coiled, only one
     end visible).

C. If green dots are wrong — **STOP**. Revert to `kUseNewPipeline = false`,
   add a diff-logging block at end of Step 1 that runs both
   `findTipCandidates` and `detectEndpoints` and logs disagreements.
   Re-run, inspect log, fix `detectEndpoints`. Iterate.

D. If centerline is wrong but green dots are right — Step 2 logic issue.
   Bisect by topology (Clean vs SelfCrossed-ring vs SelfCrossed-D2 vs
   SelfCrossed-D3).

---

### Phase 5 — Cut over and remove dead code

**Only after Phase 4 validation passes on real footage.**

1. Delete the `kUseNewPipeline` toggle and the dispatcher; rename
   `doWorkNew()` to `doWork()`.
2. Delete `doWorkLegacy()`.
3. Delete the now-unused helpers in `centerlineworker.cpp`:
   - `processOneFrame` (line 887)
   - `orientByShape` (line 137)
   - `buildSplitRingCandidate` (line 225)
   - `buildSplitRingCandidateWithCut` (line 255)
   - `diffBasedCutHint` (line 302)
   - `refineCenterlineSnake` legacy wrapper (line 725)
   - `inconsistentWithPreviousFrame` (line 157)
   - `continuityScore` (line 186)
4. Remove their helper-of-helpers if they have no other callers
   (`fillBlobMask` is also used by `buildSnakeMask`; KEEP it).

**DO NOT delete from `trackingcommon.{h,cpp}`:**
- `findTipCandidates` — likely still useful as a public utility; if grep
  confirms no remaining callers across the entire `src/` tree, then
  delete. Run `grep -rn "findTipCandidates" src/` to verify.
- `classifyTopology` — same check.
- `assignHeadTail` — same check.
- (The new `detectEndpoints` supersedes all three; if Phase 4 doesn't
  call them anywhere, they're dead code.)

**Validation after Phase 5:**
- `grep -rn "doWorkLegacy\|kUseNewPipeline\|processOneFrame\|orientByShape" src/`
  returns 0 hits.
- Build clean. Run on clip. Identical to end-of-Phase-4 results.

---

### Phase 6 — Documentation pass

1. Update the class-level comment on `CenterlineWorker` in
   [centerlineworker.h:8-22](src/core/centerlineworker.h:8) to describe
   the 2-sweep / 5-step structure.
2. Remove stale references to "Pass 1/2/3/4/5" from any comments that
   still mention them in the new code.
3. Update CLAUDE.md if it mentions the old architecture (verify with
   `grep -n "Pass [0-9]\|processOneFrame\|orientByShape" CLAUDE.md`).

---

## 3. File-management & rollback strategy

The user asked whether the existing `centerlineworker.cpp` should be
deleted to start blank. **Recommendation: NO — keep it intact through
Phase 4.**

Reasons:
1. The old file contains many helpers (`refineSnakeCore`, geodesic
   utilities, `CenterlineState`, snake wrappers, geometry helpers) that
   the new pipeline reuses verbatim. Deleting the file means rewriting
   ~600 lines of correct code from scratch — a fresh source of bugs.
2. Phase 3's "rename old `doWork` to `doWorkLegacy`" pattern lets the
   app keep running through every intermediate commit, so problems can
   be diagnosed against a known-good reference.
3. If the new pipeline regresses, flip `kUseNewPipeline = false` to
   ship the legacy code while diagnosing.

If at the end of Phase 5 the file shrinks substantially, that's fine —
it just means the new code was leaner.

**Git checkpoints** (suggested, not mandatory):
- After Phase 1: `wip(centerline): add SkeletonGraph/TrueTip/EndpointResult structs`
- After Phase 2: `wip(centerline): implement detectEndpoints()`
- After Phase 3: `wip(centerline): scaffold doWorkNew alongside legacy`
- After Phase 4: `wip(centerline): implement 2-sweep pipeline (toggle off)`
- After Phase 4 validation: `feat(centerline): enable new 2-sweep pipeline`
- After Phase 5: `chore(centerline): remove legacy 5-pass code`

---

## 4. Open design questions — DECIDED

Resolutions made before starting implementation. Revisit if the smoke
tests in section 6 surface a problem.

1. **Baseline accumulation timing — DECISION: inline (as planned).**
   Accept that frames near the keyframe see a smaller baseline than
   frames far from it. Curvature-feature-CV gate engages later in the
   run; this is acceptable because head/tail assignment falls back to
   distance alone when the baseline isn't reliable. A pre-sweep can be
   added later if real footage shows mis-assignments on early frames.

2. **Clean-frame snake refinement (Step 4) — DECISION: use
   `refineSnakeCore` as-is.** Tension (α, 1st-derivative penalty)
   couples to point spacing and provides an implicit length preservation
   when combined with pinned endpoints. Adding an explicit
   length-preservation term would couple all-to-all and complicate the
   force balance — defer unless the snake is observed to drift.

3. **Synthetic-hole cut endpoints (Step 2, ring branch) — REVISED
   2026-05-12 after visual smoke test:** the original "prev-nose-to-
   prev-tail" heuristic cuts THROUGH the body on S-shaped worms because
   the hole isn't between the two tips — it sits in the body's
   closure. Re-skeletonization then picks its OWN longest-path
   endpoints, completely ignoring head/tail. **New rule:** D-2 and D-3
   run FIRST whenever head and/or tail are assigned (they pin the
   centerline at those positions and respect the assignment). The
   synthetic-hole-cut path runs only as a last-resort when ZERO tips
   are assigned AND a hole is present — the rare degenerate case of a
   tightly coiled ring with no protruding ends visible. In that
   fallback, cut along the largest hole's major axis (no usable
   prediction available).

4. **`extractCenterlineFromMask` reuse — DECISION: don't cache.**
   Skeletonising twice per worm-frame is acceptable (Sweep 0 once for
   length, Sweep 1 once via `detectEndpoints`). Caching would require
   exposing the function and threading a per-frame map through the
   worker; not worth the complexity until profiling proves it's a bottleneck.

---

## 5. Quick-reference: what each existing helper does + line numbers

| Helper | File | Line | Status in new design |
|---|---|---|---|
| `ptDist`, `arcLen` | centerlineworker.cpp | 14, 20 | KEEP |
| `nearestContourIdx`, `nearestContourPoint` | centerlineworker.cpp | 28, 42 | KEEP |
| `nearestHolePoint` | centerlineworker.cpp | 50 | KEEP (used by ring cut) |
| `resample` | centerlineworker.cpp | 71 | KEEP |
| `CenterlineState` | centerlineworker.cpp | 99 | KEEP (right-hand-rule) |
| `totalSignedTurningAngle` | centerlineworker.cpp | 120 | KEEP |
| `orientByShape` | centerlineworker.cpp | 137 | DELETE (Phase 5) |
| `blobCentroid` | centerlineworker.cpp | 151 | KEEP |
| `inconsistentWithPreviousFrame` | centerlineworker.cpp | 157 | DELETE (Phase 5) |
| `continuityScore` | centerlineworker.cpp | 186 | DELETE (Phase 5) |
| `buildSplitRingCandidate*` | centerlineworker.cpp | 225, 255 | DELETE (Phase 5) |
| `fillBlobMask` | centerlineworker.cpp | 276 | KEEP (used by buildSnakeMask) |
| `diffBasedCutHint` | centerlineworker.cpp | 302 | DELETE (Phase 5) |
| `runDijkstraInMask`, `reconstructPath`, `geodesicPathInMask` | centerlineworker.cpp | 477, 522, 547 | KEEP |
| `refineSnakeCore` | centerlineworker.cpp | 587 | KEEP |
| `buildSnakeMask` | centerlineworker.cpp | 705 | KEEP |
| `refineCenterlineSnake` (legacy) | centerlineworker.cpp | 725 | DELETE (Phase 5) |
| `refineCenterlineSnakeFromTips` | centerlineworker.cpp | 767 | KEEP |
| `refineCenterlineSnakeFromOneTip` | centerlineworker.cpp | 819 | KEEP |
| `processOneFrame` | centerlineworker.cpp | 887 | DELETE (Phase 5) |
| `skeletonizeBinaryMask` (anon ns) | trackingcommon.cpp | 74 | REUSE in detectEndpoints |
| `dijkstraSkeleton` (anon ns) | trackingcommon.cpp | 107 | REUSE in detectEndpoints + Step 2 Clean branch |
| `reconstructCenterlinePath` (anon ns) | trackingcommon.cpp | 149 | REUSE in Step 2 Clean branch |
| `buildCenterlineMask` (anon ns) | trackingcommon.cpp | 181 | REUSE in detectEndpoints |
| `extractCenterlineFromMask` (anon ns) | trackingcommon.cpp | 220 | NOT directly used by new code; still called via populateCenterlineFromContour for D-4 fallback |
| `findTipCandidates` (public) | trackingcommon.cpp | 353 | SUPERSEDED — delete in Phase 5 if grep confirms no callers |
| `classifyTopology` (public) | trackingcommon.cpp | 601 | SUPERSEDED — delete in Phase 5 if grep confirms no callers |
| `assignHeadTail` (public) | trackingcommon.cpp | 640 | SUPERSEDED — delete in Phase 5 if grep confirms no callers |
| `populateCenterlineFromContour` (public) | trackingcommon.cpp | 920 | KEEP (Sweep 0 + D-4 fallback + external callers in trackingmanager) |
| `populateCenterlineFromContourWithCut` (public) | trackingcommon.cpp | 939 | KEEP (Step 2 ring branch) |

---

## 6. Smoke-test checklist (run after every code-changing phase)

- [ ] Build succeeds: `cd build && cmake --build . -j` (or project's build cmd).
- [ ] App launches without crash.
- [ ] Open a known-good video. Run tracking on at least one worm.
- [ ] Toggle "Tip Candidates" view. Green dots appear on Clean frames
      at expected nose/tail positions.
- [ ] Cyan ring (head) and red ring (tail) appear on the assigned tips.
      They flip consistency-correctly when the worm reverses direction.
- [ ] Toggle "Centerline" view. White polyline runs along the body axis.
- [ ] Step through a sequence of self-crossing frames. The orange hollow
      square appears for D-3 frames (worm coiled, one end visible).
- [ ] No red error logs in the console.

If any of those fail, STOP and diagnose before proceeding. Do not
"clean up later."

---

## 7. Quick session-restart preamble (paste at top of new session)

> I am rewriting `src/core/centerlineworker.cpp` per the plan in
> `CENTERLINE_REWRITE_PLAN.md` at the repo root. Read that file end-to-end
> before doing anything. The previous attempt corrupted the green
> skeleton-endpoint dots in the rendered overlay, so be especially careful
> in Phase 2 (`detectEndpoints` implementation) and Phase 4 Step 1
> (`TrueTip → TipCandidate` conversion with explicit
> `source = SkeletonEndpoint`). Do NOT delete the legacy `doWork()` until
> Phase 5; keep it reachable behind the `kUseNewPipeline` toggle. After
> each phase, run the smoke-test checklist in section 6.
