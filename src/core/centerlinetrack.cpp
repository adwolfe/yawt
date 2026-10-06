#include "centerlinetrack.h"
#include "centerlineroutes.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace Centerline {

namespace {

constexpr float kShapeSigma = 3.f;            // px per frame: mean midline displacement
constexpr float kTipSigma = 3.f;              // px per frame: visible tip displacement
constexpr float kLoopFlipCost = 3.f;          // loop direction reversing between frames
constexpr float kVanishCost = 3.f;            // a visible end becoming hidden
constexpr float kAppearCost = 1.5f;           // a hidden end becoming visible
constexpr float kLoopMinTurning = 0.75f * static_cast<float>(CV_PI);
constexpr int   kMaxOptionsPerFrame = 30;
constexpr float kSkipCost = 6.f;              // leaving one frame out of a bridge sequence
constexpr int   kMaxSkip = 3;                 // consecutive frames a bridge may skip
constexpr float kOptionDedupDistance = 1.f;   // px mean distance treated as the same route
constexpr float kIslandBreakFraction = 0.35f; // mean displacement (of body length) that ends an island
constexpr float kIslandAmbiguity = 1.f;       // px: aligned vs reversed too close to call

std::vector<cv::Point2f> reversedPoints(std::vector<cv::Point2f> pts)
{
    std::reverse(pts.begin(), pts.end());
    return pts;
}

float meanDistance(const std::vector<cv::Point2f>& a, const std::vector<cv::Point2f>& b)
{
    if (a.size() != b.size() || a.empty()) return std::numeric_limits<float>::infinity();
    float total = 0.f;
    for (size_t i = 0; i < a.size(); ++i) total += static_cast<float>(cv::norm(a[i] - b[i]));
    return total / static_cast<float>(a.size());
}

// One way a frame's centerline can run, front = the end continuing the left
// anchor's front.
struct State {
    std::vector<cv::Point2f> pts;
    RouteEndKind frontKind = RouteEndKind::ObservedTip;
    RouteEndKind backKind = RouteEndKind::ObservedTip;
    float unary = 0.f;
    float turning = 0.f;
    int option = -1;         // index into the frame's route options; -1 for a clean centerline
    bool reversed = false;   // relative to the option's (or clean centerline's) stored order
    QString summary;
};

State makeState(std::vector<cv::Point2f> pts, RouteEndKind front, RouteEndKind back,
                float unary, int option, bool reversed, const QString& summary)
{
    State s;
    s.pts = std::move(pts);
    s.frontKind = front;
    s.backKind = back;
    s.unary = unary;
    s.turning = signedTurning(s.pts);
    s.option = option;
    s.reversed = reversed;
    s.summary = summary;
    return s;
}

bool visible(RouteEndKind k) { return k != RouteEndKind::Hidden; }

// Cost of moving from state a to state b across `frames` frames.
float transitionCost(const State& a, const State& b, int frames)
{
    // Allowances grow like a random walk over skipped frames.
    const float k = std::sqrt(static_cast<float>(std::max(1, frames)));
    const float shape = meanDistance(a.pts, b.pts) / (kShapeSigma * k);
    float cost = 0.5f * shape * shape;
    // Each end must move plausibly. A hidden end is only a guess, so it gets a
    // wider allowance; charging it at all keeps a visible tip from silently
    // changing ends when the other end is hidden.
    auto endCost = [&](const cv::Point2f& p, RouteEndKind pk, const cv::Point2f& q, RouteEndKind qk) {
        const int hidden = (visible(pk) ? 0 : 1) + (visible(qk) ? 0 : 1);
        const float sigma = kTipSigma * static_cast<float>(1 + hidden) * k;
        const float d = static_cast<float>(cv::norm(p - q)) / sigma;
        return 0.5f * d * d;
    };
    cost += endCost(a.pts.front(), a.frontKind, b.pts.front(), b.frontKind);
    cost += endCost(a.pts.back(), a.backKind, b.pts.back(), b.backKind);
    // A visible end rarely vanishes, and a hidden one rarely reappears, from one
    // frame to the next; a head/tail swap needs both at once.
    auto visibilityCost = [](RouteEndKind from, RouteEndKind to) {
        if (visible(from) && !visible(to)) return kVanishCost;
        if (!visible(from) && visible(to)) return kAppearCost;
        return 0.f;
    };
    cost += visibilityCost(a.frontKind, b.frontKind) + visibilityCost(a.backKind, b.backKind);
    if (std::abs(a.turning) >= kLoopMinTurning && std::abs(b.turning) >= kLoopMinTurning &&
        a.turning * b.turning < 0.f)
        cost += kLoopFlipCost;
    return cost;
}

enum class FrameKind { Empty, Clean, SelfCrossed };

struct FrameInfo {
    int index = -1;
    int frame = -1;
    FrameKind kind = FrameKind::Empty;
    Tracking::DetectedBlob blob;
    int island = -1;
    // Pass 2, self-crossed frames only.
    EndpointResult er;
    Debug::EndpointDebug epDebug;
    RouteEnumeration routes;
    std::vector<State> states;
};

struct GapSolution {
    std::vector<int> chosen;         // per gap frame: index into its states, -1 = none
    std::vector<float> frameCost;    // unary + incoming transition of the chosen state
    std::vector<int> alternative;    // best path under the other pairing of the right island's ends
    std::vector<float> alternativeCost;
    std::vector<int> skipped;        // gap members the sequence skipped, filled from neighbours
    // Per gap member: extra cost of continuing from the previous chosen state
    // with this frame's chosen route reversed. Small values mark frames where
    // orientation is weakly supported.
    std::vector<float> orientationSupport;
    bool rightReversed = false;
    float costSame = std::numeric_limits<float>::infinity();
    float costReversed = std::numeric_limits<float>::infinity();
    float margin = std::numeric_limits<float>::infinity();
    bool solved = false;
};

// Cheapest state sequence through a gap, anchored on whichever sides exist.
// A frame may be skipped (up to kMaxSkip in a row, kSkipCost each) so that one
// frame with no good route cannot hide a change of orientation; skipped frames
// are filled afterwards from their chosen neighbours.
GapSolution solveGap(const State* left, int leftFrame,
                     const State* right, int rightFrame,
                     const std::vector<FrameInfo*>& frames)
{
    GapSolution sol;
    sol.chosen.assign(frames.size(), -1);
    sol.frameCost.assign(frames.size(), 0.f);
    std::vector<int> active;
    for (int j = 0; j < static_cast<int>(frames.size()); ++j)
        if (!frames[j]->states.empty()) active.push_back(j);

    State rightRev;
    if (right) rightRev = makeState(reversedPoints(right->pts), right->backKind, right->frontKind,
                                    0.f, -1, true, QString());
    const float inf = std::numeric_limits<float>::infinity();
    const int m = static_cast<int>(active.size());

    // cost[a][s]: best path ending in state s of active frame a (a = m is the
    // right anchor, states 0 = same, 1 = reversed). back = (frame, state).
    std::vector<std::vector<float>> cost(m + 1);
    std::vector<std::vector<std::pair<int, int>>> back(m + 1);
    auto statesAt = [&](int a) -> int { return a == m ? (right ? 2 : 0) : static_cast<int>(frames[active[a]]->states.size()); };
    auto stateAt = [&](int a, int s) -> const State& {
        if (a == m) return s == 0 ? *right : rightRev;
        return frames[active[a]]->states[s];
    };
    auto frameAt = [&](int a) { return a == m ? rightFrame : frames[active[a]]->frame; };
    for (int a = 0; a <= m; ++a) {
        const int ns = statesAt(a);
        cost[a].assign(ns, inf);
        back[a].assign(ns, {-2, -1});
        for (int s = 0; s < ns; ++s) {
            const State& st = stateAt(a, s);
            // From the left anchor (or a free start), skipping the first a frames.
            if (a <= kMaxSkip) {
                const float skip = static_cast<float>(a) * kSkipCost;
                const float c = st.unary + skip + (left ? transitionCost(*left, st, frameAt(a) - leftFrame) : 0.f);
                if ((left || a == 0 || a == m) && c < cost[a][s]) { cost[a][s] = c; back[a][s] = {-1, -1}; }
            }
            for (int d = 1; d <= kMaxSkip + 1 && a - d >= 0; ++d) {
                const int p = a - d;
                for (int q = 0; q < statesAt(p); ++q) {
                    if (!std::isfinite(cost[p][q])) continue;
                    const float c = cost[p][q] + transitionCost(stateAt(p, q), st, frameAt(a) - frameAt(p)) +
                                    st.unary + static_cast<float>(d - 1) * kSkipCost;
                    if (c < cost[a][s]) { cost[a][s] = c; back[a][s] = {p, q}; }
                }
            }
        }
    }

    auto backtrack = [&](int endFrame, int endState, std::vector<int>& chosen, std::vector<float>& frameCost) {
        chosen.assign(frames.size(), -1);
        frameCost.assign(frames.size(), 0.f);
        int a = endFrame, s = endState;
        while (a >= 0 && s >= 0) {
            const auto [p, q] = back[a][s];
            if (a < m) {
                const State& st = stateAt(a, s);
                const float incoming = p >= 0 ? transitionCost(stateAt(p, q), st, frameAt(a) - frameAt(p))
                                              : (left ? transitionCost(*left, st, frameAt(a) - leftFrame) : 0.f);
                chosen[active[a]] = s;
                frameCost[active[a]] = st.unary + incoming;
            }
            a = p;
            s = q;
        }
    };

    if (right) {
        if (m == 0 && !left) return sol;
        sol.costSame = cost[m][0];
        sol.costReversed = cost[m][1];
        sol.rightReversed = sol.costReversed < sol.costSame;
        sol.margin = std::abs(sol.costSame - sol.costReversed);
        backtrack(m, sol.rightReversed ? 1 : 0, sol.chosen, sol.frameCost);
        backtrack(m, sol.rightReversed ? 0 : 1, sol.alternative, sol.alternativeCost);
    } else {
        // No right anchor: best end state among the last kMaxSkip+1 frames.
        int bestA = -1, bestS = -1;
        for (int a = std::max(0, m - kMaxSkip - 1); a < m; ++a)
            for (int s = 0; s < statesAt(a); ++s) {
                const float c = cost[a][s] + static_cast<float>(m - 1 - a) * kSkipCost;
                if (c < sol.costSame) { sol.costSame = c; bestA = a; bestS = s; }
            }
        if (bestA < 0) return sol;
        backtrack(bestA, bestS, sol.chosen, sol.frameCost);
    }

    // Fill skipped frames with the state that best fits their chosen neighbours.
    for (int a = 0; a < m; ++a) {
        const int j = active[a];
        if (sol.chosen[j] >= 0) continue;
        const State* prev = left;
        int prevFrame = leftFrame;
        for (int b = a - 1; b >= 0; --b)
            if (sol.chosen[active[b]] >= 0) { prev = &stateAt(b, sol.chosen[active[b]]); prevFrame = frameAt(b); break; }
        const State* next = right && !sol.rightReversed ? right : (right ? &rightRev : nullptr);
        int nextFrame = rightFrame;
        for (int b = a + 1; b < m; ++b)
            if (sol.chosen[active[b]] >= 0) { next = &stateAt(b, sol.chosen[active[b]]); nextFrame = frameAt(b); break; }
        float best = inf;
        for (int s = 0; s < statesAt(a); ++s) {
            const State& st = stateAt(a, s);
            const float c = st.unary + (prev ? transitionCost(*prev, st, frameAt(a) - prevFrame) : 0.f) +
                            (next ? transitionCost(st, *next, nextFrame - frameAt(a)) : 0.f);
            if (c < best) { best = c; sol.chosen[j] = s; sol.frameCost[j] = c; }
        }
        sol.skipped.push_back(j);
    }
    sol.orientationSupport.assign(frames.size(), 0.f);
    const State* prev = left;
    int prevFrame = leftFrame;
    for (int a = 0; a < m; ++a) {
        const int j = active[a];
        const State& st = stateAt(a, sol.chosen[j]);
        if (prev) {
            State flipped = makeState(reversedPoints(st.pts), st.backKind, st.frontKind, st.unary, st.option,
                                      !st.reversed, QString());
            sol.orientationSupport[j] = transitionCost(*prev, flipped, frameAt(a) - prevFrame) -
                                        transitionCost(*prev, st, frameAt(a) - prevFrame);
        }
        prev = &st;
        prevFrame = frameAt(a);
    }
    sol.solved = true;
    return sol;
}

void reverseCenterline(Tracking::BlobCenterline& cl)
{
    std::reverse(cl.points.begin(), cl.points.end());
    std::swap(cl.headTipIdx, cl.tailTipIdx);
}

} // namespace

void reverseStoredFrame(CenterlineFrameIo& io, int wormId, int frameNumber, const QString& reason)
{
    const QMap<int, Tracking::DetectedBlob> blobs = io.getDetectedBlobsForFrame(frameNumber);
    if (!blobs.contains(wormId)) return;
    Tracking::DetectedBlob blob = blobs[wormId];
    reverseCenterline(blob.centerline);
    io.setDetectedBlobForFrame(frameNumber, wormId, blob);
    Debug::CenterlineFrameDebug record;
    if (io.getCenterlineDebugFrame && io.setCenterlineDebugFrame &&
        io.getCenterlineDebugFrame(wormId, frameNumber, record)) {
        for (auto* line : {&record.initialCenterline, &record.resampledCenterline, &record.finalCenterline})
            std::reverse(line->begin(), line->end());
        std::swap(record.assignedHeadTipIdx, record.assignedTailTipIdx);
        record.finalTurningAngle = -record.finalTurningAngle;
        for (QString& role : record.tipCapRoles) {
            if (role == QStringLiteral("head")) role = QStringLiteral("tail");
            else if (role == QStringLiteral("tail")) role = QStringLiteral("head");
        }
        record.decisions << QStringLiteral("reversed: %1").arg(reason);
        io.setCenterlineDebugFrame(record);
    }
}

void flagStoredFrame(CenterlineFrameIo& io, int wormId, int frameNumber, const QString& reason)
{
    const QMap<int, Tracking::DetectedBlob> blobs = io.getDetectedBlobsForFrame(frameNumber);
    if (!blobs.contains(wormId)) return;
    Tracking::DetectedBlob blob = blobs[wormId];
    blob.centerline.needsReview = true;
    blob.centerline.reviewReason = blob.centerline.reviewReason.isEmpty()
        ? reason : blob.centerline.reviewReason + QStringLiteral("; ") + reason;
    io.setDetectedBlobForFrame(frameNumber, wormId, blob);
    Debug::CenterlineFrameDebug record;
    if (io.getCenterlineDebugFrame && io.setCenterlineDebugFrame &&
        io.getCenterlineDebugFrame(wormId, frameNumber, record)) {
        record.decisions << QStringLiteral("REVIEW: %1").arg(reason);
        io.setCenterlineDebugFrame(record);
    }
}

QList<int> propagateAcrossWeakLinks(CenterlineFrameIo& io, int wormId,
                                    const TrackPassResult& continuity,
                                    std::vector<bool>& decided,
                                    std::vector<bool>& flipped)
{
    QList<int> frames;
    bool changed = true;
    while (changed) {
        changed = false;
        for (const ChainLink& link : continuity.weakLinks) {
            const int l = link.leftChain, r = link.rightChain;
            if (l < 0 || r < 0 || decided[l] == decided[r]) continue;
            const int from = decided[l] ? l : r, to = decided[l] ? r : l;
            const bool wantFlip = flipped[from] != link.reversed;
            if (flipped[to] != wantFlip) {
                for (int f : continuity.chains[to]) {
                    reverseStoredFrame(io, wormId, f,
                        QStringLiteral("continuity: follows neighbouring chain across a weak bridge (margin %1)")
                            .arg(link.margin, 0, 'f', 1));
                    frames.append(f);
                }
                flipped[to] = wantFlip;
            }
            decided[to] = true;
            changed = true;
        }
    }
    return frames;
}

TrackPassResult processTrackContinuity(const CenterlineFrameContext& ctx,
                                       CenterlineFrameIo& io,
                                       const TrackPassConfig& config)
{
    TrackPassResult result;
    if (!ctx.sortedPoints) return result;
    const Tracking::Track& points = *ctx.sortedPoints;
    const int n = static_cast<int>(points.size());
    const TipFeatureBaseline baseline = io.getTipBaseline(ctx.wormId);
    auto blobFor = [&](int frame, Tracking::DetectedBlob& out) {
        const QMap<int, Tracking::DetectedBlob> blobs = io.getDetectedBlobsForFrame(frame);
        if (!blobs.contains(ctx.wormId)) return false;
        out = blobs[ctx.wormId];
        return out.isValid && !out.contourPoints.empty();
    };

    // ── Pass 1: clean frames, each on its own ────────────────────────────────
    std::vector<FrameInfo> frames(n);
    for (int i = 0; i < n; ++i) {
        FrameInfo& fi = frames[i];
        fi.index = i;
        fi.frame = points[i].frameNumber;
        if (points[i].quality == Tracking::TrackPointQuality::Lost) continue;
        CenterlineSweepState state;
        CenterlineFrameRequest req{i, 1, true};
        req.skipIfMerged = config.skipMergedFrames;
        const CenterlineFrameResult r = processFrame(ctx, req, state, io);
        if (!r.processed || points[i].quality == Tracking::TrackPointQuality::Merged) continue;
        fi.blob = r.blob;
        const auto& cl = fi.blob.centerline;
        if (cl.topology == Tracking::TopologyState::Clean && cl.points.size() >= 2)
            fi.kind = FrameKind::Clean;
        else if (cl.topology == Tracking::TopologyState::SelfCrossed)
            fi.kind = FrameKind::SelfCrossed;
    }

    const float fallbackLength = baseline.lengthSamples >= 30 ? baseline.meanBodyLength
                                                              : (ctx.refLength > 0.f ? ctx.refLength : 40.f);
    auto sampled = [&](const Tracking::BlobCenterline& cl) {
        return resamplePolyline(std::vector<cv::Point2f>(cl.points.begin(), cl.points.end()), ctx.nPts);
    };

    // Link consecutive clean frames into islands, aligning each to the last.
    std::vector<std::vector<int>> islands;
    for (int i = 0; i < n; ++i) {
        FrameInfo& fi = frames[i];
        if (fi.kind != FrameKind::Clean) continue;
        bool extend = false;
        if (i > 0 && frames[i - 1].kind == FrameKind::Clean && frames[i - 1].frame == fi.frame - 1) {
            const auto prev = sampled(frames[i - 1].blob.centerline);
            const auto cur = sampled(fi.blob.centerline);
            const float dSame = meanDistance(prev, cur);
            const float dRev = meanDistance(prev, reversedPoints(cur));
            extend = std::min(dSame, dRev) <= kIslandBreakFraction * fallbackLength &&
                     std::abs(dSame - dRev) >= kIslandAmbiguity;
            if (extend && dRev < dSame) {
                reverseCenterline(fi.blob.centerline);
                reverseStoredFrame(io, ctx.wormId, fi.frame,
                                   QStringLiteral("aligned with previous clean frame"));
            }
        }
        if (!extend) islands.emplace_back();
        islands.back().push_back(i);
        fi.island = static_cast<int>(islands.size()) - 1;
    }

    std::vector<int> anchors;
    for (int k = 0; k < static_cast<int>(islands.size()); ++k)
        if (static_cast<int>(islands[k].size()) >= config.minIslandFrames) anchors.push_back(k);
    if (anchors.empty() && !islands.empty()) {
        int longest = 0;
        for (int k = 1; k < static_cast<int>(islands.size()); ++k)
            if (islands[k].size() > islands[longest].size()) longest = k;
        anchors.push_back(longest);
    }
    std::vector<char> isAnchorIsland(islands.size(), 0);
    for (int k : anchors) isAnchorIsland[k] = 1;
    auto islandLength = [&](int k) {
        std::vector<float> lengths;
        for (int idx : islands[k])
            lengths.push_back(resampledArcLength(std::vector<cv::Point2f>(
                frames[idx].blob.centerline.points.begin(), frames[idx].blob.centerline.points.end()), ctx.nPts));
        std::nth_element(lengths.begin(), lengths.begin() + lengths.size() / 2, lengths.end());
        return lengths[lengths.size() / 2];
    };
    auto anchorState = [&](int idx) {
        return makeState(sampled(frames[idx].blob.centerline), RouteEndKind::ObservedTip,
                         RouteEndKind::ObservedTip, 0.f, -1, false, QStringLiteral("anchor"));
    };
    result.log << QStringLiteral("worm %1: %2 frames, %3 clean islands, %4 anchors (min %5 frames)")
                      .arg(ctx.wormId).arg(n).arg(islands.size()).arg(anchors.size())
                      .arg(config.minIslandFrames);

    // ── Pass 2: bridge every gap between anchors ─────────────────────────────
    // Gaps are the frames outside anchor islands, bounded by the anchors (or
    // the track ends) on either side.
    struct Gap { int leftAnchor = -1; int rightAnchor = -1; std::vector<int> members; GapSolution sol; float length = 0.f; };
    std::vector<Gap> gaps;
    {
        Gap current;
        int lastAnchor = -1;
        for (int i = 0; i < n; ++i) {
            const int isl = frames[i].island;
            const bool inAnchor = isl >= 0 && isAnchorIsland[isl];
            if (inAnchor) {
                if (!current.members.empty()) {
                    current.leftAnchor = lastAnchor;
                    current.rightAnchor = isl;
                    gaps.push_back(current);
                    current = Gap{};
                }
                lastAnchor = isl;
            } else {
                current.members.push_back(i);
            }
        }
        if (!current.members.empty()) {
            current.leftAnchor = lastAnchor;
            current.rightAnchor = -1;
            gaps.push_back(current);
        }
        // Adjacent anchors with no frames between them still need a link.
        for (size_t a = 0; a + 1 < anchors.size(); ++a) {
            const int l = anchors[a], r = anchors[a + 1];
            bool hasGap = false;
            for (const Gap& g : gaps) hasGap |= g.leftAnchor == l && g.rightAnchor == r;
            if (!hasGap) { Gap g; g.leftAnchor = l; g.rightAnchor = r; gaps.push_back(g); }
        }
        std::sort(gaps.begin(), gaps.end(), [&](const Gap& a, const Gap& b) {
            const int fa = a.members.empty() ? islands[a.rightAnchor].front() : a.members.front();
            const int fb = b.members.empty() ? islands[b.rightAnchor].front() : b.members.front();
            return fa < fb;
        });
    }

    for (Gap& gap : gaps) {
        std::vector<float> ends;
        if (gap.leftAnchor >= 0) ends.push_back(islandLength(gap.leftAnchor));
        if (gap.rightAnchor >= 0) ends.push_back(islandLength(gap.rightAnchor));
        gap.length = ends.empty() ? fallbackLength
                                  : std::accumulate(ends.begin(), ends.end(), 0.f) / static_cast<float>(ends.size());
        std::vector<cv::Point2f> hints;
        State leftState, rightState;
        int leftFrame = -1, rightFrame = -1;
        if (gap.leftAnchor >= 0) {
            const int idx = islands[gap.leftAnchor].back();
            leftState = anchorState(idx);
            leftFrame = frames[idx].frame;
            hints.push_back(leftState.pts.front());
            hints.push_back(leftState.pts.back());
        }
        if (gap.rightAnchor >= 0) {
            const int idx = islands[gap.rightAnchor].front();
            rightState = anchorState(idx);
            rightFrame = frames[idx].frame;
            hints.push_back(rightState.pts.front());
            hints.push_back(rightState.pts.back());
        }

        std::vector<FrameInfo*> members;
        for (int idx : gap.members) {
            FrameInfo& fi = frames[idx];
            members.push_back(&fi);
            if (fi.kind == FrameKind::Clean) {
                const auto pts = sampled(fi.blob.centerline);
                fi.states.push_back(makeState(pts, RouteEndKind::ObservedTip, RouteEndKind::ObservedTip,
                                              0.f, -1, false, QStringLiteral("short clean run")));
                fi.states.push_back(makeState(reversedPoints(pts), RouteEndKind::ObservedTip,
                                              RouteEndKind::ObservedTip, 0.f, -1, true,
                                              QStringLiteral("short clean run reversed")));
                continue;
            }
            if (fi.kind != FrameKind::SelfCrossed) continue;
            Tracking::DetectedBlob blob;
            if (!blobFor(fi.frame, blob)) continue;
            fi.er = detectEndpoints(blob, HeadTailPredictor{}, baseline, false,
                                    ctx.captureDebug ? &fi.epDebug : nullptr);
            RouteSelectionInput in;
            in.graph = &fi.er.skeleton;
            in.distTransform = fi.er.distTransform;
            in.localBounds = fi.er.localBounds;
            for (size_t t = 0; t < fi.er.tips.size() && t < fi.er.skeleton.endpointIndices.size(); ++t)
                in.observedTips.push_back({fi.er.skeleton.endpointIndices[t], fi.er.tips[t].point});
            in.hintPoints = hints;
            in.bodyLength = gap.length;
            in.nPoints = ctx.nPts;
            fi.routes = enumerateSelfCrossedRoutes(in);
            // Keep the cheapest distinct routes.
            std::vector<int> order(fi.routes.options.size());
            std::iota(order.begin(), order.end(), 0);
            auto unary = [&](const RouteOption& o) {
                return o.lengthCost + o.junctionCost + routeEndCost(o.startKind) + routeEndCost(o.endKind) +
                       (o.retrace ? routeRetraceCost() : 0.f);
            };
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                return unary(fi.routes.options[a]) < unary(fi.routes.options[b]);
            });
            std::vector<int> kept;
            for (int o : order) {
                const auto& pts = fi.routes.options[o].points;
                bool duplicate = false;
                for (int k : kept) {
                    const auto& other = fi.routes.options[k].points;
                    duplicate |= meanDistance(pts, other) < kOptionDedupDistance ||
                                 meanDistance(pts, reversedPoints(other)) < kOptionDedupDistance;
                }
                if (duplicate) continue;
                kept.push_back(o);
                if (static_cast<int>(kept.size()) >= kMaxOptionsPerFrame) break;
            }
            for (int o : kept) {
                const RouteOption& opt = fi.routes.options[o];
                const QString summary = QStringLiteral("route %1 len=%2 ends=%3/%4 unary=%5")
                    .arg(opt.path).arg(opt.length, 0, 'f', 1)
                    .arg(routeEndKindName(opt.startKind), routeEndKindName(opt.endKind))
                    .arg(unary(opt), 0, 'f', 2);
                fi.states.push_back(makeState(opt.points, opt.startKind, opt.endKind, unary(opt), o, false, summary));
                fi.states.push_back(makeState(reversedPoints(opt.points), opt.endKind, opt.startKind,
                                              unary(opt), o, true, summary + QStringLiteral(" reversed")));
            }
        }
        gap.sol = solveGap(gap.leftAnchor >= 0 ? &leftState : nullptr, leftFrame,
                           gap.rightAnchor >= 0 ? &rightState : nullptr, rightFrame, members);
        result.log << QStringLiteral("gap frames %1-%2 between islands %3 and %4: L=%5 same=%6 reversed=%7 margin=%8%9")
            .arg(gap.members.empty() ? -1 : frames[gap.members.front()].frame)
            .arg(gap.members.empty() ? -1 : frames[gap.members.back()].frame)
            .arg(gap.leftAnchor).arg(gap.rightAnchor)
            .arg(gap.length, 0, 'f', 1)
            .arg(gap.sol.costSame, 0, 'f', 2).arg(gap.sol.costReversed, 0, 'f', 2)
            .arg(gap.sol.margin, 0, 'f', 2)
            .arg(gap.sol.rightReversed ? QStringLiteral(" right island reversed") : QString());
    }

    // Orient anchors along each chain and break chains at untrusted bridges.
    std::vector<int> anchorSign(islands.size(), 1), anchorChain(islands.size(), -1);
    int chainCount = 0;
    for (size_t a = 0; a < anchors.size(); ++a) {
        const int k = anchors[a];
        if (a == 0) { anchorChain[k] = chainCount++; continue; }
        const int prev = anchors[a - 1];
        const Gap* link = nullptr;
        for (const Gap& g : gaps)
            if (g.leftAnchor == prev && g.rightAnchor == k) link = &g;
        if (link && link->sol.solved && link->sol.margin >= config.reviewMargin) {
            anchorChain[k] = anchorChain[prev];
            anchorSign[k] = anchorSign[prev] * (link->sol.rightReversed ? -1 : 1);
        } else {
            anchorChain[k] = chainCount++;
        }
    }
    for (int k : anchors) {
        if (anchorSign[k] > 0) continue;
        for (int idx : islands[k]) {
            reverseCenterline(frames[idx].blob.centerline);
            reverseStoredFrame(io, ctx.wormId, frames[idx].frame,
                               QStringLiteral("island oriented to match its chain across a contact bridge"));
        }
    }

    std::vector<std::pair<int, int>> weakBridges;   // (left anchor, right anchor) below margin
    for (size_t a = 1; a < anchors.size(); ++a)
        if (anchorChain[anchors[a]] != anchorChain[anchors[a - 1]])
            weakBridges.push_back({anchors[a - 1], anchors[a]});
    for (const auto& [l, r] : weakBridges) {
        ChainLink link;
        link.leftChain = anchorChain[l];
        link.rightChain = anchorChain[r];
        for (const Gap& g : gaps)
            if (g.leftAnchor == l && g.rightAnchor == r && g.sol.solved) {
                // The left chain may itself have been reoriented; the guess is
                // relative to how each chain was finally written.
                link.reversed = g.sol.rightReversed != ((anchorSign[l] < 0) != (anchorSign[r] < 0));
                link.margin = g.sol.margin;
            }
        result.weakLinks.push_back(link);
    }

    std::vector<int> chainOf(n, -1);
    for (int k : anchors)
        for (int idx : islands[k]) chainOf[idx] = anchorChain[k];

    // Write the bridged frames, oriented like the anchor they were solved against.
    for (const Gap& gap : gaps) {
        const int ref = gap.leftAnchor >= 0 ? gap.leftAnchor : gap.rightAnchor;
        const int sign = ref >= 0 ? anchorSign[ref] : 1;
        const int chain = ref >= 0 ? anchorChain[ref] : chainCount;
        const bool trustedRight = gap.rightAnchor < 0 || gap.leftAnchor < 0 ||
                                  (gap.sol.solved && gap.sol.margin >= config.reviewMargin);
        for (size_t j = 0; j < gap.members.size(); ++j) {
            FrameInfo& fi = frames[gap.members[j]];
            chainOf[fi.index] = chain;
            const int s = gap.sol.chosen.empty() ? -1 : gap.sol.chosen[j];
            if (fi.kind == FrameKind::Clean) {
                if (s >= 0 && (fi.states[s].reversed != (sign < 0))) {
                    reverseCenterline(fi.blob.centerline);
                    reverseStoredFrame(io, ctx.wormId, fi.frame,
                                       QStringLiteral("short clean run oriented by contact bridge"));
                }
            } else if (fi.kind == FrameKind::SelfCrossed) {
                Tracking::DetectedBlob blob;
                if (!blobFor(fi.frame, blob)) continue;
                auto& cl = blob.centerline;
                cl.points.clear();
                cl.tipCandidates.clear();
                cl.hasCutPoint = false;
                cl.topology = Tracking::TopologyState::SelfCrossed;
                for (const TrueTip& t : fi.er.tips) {
                    Tracking::TipCandidate tc;
                    tc.point = t.point;
                    tc.curvature = t.curvature;
                    tc.width = t.width;
                    tc.source = t.extended ? Tracking::TipCandidate::Source::CurvaturePeak
                                           : Tracking::TipCandidate::Source::SkeletonEndpoint;
                    cl.tipCandidates.push_back(tc);
                }
                cl.headTipIdx = -1;
                cl.tailTipIdx = -1;
                Debug::CenterlineFrameDebug rec;
                rec.wormId = ctx.wormId;
                rec.frameNumber = fi.frame;
                rec.sweepStep = 0;
                rec.topology = Tracking::TopologyState::SelfCrossed;
                if (ctx.captureDebug) captureEndpointDiagnostics(fi.er, fi.epDebug, rec);
                rec.decisions << QStringLiteral("contact bridge: gap frames %1-%2, left island %3, right island %4, L=%5")
                                     .arg(frames[gap.members.front()].frame).arg(frames[gap.members.back()].frame)
                                     .arg(gap.leftAnchor).arg(gap.rightAnchor).arg(gap.length, 0, 'f', 1);
                rec.decisions << QStringLiteral("contact bridge: cost same=%1 reversed=%2 margin=%3%4")
                                     .arg(gap.sol.costSame, 0, 'f', 2).arg(gap.sol.costReversed, 0, 'f', 2)
                                     .arg(gap.sol.margin, 0, 'f', 2)
                                     .arg(trustedRight ? QString() : QStringLiteral(" (below review margin; chain breaks)"));
                rec.decisions << fi.routes.decisions;
                rec.decisions << QStringLiteral("contact bridge: %1 routes in window, %2 kept, %3 rejected by length (L=%4)")
                                     .arg(fi.routes.options.size()).arg(fi.states.size() / 2)
                                     .arg(fi.routes.rejectedByLength).arg(gap.length, 0, 'f', 1);
                if (s < 0) {
                    rec.branch = Debug::CenterlineBranch::SelfCrossedUnresolved;
                    rec.decisions << QStringLiteral("contact bridge: no route within the body-length window; frame unresolved");
                } else {
                    State st = fi.states[s];
                    if (sign < 0) {
                        st.pts = reversedPoints(st.pts);
                        std::swap(st.frontKind, st.backKind);
                        st.turning = -st.turning;
                    }
                    cl.points.assign(st.pts.begin(), st.pts.end());
                    auto registerEnd = [&](const cv::Point2f& p, RouteEndKind kind) {
                        if (kind != RouteEndKind::Hidden) {
                            for (int t = 0; t < static_cast<int>(cl.tipCandidates.size()); ++t)
                                if (cv::norm(cl.tipCandidates[t].point - p) < 0.5) return t;
                        }
                        Tracking::TipCandidate tc;
                        tc.point = p;
                        tc.source = kind == RouteEndKind::Hidden
                            ? Tracking::TipCandidate::Source::HypothesizedHidden
                            : Tracking::TipCandidate::Source::SkeletonEndpoint;
                        cl.tipCandidates.push_back(tc);
                        if (kind == RouteEndKind::Hidden) {
                            rec.hiddenTipHypothesized = true;
                            rec.hiddenTipFinal = p;
                        }
                        return static_cast<int>(cl.tipCandidates.size()) - 1;
                    };
                    cl.headTipIdx = registerEnd(st.pts.front(), st.frontKind);
                    cl.tailTipIdx = registerEnd(st.pts.back(), st.backKind);
                    rec.branch = Debug::CenterlineBranch::ContactBridge;
                    rec.decisions << QStringLiteral("contact bridge: chose %1; frame cost %2 (unary %3)")
                                         .arg(st.summary).arg(gap.sol.frameCost[j], 0, 'f', 2).arg(st.unary, 0, 'f', 2);
                    rec.decisions << QStringLiteral("contact bridge: orientation support from previous frame %1")
                                         .arg(gap.sol.orientationSupport[j], 0, 'f', 2);
                    if (!gap.sol.alternative.empty() && gap.sol.alternative[j] >= 0)
                        rec.decisions << QStringLiteral("contact bridge: other pairing would choose %1; frame cost %2")
                                             .arg(fi.states[gap.sol.alternative[j]].summary)
                                             .arg(gap.sol.alternativeCost[j], 0, 'f', 2);
                    rec.initialCenterline = st.pts;
                    rec.resampledCenterline = st.pts;
                    rec.finalCenterline = st.pts;
                    rec.initialArcLength = rec.finalArcLength = resampledArcLength(st.pts, ctx.nPts);
                    rec.finalTurningAngle = st.turning;
                    rec.d3RouteDebugAvailable = true;
                    rec.d3RouteStartIsHead = true;
                    rec.d3RouteStart = st.pts.front();
                    rec.d3RouteEnd = st.pts.back();
                    rec.d3RouteJunction = cv::Point2f(-1.f, -1.f);
                    rec.d3RouteCenter = cv::Point2f(-1.f, -1.f);
                    rec.d3CandidatePaths.push_back(st.pts);
                    for (size_t o = 0; o < fi.states.size() && rec.d3CandidatePaths.size() < 4; o += 2)
                        if (fi.states[o].option != st.option) rec.d3CandidatePaths.push_back(fi.states[o].pts);
                    rec.d3SelectedCandidate = 0;
                    if (gap.sol.frameCost[j] > config.frameResidualReview)
                        result.review[fi.frame] = QStringLiteral("bridge frame cost %1 above %2")
                            .arg(gap.sol.frameCost[j], 0, 'f', 1).arg(config.frameResidualReview, 0, 'f', 1);
                }
                if (std::find(gap.sol.skipped.begin(), gap.sol.skipped.end(), static_cast<int>(j)) != gap.sol.skipped.end()) {
                    rec.decisions << QStringLiteral("contact bridge: frame skipped by the sequence; filled from its neighbours");
                    result.review[fi.frame] = QStringLiteral("bridge skipped this frame (no route fits its neighbours)");
                }
                if (s < 0) result.review[fi.frame] = QStringLiteral("no route within the body-length window");
                if (!trustedRight && !result.review.count(fi.frame))
                    result.review[fi.frame] = QStringLiteral("contact bridge margin %1 below %2")
                        .arg(gap.sol.margin, 0, 'f', 1).arg(config.reviewMargin, 0, 'f', 1);
                rec.tipCandidates = cl.tipCandidates;
                rec.assignedHeadTipIdx = cl.headTipIdx;
                rec.assignedTailTipIdx = cl.tailTipIdx;
                io.setDetectedBlobForFrame(fi.frame, ctx.wormId, blob);
                if (ctx.captureDebug && io.setCenterlineDebugFrame) io.setCenterlineDebugFrame(rec);
            }
        }
    }

    // Chains, in frame order; frames that never joined one (e.g. a track with
    // no clean frames at all) form their own chain.
    std::map<int, std::vector<int>> chains;
    for (int i = 0; i < n; ++i)
        chains[chainOf[i] >= 0 ? chainOf[i] : chainCount].push_back(frames[i].frame);
    for (auto& [id, list] : chains) result.chains.push_back(list);
    result.log << QStringLiteral("worm %1: %2 continuity chains, %3 frames flagged for review")
                      .arg(ctx.wormId).arg(result.chains.size()).arg(result.review.size());
    for (const auto& [frame, reason] : result.review)
        flagStoredFrame(io, ctx.wormId, frame, reason);
    return result;
}

} // namespace Centerline
