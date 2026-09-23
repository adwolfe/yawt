#include "centerlineroutes.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

namespace Centerline {

namespace {

constexpr float kBaseRoleSigma = 3.f;        // px; spread of a prediction observed last frame
constexpr float kRoleSigmaPerFrame = 1.5f;   // px added per frame since last observation
constexpr float kLengthSigmaFraction = 0.08f;
constexpr float kShortBranchCost = 1.5f;
constexpr float kHiddenEndCost = 1.5f;         // an end that was already hidden or is unknown
constexpr float kVanishingEndCost = 4.f;       // an end observed last frame now claimed hidden
constexpr float kRetraceCost = 1.0f;
constexpr float kOrientationMismatchCost = 6.f;
// A route must turn at least this much for its sense of rotation to count;
// the reference needs less, since a C-shaped body entering contact already
// fixes which way the closing loop turns.
constexpr float kOrientationMinTurning = 0.75f * static_cast<float>(CV_PI);
constexpr float kOrientationMinReferenceTurning = 0.5f * static_cast<float>(CV_PI);
constexpr float kJunctionTurnWeight = 1.f;
constexpr float kForcedNodeMinSeparation = 4.f;  // px from an observed endpoint
constexpr int   kMaxCandidates = 4000;
constexpr int   kRankedKept = 8;

float polylineLength(const std::vector<cv::Point2f>& pts)
{
    float total = 0.f;
    for (size_t i = 1; i < pts.size(); ++i) total += cv::norm(pts[i] - pts[i - 1]);
    return total;
}

std::vector<cv::Point2f> resamplePolyline(const std::vector<cv::Point2f>& pts, int n)
{
    if (pts.size() < 2 || n < 2) return pts;
    std::vector<float> cumulative(pts.size(), 0.f);
    for (size_t i = 1; i < pts.size(); ++i)
        cumulative[i] = cumulative[i - 1] + cv::norm(pts[i] - pts[i - 1]);
    const float total = cumulative.back();
    if (total <= 1e-6f) return std::vector<cv::Point2f>(n, pts.front());
    std::vector<cv::Point2f> out;
    out.reserve(n);
    size_t seg = 1;
    for (int k = 0; k < n; ++k) {
        const float target = total * static_cast<float>(k) / static_cast<float>(n - 1);
        while (seg < pts.size() - 1 && cumulative[seg] < target) ++seg;
        const float span = cumulative[seg] - cumulative[seg - 1];
        const float t = span > 1e-6f ? (target - cumulative[seg - 1]) / span : 0.f;
        out.push_back(pts[seg - 1] + (pts[seg] - pts[seg - 1]) * t);
    }
    return out;
}

// Cut a polyline at the given arc length.
std::vector<cv::Point2f> truncatePolyline(const std::vector<cv::Point2f>& pts, float length)
{
    std::vector<cv::Point2f> out;
    if (pts.empty()) return out;
    out.push_back(pts.front());
    float walked = 0.f;
    for (size_t i = 1; i < pts.size(); ++i) {
        const float seg = cv::norm(pts[i] - pts[i - 1]);
        if (walked + seg >= length) {
            const float t = seg > 1e-6f ? (length - walked) / seg : 0.f;
            out.push_back(pts[i - 1] + (pts[i] - pts[i - 1]) * t);
            return out;
        }
        walked += seg;
        out.push_back(pts[i]);
    }
    return out;
}

// Junction-compressed view of the skeleton pixel graph.
struct RouteGraph {
    enum class NodeKind { Endpoint, Junction, Forced };
    struct Node {
        NodeKind kind = NodeKind::Junction;
        int rep = -1;               // representative skeleton pixel
        std::vector<int> pixels;
    };
    struct Edge {
        int a = -1, b = -1;
        std::vector<int> pixels;    // ordered a → b, both node boundary pixels included
    };
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::vector<std::vector<int>> incident;
    std::vector<int> nodeOf;
};

RouteGraph buildRouteGraph(const SkeletonGraph& graph, const std::vector<int>& forcedPixels)
{
    RouteGraph rg;
    const int n = static_cast<int>(graph.points.size());
    rg.nodeOf.assign(n, -1);
    auto degree = [&](int i) { return static_cast<int>(graph.adjacency[i].size()); };

    for (int i = 0; i < n; ++i) {
        if (degree(i) == 1) {
            rg.nodeOf[i] = static_cast<int>(rg.nodes.size());
            rg.nodes.push_back({RouteGraph::NodeKind::Endpoint, i, {i}});
        }
    }
    for (int i = 0; i < n; ++i) {
        if (degree(i) < 3 || rg.nodeOf[i] >= 0) continue;
        const int id = static_cast<int>(rg.nodes.size());
        RouteGraph::Node node;
        std::vector<int> stack{i};
        rg.nodeOf[i] = id;
        while (!stack.empty()) {
            const int cur = stack.back();
            stack.pop_back();
            node.pixels.push_back(cur);
            for (int nb : graph.adjacency[cur]) {
                if (degree(nb) >= 3 && rg.nodeOf[nb] < 0) {
                    rg.nodeOf[nb] = id;
                    stack.push_back(nb);
                }
            }
        }
        cv::Point2f centroid(0.f, 0.f);
        for (int p : node.pixels) centroid += cv::Point2f(graph.points[p]);
        centroid *= 1.f / static_cast<float>(node.pixels.size());
        node.rep = node.pixels.front();
        for (int p : node.pixels)
            if (cv::norm(cv::Point2f(graph.points[p]) - centroid) <
                cv::norm(cv::Point2f(graph.points[node.rep]) - centroid))
                node.rep = p;
        rg.nodes.push_back(node);
    }
    std::vector<int> forced = forcedPixels;
    if (rg.nodes.empty() && forced.empty() && n > 0) forced.push_back(0);  // plain ring
    for (int p : forced) {
        if (p < 0 || p >= n || rg.nodeOf[p] >= 0) continue;
        rg.nodeOf[p] = static_cast<int>(rg.nodes.size());
        rg.nodes.push_back({RouteGraph::NodeKind::Forced, p, {p}});
    }

    rg.incident.assign(rg.nodes.size(), {});
    std::vector<char> chainVisited(n, 0);
    std::vector<std::pair<int, int>> directPairs;
    for (int nodeId = 0; nodeId < static_cast<int>(rg.nodes.size()); ++nodeId) {
        for (int p : rg.nodes[nodeId].pixels) {
            for (int q : graph.adjacency[p]) {
                const int qNode = rg.nodeOf[q];
                if (qNode == nodeId) continue;
                RouteGraph::Edge edge;
                edge.a = nodeId;
                edge.pixels.push_back(p);
                if (qNode >= 0) {
                    const auto key = std::minmax(nodeId, qNode);
                    if (std::find(directPairs.begin(), directPairs.end(), key) != directPairs.end())
                        continue;
                    directPairs.push_back(key);
                    edge.pixels.push_back(q);
                    edge.b = qNode;
                } else {
                    if (chainVisited[q]) continue;
                    int prev = p, cur = q;
                    while (true) {
                        chainVisited[cur] = 1;
                        edge.pixels.push_back(cur);
                        int next = -1;
                        for (int nb : graph.adjacency[cur]) {
                            if (nb == prev) continue;
                            if (rg.nodeOf[nb] >= 0 && nb != p) { next = nb; break; }
                            if (rg.nodeOf[nb] < 0 && !chainVisited[nb]) { next = nb; }
                        }
                        if (next < 0) {
                            // The chain closed on its starting node through a pixel
                            // adjacent to p; finish there.
                            next = p;
                        }
                        if (rg.nodeOf[next] >= 0) {
                            edge.pixels.push_back(next);
                            edge.b = rg.nodeOf[next];
                            break;
                        }
                        prev = cur;
                        cur = next;
                    }
                }
                // Pixel triangles at a junction produce tiny self-loops that
                // are not body; drop them.
                if (edge.a == edge.b && edge.pixels.size() < 5) continue;
                const int edgeId = static_cast<int>(rg.edges.size());
                rg.edges.push_back(std::move(edge));
                rg.incident[rg.edges.back().a].push_back(edgeId);
                if (rg.edges.back().b != rg.edges.back().a)
                    rg.incident[rg.edges.back().b].push_back(edgeId);
            }
        }
    }
    return rg;
}

struct WalkEnd {
    RouteEndKind kind = RouteEndKind::Hidden;
    int graphIndex = -1;            // skeleton index when the end is an endpoint
};

struct RawRoute {
    std::vector<cv::Point2f> points;          // start → end, video coords
    std::vector<int> junctionPointIndices;    // pass-through positions in `points`
    WalkEnd start;
    WalkEnd end;
    bool retrace = false;
    QString path;
};

float angleBetween(const cv::Point2f& a, const cv::Point2f& b)
{
    return std::atan2(a.x * b.y - a.y * b.x, a.x * b.x + a.y * b.y);
}

} // namespace

RolePrediction predictRole(bool known, const cv::Point2f& last, const cv::Point2f& velocity, int age)
{
    RolePrediction rp;
    rp.valid = known;
    rp.position = last + 0.5f * velocity;
    rp.age = age;
    rp.extraSigma = 0.5f * static_cast<float>(cv::norm(velocity));
    return rp;
}

float roleSigma(const RolePrediction& prediction, float bodyLength)
{
    const float cap = std::max(kBaseRoleSigma, 0.5f * bodyLength);
    return std::min(kBaseRoleSigma + prediction.extraSigma +
                    kRoleSigmaPerFrame * static_cast<float>(std::max(0, prediction.age)), cap);
}

float roleCost(const cv::Point2f& point, const RolePrediction& prediction, float bodyLength)
{
    if (!prediction.valid)
        return 0.5f + std::log(std::max(kBaseRoleSigma, 0.5f * bodyLength));
    const float sigma = roleSigma(prediction, bodyLength);
    const float d = static_cast<float>(cv::norm(point - prediction.position)) / sigma;
    return 0.5f * d * d + std::log(sigma);
}

float signedTurning(const std::vector<cv::Point2f>& points)
{
    float total = 0.f;
    for (size_t i = 2; i < points.size(); ++i) {
        const cv::Point2f v1 = points[i - 1] - points[i - 2];
        const cv::Point2f v2 = points[i] - points[i - 1];
        if (cv::norm(v1) < 1e-6 || cv::norm(v2) < 1e-6) continue;
        total += angleBetween(v1, v2);
    }
    return total;
}

RouteSelectionResult selectSelfCrossedRoute(const RouteSelectionInput& input)
{
    RouteSelectionResult result;
    if (!input.graph || input.graph->points.size() < 2 || input.bodyLength <= 0.f) {
        result.decisions << QStringLiteral("route selection skipped: missing skeleton or body length");
        return result;
    }
    const SkeletonGraph& graph = *input.graph;
    const cv::Point2f origin(static_cast<float>(input.localBounds.x),
                             static_cast<float>(input.localBounds.y));
    auto video = [&](int idx) { return cv::Point2f(graph.points[idx]) + origin; };
    const float L = input.bodyLength;
    const float minLength = kRouteMinLengthFraction * L;
    const float maxLength = kRouteMaxLengthFraction * L;

    // Hidden ends may lie anywhere; give each predicted end that is not already
    // explained by a visible endpoint its own node so routes can stop there.
    std::vector<int> forcedPixels;
    std::vector<int> endpointPixels;
    for (int i = 0; i < static_cast<int>(graph.points.size()); ++i)
        if (graph.adjacency[i].size() == 1) endpointPixels.push_back(i);
    for (const RolePrediction* role : {&input.head, &input.tail}) {
        if (!role->valid) continue;
        bool explained = false;
        for (int e : endpointPixels)
            explained |= cv::norm(video(e) - role->position) <= kForcedNodeMinSeparation;
        if (explained) continue;
        int nearest = -1;
        float best = std::numeric_limits<float>::max();
        for (int i = 0; i < static_cast<int>(graph.points.size()); ++i) {
            const float d = cv::norm(video(i) - role->position);
            if (d < best) { best = d; nearest = i; }
        }
        if (nearest >= 0 && best <= 0.5f * L) forcedPixels.push_back(nearest);
    }
    const RouteGraph rg = buildRouteGraph(graph, forcedPixels);

    auto tipPoint = [&](int graphIndex) -> cv::Point2f {
        for (const auto& [idx, point] : input.observedTips)
            if (idx == graphIndex) return point;
        return video(graphIndex);
    };
    auto endpointKind = [&](int nodeId) -> RouteEndKind {
        const auto& node = rg.nodes[nodeId];
        if (node.kind != RouteGraph::NodeKind::Endpoint) return RouteEndKind::Hidden;
        if (rg.incident[nodeId].empty()) return RouteEndKind::ObservedTip;
        const auto& edge = rg.edges[rg.incident[nodeId].front()];
        std::vector<cv::Point2f> pts;
        for (int p : edge.pixels) pts.push_back(video(p));
        const int other = edge.a == nodeId ? edge.b : edge.a;
        float halfWidth = 0.f;
        if (!input.distTransform.empty() && other >= 0) {
            const cv::Point& rp = graph.points[rg.nodes[other].rep];
            halfWidth = input.distTransform.at<float>(rp.y, rp.x);
        }
        const bool leadsToJunction = other >= 0 && rg.nodes[other].kind == RouteGraph::NodeKind::Junction;
        return leadsToJunction && polylineLength(pts) <= std::max(3.f, halfWidth + 1.f)
            ? RouteEndKind::ShortBranch : RouteEndKind::ObservedTip;
    };
    auto edgePoints = [&](int edgeId, int fromNode) {
        const auto& edge = rg.edges[edgeId];
        std::vector<cv::Point2f> pts;
        const bool forward = edge.a == fromNode;
        for (int k = 0; k < static_cast<int>(edge.pixels.size()); ++k) {
            const int p = forward ? edge.pixels[k] : edge.pixels[edge.pixels.size() - 1 - k];
            pts.push_back(video(p));
        }
        const int toNode = forward ? edge.b : edge.a;
        pts.insert(pts.begin(), video(rg.nodes[fromNode].rep));
        pts.push_back(video(rg.nodes[toNode].rep));
        return std::make_pair(pts, toNode);
    };
    auto append = [](std::vector<cv::Point2f>& dst, const std::vector<cv::Point2f>& src) {
        for (const auto& p : src)
            if (dst.empty() || cv::norm(dst.back() - p) > 1e-4) dst.push_back(p);
    };

    std::vector<RawRoute> raw;
    std::vector<int> starts;
    for (int id = 0; id < static_cast<int>(rg.nodes.size()); ++id)
        if (rg.nodes[id].kind == RouteGraph::NodeKind::Endpoint) starts.push_back(id);
    if (starts.empty())
        for (int id = 0; id < static_cast<int>(rg.nodes.size()); ++id)
            if (rg.nodes[id].kind == RouteGraph::NodeKind::Forced) starts.push_back(id);

    // Truncating at L measures raw pixel length, which runs a few percent
    // longer than the resampled curve compared against L.
    const float truncateAt = 1.04f * L;
    for (int start : starts) {
        WalkEnd startEnd;
        startEnd.kind = rg.nodes[start].kind == RouteGraph::NodeKind::Endpoint
            ? endpointKind(start) : RouteEndKind::Hidden;
        startEnd.graphIndex = rg.nodes[start].kind == RouteGraph::NodeKind::Endpoint
            ? rg.nodes[start].rep : -1;
        std::vector<cv::Point2f> initial;
        initial.push_back(startEnd.graphIndex >= 0 ? tipPoint(startEnd.graphIndex)
                                                   : video(rg.nodes[start].rep));
        std::vector<char> used(rg.edges.size(), 0);
        std::vector<int> junctions;
        QString path = QString::number(start);

        std::function<void(int, std::vector<cv::Point2f>&, int)> walk =
            [&](int node, std::vector<cv::Point2f>& pts, int lastEdge) {
            if (static_cast<int>(raw.size()) >= kMaxCandidates) return;
            const float len = polylineLength(pts);
            const auto kind = rg.nodes[node].kind;
            // A walk may close on a hidden start (a ring whose two ends touch).
            const bool closedOnHiddenStart = node == start && kind != RouteGraph::NodeKind::Endpoint;
            if ((node != start || closedOnHiddenStart) && lastEdge >= 0) {
                RawRoute r{pts, junctions, startEnd, {}, false, path};
                if (kind == RouteGraph::NodeKind::Endpoint) {
                    r.end.kind = endpointKind(node);
                    r.end.graphIndex = rg.nodes[node].rep;
                    r.points.back() = tipPoint(r.end.graphIndex);
                }
                raw.push_back(r);
                if (kind == RouteGraph::NodeKind::Endpoint || closedOnHiddenStart) return;
                // The body may fold back along the branch it just used.
                if (kind == RouteGraph::NodeKind::Junction && len < truncateAt - 2.f) {
                    auto [back, unused] = edgePoints(lastEdge, node);
                    (void)unused;
                    RawRoute folded{pts, junctions, startEnd, {}, true, path + QStringLiteral("~")};
                    append(folded.points, truncatePolyline(back, truncateAt - len));
                    raw.push_back(folded);
                }
            }
            for (int edgeId : rg.incident[node]) {
                if (used[edgeId]) continue;
                auto [seg, next] = edgePoints(edgeId, node);
                std::vector<cv::Point2f> extended = pts;
                const bool passThrough = lastEdge >= 0 && kind != RouteGraph::NodeKind::Endpoint;
                if (passThrough) junctions.push_back(static_cast<int>(extended.size()) - 1);
                append(extended, seg);
                const float newLen = polylineLength(extended);
                if (len < truncateAt && newLen >= truncateAt) {
                    RawRoute cut{truncatePolyline(extended, truncateAt), junctions, startEnd, {}, false,
                                 path + QStringLiteral(">%1|").arg(edgeId)};
                    raw.push_back(cut);
                }
                if (newLen <= 1.04f * maxLength) {
                    used[edgeId] = 1;
                    const QString savedPath = path;
                    path += QStringLiteral(">%1").arg(next);
                    walk(next, extended, edgeId);
                    path = savedPath;
                    used[edgeId] = 0;
                }
                if (passThrough) junctions.pop_back();
            }
        };
        walk(start, initial, -1);
    }
    result.generated = static_cast<int>(raw.size());

    // An end seen in the previous frame rarely disappears in the next, while an
    // end that is already hidden usually stays hidden.
    auto endCost = [](RouteEndKind kind, const RolePrediction& role) {
        switch (kind) {
        case RouteEndKind::ObservedTip: return 0.f;
        case RouteEndKind::ShortBranch: return kShortBranchCost;
        case RouteEndKind::Hidden:
        default: return (role.valid && role.age == 0) ? kVanishingEndCost : kHiddenEndCost;
        }
    };
    auto kindName = [](RouteEndKind kind) {
        switch (kind) {
        case RouteEndKind::ObservedTip: return QStringLiteral("tip");
        case RouteEndKind::ShortBranch: return QStringLiteral("short");
        case RouteEndKind::Hidden:
        default: return QStringLiteral("hidden");
        }
    };

    std::vector<RouteCandidate> scored;
    for (const RawRoute& r : raw) {
        if (r.points.size() < 2) continue;
        const std::vector<cv::Point2f> sampled = resamplePolyline(r.points, input.nPoints);
        const float length = polylineLength(sampled);
        if (length < minLength || length > maxLength) {
            ++result.rejectedByLength;
            continue;
        }
        float junctionCost = 0.f;
        for (int j : r.junctionPointIndices) {
            const int back = std::max(0, j - 4);
            const int ahead = std::min(static_cast<int>(r.points.size()) - 1, j + 4);
            if (back == j || ahead == j) continue;
            const float turn = std::abs(angleBetween(r.points[j] - r.points[back],
                                                     r.points[ahead] - r.points[j]));
            const float excess = std::max(0.f, turn - 0.25f * static_cast<float>(CV_PI)) /
                                 (0.5f * static_cast<float>(CV_PI));
            junctionCost += kJunctionTurnWeight * excess * excess;
        }
        const float lengthDev = (length - L) / (kLengthSigmaFraction * L);
        const float shared = 0.5f * lengthDev * lengthDev + junctionCost +
                             (r.retrace ? kRetraceCost : 0.f);

        for (int assignment = 0; assignment < 2; ++assignment) {
            RouteCandidate c;
            c.points = sampled;
            c.headKind = r.start.kind;
            c.tailKind = r.end.kind;
            if (assignment == 1) {
                std::reverse(c.points.begin(), c.points.end());
                std::swap(c.headKind, c.tailKind);
            }
            c.length = length;
            c.turning = signedTurning(c.points);
            const float headCost = roleCost(c.points.front(), input.head, L);
            const float tailCost = roleCost(c.points.back(), input.tail, L);
            float orientationCost = 0.f;
            if (input.hasOrientationReference &&
                std::abs(input.orientationReference) >= kOrientationMinReferenceTurning &&
                std::abs(c.turning) >= kOrientationMinTurning &&
                c.turning * input.orientationReference < 0.f) {
                orientationCost = kOrientationMismatchCost;
            }
            const float endsCost = endCost(c.headKind, input.head) + endCost(c.tailKind, input.tail) +
                                   (r.retrace ? kRetraceCost : 0.f);
            c.score = shared + endCost(c.headKind, input.head) + endCost(c.tailKind, input.tail) +
                      headCost + tailCost + orientationCost;
            c.summary = QStringLiteral("route %1%2 len=%3 head=%4(%5,%6) tail=%7(%8,%9) "
                                       "cost: length=%10 ends=%11 headPred=%12 tailPred=%13 "
                                       "junction=%14 orient=%15 turning=%16 total=%17")
                .arg(r.path, assignment == 1 ? QStringLiteral(" reversed") : QString())
                .arg(length, 0, 'f', 1)
                .arg(kindName(c.headKind))
                .arg(c.points.front().x, 0, 'f', 1).arg(c.points.front().y, 0, 'f', 1)
                .arg(kindName(c.tailKind))
                .arg(c.points.back().x, 0, 'f', 1).arg(c.points.back().y, 0, 'f', 1)
                .arg(0.5f * lengthDev * lengthDev, 0, 'f', 2)
                .arg(endsCost, 0, 'f', 2)
                .arg(headCost, 0, 'f', 2)
                .arg(tailCost, 0, 'f', 2)
                .arg(junctionCost, 0, 'f', 2)
                .arg(orientationCost, 0, 'f', 1)
                .arg(c.turning, 0, 'f', 2)
                .arg(c.score, 0, 'f', 2);
            scored.push_back(std::move(c));
        }
    }

    std::sort(scored.begin(), scored.end(),
              [](const RouteCandidate& a, const RouteCandidate& b) { return a.score < b.score; });
    result.decisions << QStringLiteral("route selection: bodyLength=%1 window=[%2,%3] nodes=%4 edges=%5 "
                                       "generated=%6 rejectedByLength=%7 scored=%8 orientationRef=%9")
        .arg(L, 0, 'f', 1).arg(minLength, 0, 'f', 1).arg(maxLength, 0, 'f', 1)
        .arg(rg.nodes.size()).arg(rg.edges.size())
        .arg(result.generated).arg(result.rejectedByLength).arg(scored.size())
        .arg(input.hasOrientationReference ? QString::number(input.orientationReference, 'f', 2)
                                           : QStringLiteral("none"));
    if (scored.empty()) {
        result.decisions << QStringLiteral("route selection: no route within the body-length window; frame unresolved");
        return result;
    }
    for (int i = 0; i < std::min(kRankedKept, static_cast<int>(scored.size())); ++i) {
        result.decisions << QStringLiteral("  #%1 %2").arg(i).arg(scored[i].summary);
        result.ranked.push_back(scored[i]);
    }
    result.best = scored.front();
    result.found = true;
    return result;
}

} // namespace Centerline
