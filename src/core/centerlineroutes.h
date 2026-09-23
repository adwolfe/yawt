#ifndef CENTERLINEROUTES_H
#define CENTERLINEROUTES_H

#include "centerlinetypes.h"

#include <QStringList>
#include <vector>

namespace Centerline {

/**
 * @brief Predicted position of one body end, weighted by how long ago it was
 *        actually observed.
 *
 * `age` is the number of frames since the end was last seen as a real tip
 * (0 = seen in the previous frame). Hypothesised hidden positions never reset
 * the age, so a guess cannot masquerade as an observation. The spread of the
 * prediction grows with age; see `roleSigma()`.
 */
struct RolePrediction {
    bool        valid = false;
    cv::Point2f position{0.f, 0.f};
    int         age = 0;
    float       extraSigma = 0.f;   // added spread, e.g. from an uncertain velocity
};

// Build a prediction from an end's last position and per-frame velocity. Tip
// velocities are noisy, so only half is extrapolated and the spread widens
// with speed.
RolePrediction predictRole(bool known, const cv::Point2f& last, const cv::Point2f& velocity, int age);

// Standard deviation (px) of a role prediction.
float roleSigma(const RolePrediction& prediction, float bodyLength);

// Negative log-likelihood of observing a tip at `point` given `prediction`,
// including the log-spread term so a vague prediction cannot match everything
// cheaply. An invalid prediction costs as much as a 1σ hit at the widest spread.
float roleCost(const cv::Point2f& point, const RolePrediction& prediction, float bodyLength);

// Total signed turning angle (radians) of an ordered polyline. Scale-free,
// and negated when the traversal direction is reversed.
float signedTurning(const std::vector<cv::Point2f>& points);

/**
 * @brief Inputs for choosing a centerline through a self-crossed skeleton.
 *
 * Coordinates: `graph` and `distTransform` are local to `localBounds`;
 * everything else is in video coordinates.
 */
struct RouteSelectionInput {
    const SkeletonGraph* graph = nullptr;
    cv::Mat              distTransform;
    cv::Rect             localBounds;
    // Observed tips from detectEndpoints, keyed by skeleton graph index.
    std::vector<std::pair<int, cv::Point2f>> observedTips;
    RolePrediction       head;
    RolePrediction       tail;
    float                bodyLength = 0.f;   // expected resampled arc length
    int                  nPoints = 20;
    // Loop orientation (signed turning, head→tail) captured before contact.
    bool                 hasOrientationReference = false;
    float                orientationReference = 0.f;
};

enum class RouteEndKind {
    ObservedTip,   // a degree-1 skeleton endpoint with a real branch behind it
    ShortBranch,   // a degree-1 endpoint on a branch no longer than the body half-width + 1
    Hidden         // the body continues out of sight: truncated, junction, or retrace
};

struct RouteCandidate {
    std::vector<cv::Point2f> points;       // head → tail, video coords
    RouteEndKind headKind = RouteEndKind::Hidden;
    RouteEndKind tailKind = RouteEndKind::Hidden;
    float length = 0.f;                    // resampled arc length
    float turning = 0.f;                   // signed turning head → tail
    float score = 0.f;
    float orientationCost = 0.f;
    std::pair<int, int> labeling{-1, -1};   // visible (head, tail) graph indices; -1 = hidden
    QString summary;
};

struct RouteSelectionResult {
    bool found = false;
    RouteCandidate best;
    std::vector<RouteCandidate> ranked;     // best first; includes rejected-by-length only in counts
    int  generated = 0;
    int  rejectedByLength = 0;
    QStringList decisions;
};

// Length window accepted for a self-crossed route, as fractions of body length.
constexpr float kRouteMinLengthFraction = 0.75f;
constexpr float kRouteMaxLengthFraction = 1.15f;

/**
 * @brief Enumerate routes through a self-crossed skeleton and pick the one most
 *        consistent with the body length, observed tips, role predictions,
 *        loop orientation, and straight passage through crossings.
 *
 * Every route is an edge-simple walk through the junction-compressed skeleton
 * starting at an observed endpoint (or at the node nearest a predicted end
 * when none is visible). A route may end at another endpoint, at a junction,
 * part-way along a branch once it reaches the body length, or after retracing
 * its last branch when the body runs back along itself. Routes whose length is
 * outside [kRouteMinLengthFraction, kRouteMaxLengthFraction] × bodyLength are
 * rejected. Both head/tail assignments of each surviving route are scored and
 * the lowest-cost one is returned with points ordered head → tail.
 */
RouteSelectionResult selectSelfCrossedRoute(const RouteSelectionInput& input);

} // namespace Centerline

#endif // CENTERLINEROUTES_H
