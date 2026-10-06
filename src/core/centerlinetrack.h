#ifndef CENTERLINETRACK_H
#define CENTERLINETRACK_H

#include "centerlineprocessor.h"

#include <QString>
#include <QStringList>
#include <map>
#include <vector>

namespace Centerline {

/**
 * @brief Settings for the whole-track continuity pass.
 *
 * `minIslandFrames` is the number of consecutive clean frames needed for a run
 * to anchor a contact bridge; shorter runs are solved as part of the gap.
 * `reviewMargin` is the cost margin a bridge needs before its end pairing is
 * trusted; below it the chain breaks and the bridge is flagged for review.
 */
struct TrackPassConfig {
    bool  skipMergedFrames = false;
    int   minIslandFrames = 25;
    float reviewMargin = 4.f;
    float frameResidualReview = 8.f;
};

// A bridge whose margin was too small to merge two chains. `reversed` is the
// bridge's best guess: whether the right chain must be flipped to agree with
// the left chain, both as left by processTrackContinuity.
struct ChainLink {
    int leftChain = -1;
    int rightChain = -1;
    bool reversed = false;
    float margin = 0.f;
};

struct TrackPassResult {
    // Frame numbers per continuity chain: every frame in a chain has head/tail
    // oriented consistently with the others, so a later head/tail decision
    // must flip a chain as a unit.
    std::vector<std::vector<int>> chains;
    std::vector<ChainLink> weakLinks;
    std::map<int, QString> review;   // frame number -> reason for human review
    QStringList log;
};

/**
 * @brief Compute centerlines for one worm with continuity carried across self-contacts.
 *
 * Pass 1 processes every clean frame independently and links consecutive clean
 * frames into islands by matching whole centerlines, so each physical end is
 * tracked without naming it. Pass 2 bridges every gap between anchoring islands
 * (self-crossed, short clean runs, merged or lost frames): candidate routes are
 * listed per frame and the cheapest sequence is chosen with both neighbouring
 * islands fixed, once for each pairing of the right island's ends. The cheaper
 * pairing orients the right island; the cost difference is the bridge margin.
 * Islands linked by trusted bridges form a chain with one consistent labeling.
 */
TrackPassResult processTrackContinuity(const CenterlineFrameContext& ctx,
                                       CenterlineFrameIo& io,
                                       const TrackPassConfig& config);

/**
 * @brief Name undecided chains from decided neighbours across weak links.
 *
 * `decided[c]` marks chains whose head/tail naming came from direct evidence;
 * `flipped[c]` records whether chain c has been reversed since
 * processTrackContinuity. For each weak link with exactly one decided side,
 * the other side is flipped (if needed) to follow the link's best guess and
 * becomes decided. Repeats until nothing changes. Returns the flipped frames.
 */
QList<int> propagateAcrossWeakLinks(CenterlineFrameIo& io, int wormId,
                                    const TrackPassResult& continuity,
                                    std::vector<bool>& decided,
                                    std::vector<bool>& flipped);

// Reverse a stored frame's centerline and swap its head/tail roles, keeping its
// DEBUG record (when available) in step. `reason` is appended to the record.
void reverseStoredFrame(CenterlineFrameIo& io, int wormId, int frameNumber, const QString& reason);

// Mark a stored frame for human review with the given reason (appended when already flagged).
void flagStoredFrame(CenterlineFrameIo& io, int wormId, int frameNumber, const QString& reason);

} // namespace Centerline

#endif // CENTERLINETRACK_H
