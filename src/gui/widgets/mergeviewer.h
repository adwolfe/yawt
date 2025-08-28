#ifndef MERGEVIEWER_H
#define MERGEVIEWER_H

#include <QWidget>
#include <QMap>
#include <QColor>
#include <QSet>

///
/// MergeViewer
/// A compact visual widget that represents a 5-frame history (centered on the current frame)
/// and shows which worm identities (from a provided visible set) are present in view.
///
/// Responsibilities implemented in this header (declaration only):
/// - Divide the widget horizontally into 5 segments with dashed vertical separators.
/// - Draw horizontal background bars (one per visible worm id) spanning the full width,
///   each painted using the color associated with that worm id.
/// - Label each bar at its left edge with the worm id number.
/// - Provide a public API for feeding visible worm ids and their colors, and for updating
///   the current frame index (the center of the 5-frame window).
///
/// Notes / future work:
/// - Circular nodes for blobs centered in each segment (overlay) are planned and will be added
///   later (not implemented here).
/// - The color mapping is supplied by the caller as a map from worm id -> QColor. This keeps
///   this widget independent of the blob table model and allows MainWindow (or another owner)
///   to pass the same colors used elsewhere in the UI.
///
class MergeViewer : public QWidget
{
    Q_OBJECT

public:
    explicit MergeViewer(QWidget* parent = nullptr);
    ~MergeViewer() override = default;

    // Set the visible worms and their display colors. Passing an empty map clears the set.
    // The keys are worm IDs; the values are the desired QColor for the corresponding worm.
    void setVisibleWormColors(const QMap<int, QColor>& wormColors);

    // Convenience: set visible IDs without providing colors. The widget will use a fallback
    // color (gray) for IDs that aren't present in the colors map previously provided.
    void setVisibleWormIds(const QSet<int>& ids);

    // Provide per-frame visibility mapping so bars can be painted partially per segment.
    // Key = frame number, value = set of visible worm IDs for that frame.
    // If this map is empty the widget will fall back to using the visible IDs/colors
    // supplied via setVisibleWormIds / setVisibleWormColors.
    void setVisibleByFrame(const QMap<int, QSet<int>>& visibleByFrame);

    // Set the currently selected frame (center of the 5-frame window).
    // This affects which frame label is considered the center when drawing overlays.
    void setCurrentFrame(int frameNumber);

    // Set the radius (# frames either side of current) to display.
    // Default is 2 (5 segments total). Minimum value is 1.
    void setRadius(int radius);

    // Optional appearance tuning:
    void setBarSpacing(int px);        // whitespace between horizontal bars
    void setBarHeight(int px);         // height of each worm bar
    void setLeftLabelWidth(int px);    // width reserved for the left-side ID labels

    // Suggested size
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

public slots:
    // Combined update helper that sets both visible colors and current frame then repaints.
    void updateVisibleAndFrame(const QMap<int, QColor>& wormColors, int currentFrame);

protected:
    void paintEvent(QPaintEvent* ev) override;

private:
    // Internal helpers
    QColor colorForId(int id) const;

    // Visible worm information: map id -> color. Empty map means no visible worms.
    QMap<int, QColor> m_visibleWormColors;
    QSet<int> m_visibleIdsCache; // quick list of ids when caller uses setVisibleWormIds

    // Optional per-frame visibility information. This allows the widget to paint
    // bars partially per segment: the map key is the frame number and the value is
    // the set of worm IDs visible in that frame. If empty, painting falls back to
    // m_visibleIdsCache / m_visibleWormColors behavior.
    QMap<int, QSet<int>> m_visibleByFrame;

    int m_currentFrame = 0;
    int m_radius = 2; // default => 2 on each side (total segments = 2*radius + 1 = 5)

    // Increase default bar height by ~50% and increase spacing to roughly the bar height
    // (per your request to make bars wider and spacing larger).
    int m_barHeight = 27;
    int m_barSpacing = 27;

    int m_leftLabelWidth = 36;

    QColor m_defaultColor = QColor(160, 160, 160); // fallback gray
};

#endif // MERGEVIEWER_H