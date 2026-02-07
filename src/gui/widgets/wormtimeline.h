#ifndef WORMTIMELINE_H
#define WORMTIMELINE_H

#include <QWidget>
#include <QColor>
#include <QList>
#include <QMap>

class WormTimeline : public QWidget
{
    Q_OBJECT

public:
    explicit WormTimeline(QWidget* parent = nullptr);
    ~WormTimeline() override = default;

    void setTotalFrames(int totalFrames);
    void setKeyframeFrame(int frame);
    void setCurrentFrame(int frame);
    void setWormColors(const QMap<int, QColor>& idColors);
    void setMergeGroupsByFrame(const QMap<int, QList<QList<int>>>& mergeGroupsByFrame);

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

signals:
    void eventClicked(int frame, const QList<int>& wormIds);
    void frameScrubbed(int frame);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    struct EventNode {
        int frame = 0;
        QList<int> wormIds;
        QRectF bounds;
    };
    struct MergeSpan {
        int startFrame = 0;
        int endFrame = 0;
        QList<int> wormIds;
    };

    void rebuildEventNodes();
    double frameToX(int frame, const QRect& contentRect) const;
    double frameToXWithZoom(int frame, const QRect& contentRect) const;
    int xToFrameWithZoom(double x, const QRect& contentRect) const;
    void updatePanForZoom(const QRect& contentRect, double zoomFactor, double cursorX);
    QList<int> sortedWormIds() const;

    int m_totalFrames = 0;
    int m_keyframeFrame = 0;
    int m_currentFrame = 0;
    QMap<int, QColor> m_wormColors;
    QMap<int, QList<QList<int>>> m_mergeGroupsByFrame;

    QList<EventNode> m_eventNodes;
    QList<MergeSpan> m_mergeSpans;
    int m_selectedNodeIndex = -1;

    double m_zoom = 1.0;
    double m_panX = 0.0;
    bool m_isPanning = false;
    double m_lastPanX = 0.0;
    bool m_isDraggingFrame = false;

    int m_lineThickness = 6;
    int m_minRowSpacing = 12;
    int m_leftPadding = 12;
    int m_rightLabelWidth = 90;
    int m_topPadding = 12;
    int m_bottomPadding = 12;
};

#endif // WORMTIMELINE_H
