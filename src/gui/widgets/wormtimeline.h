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
    void setWormColors(const QMap<int, QColor>& idColors);
    void setMergeGroupsByFrame(const QMap<int, QList<QList<int>>>& mergeGroupsByFrame);

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

signals:
    void eventClicked(int frame, const QList<int>& wormIds);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;

private:
    struct EventNode {
        int frame = 0;
        QList<int> wormIds;
        QRectF bounds;
    };

    void rebuildEventNodes();
    double frameToX(int frame, const QRect& contentRect) const;
    QList<int> sortedWormIds() const;

    int m_totalFrames = 0;
    int m_keyframeFrame = 0;
    QMap<int, QColor> m_wormColors;
    QMap<int, QList<QList<int>>> m_mergeGroupsByFrame;

    QList<EventNode> m_eventNodes;
    int m_selectedNodeIndex = -1;

    int m_lineThickness = 6;
    int m_minRowSpacing = 12;
    int m_leftPadding = 12;
    int m_rightLabelWidth = 90;
    int m_topPadding = 12;
    int m_bottomPadding = 12;
};

#endif // WORMTIMELINE_H
