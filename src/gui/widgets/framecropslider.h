#ifndef FRAMECROPSLIDER_H
#define FRAMECROPSLIDER_H

#include <QSlider>
#include <QStylePainter>
#include <QStyleOptionSlider>

// Keep the full video scale while restricting navigation to an inclusive crop.
class FrameCropSlider : public QSlider {
public:
    explicit FrameCropSlider(QWidget* parent = nullptr) : QSlider(parent) {
        connect(this, &QSlider::sliderMoved, this, [this](int position) {
            setSliderPosition(boundedFrame(position));
        });
        // Qt emits this before committing a keyboard, wheel, or groove action.
        connect(this, &QSlider::actionTriggered, this, [this](int) {
            setSliderPosition(boundedFrame(sliderPosition()));
        });
    }

    void setFrameCrop(int first, int last) {
        m_first = qBound(minimum(), first, maximum());
        m_last = qBound(m_first, last, maximum());
        setValue(value());
        setSliderPosition(boundedFrame(sliderPosition()));
        update();
    }
    int boundedFrame(int frame) const { return qBound(m_first, frame, m_last); }
    void setValue(int frame) { QSlider::setValue(boundedFrame(frame)); }

protected:
    void paintEvent(QPaintEvent*) override {
        QStylePainter painter(this);
        QStyleOptionSlider option;
        initStyleOption(&option);
        option.subControls = QStyle::SC_SliderGroove | QStyle::SC_SliderTickmarks;
        painter.drawComplexControl(QStyle::CC_Slider, option);

        auto centerForFrame = [&](int frame) {
            auto positionOption = option;
            positionOption.sliderPosition = frame;
            return style()->subControlRect(QStyle::CC_Slider, &positionOption,
                                            QStyle::SC_SliderHandle, this).center();
        };
        const QRect groove = style()->subControlRect(QStyle::CC_Slider, &option,
                                                     QStyle::SC_SliderGroove, this);
        QPointF first = centerForFrame(minimum());
        QPointF last = centerForFrame(maximum());
        // The groove extends beyond the handle's travel at both ends.
        // Cover those end caps too, while retaining frame-aligned inner edges.
        if (orientation() == Qt::Horizontal) {
            first.setX(option.upsideDown ? groove.right() + 1 : groove.left());
            last.setX(option.upsideDown ? groove.left() : groove.right() + 1);
        } else {
            first.setY(option.upsideDown ? groove.bottom() + 1 : groove.top());
            last.setY(option.upsideDown ? groove.top() : groove.bottom() + 1);
        }
        painter.setPen(QPen(QColor(128, 128, 128), 8, Qt::SolidLine, Qt::FlatCap));
        if (m_first > minimum()) painter.drawLine(first, centerForFrame(m_first));
        if (m_last < maximum()) painter.drawLine(centerForFrame(m_last), last);

        option.subControls = QStyle::SC_SliderHandle;
        painter.drawComplexControl(QStyle::CC_Slider, option);
    }

private:
    int m_first = 0;
    int m_last = 0;
};

#endif
