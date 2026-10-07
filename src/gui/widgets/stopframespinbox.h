#ifndef STOPFRAMESPINBOX_H
#define STOPFRAMESPINBOX_H

#include <QSpinBox>

// Let users type or paste a large frame number as a shortcut to the video end.
class StopFrameSpinBox : public QSpinBox {
public:
    explicit StopFrameSpinBox(QWidget* parent = nullptr) : QSpinBox(parent) {}

protected:
    QValidator::State validate(QString& text, int& position) const override {
        return exceedsMaximum(text) ? QValidator::Acceptable
                                    : QSpinBox::validate(text, position);
    }

    int valueFromText(const QString& text) const override {
        return exceedsMaximum(text) ? maximum() : QSpinBox::valueFromText(text);
    }

private:
    bool exceedsMaximum(const QString& text) const {
        QString digits = text.trimmed();
        if (digits.startsWith(QLatin1Char('+'))) digits.remove(0, 1);
        int value = 0;
        bool exceeds = false;
        for (QChar character : digits) {
            const int digit = character.digitValue();
            if (digit < 0) return false;
            // Compare before multiplying, so even arbitrarily long input is safe.
            if (!exceeds) {
                exceeds = value > maximum() / 10 ||
                          (value == maximum() / 10 && digit > maximum() % 10);
                if (!exceeds) value = value * 10 + digit;
            }
        }
        return exceeds;
    }
};

#endif
