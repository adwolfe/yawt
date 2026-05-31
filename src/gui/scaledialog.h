#pragma once

#include <QDialog>
#include <QString>

class QDoubleSpinBox;
class QComboBox;

/**
 * ScaleDialog — asks the user what physical distance their measurement line will represent.
 *
 * Usage:
 *   ScaleDialog dlg(this);
 *   if (dlg.exec() == QDialog::Accepted) {
 *       double value = dlg.physicalValue();   // e.g. 1.0
 *       QString unit = dlg.unit();            // e.g. "mm"
 *   }
 */
class ScaleDialog : public QDialog
{
    Q_OBJECT
public:
    explicit ScaleDialog(QWidget* parent = nullptr);

    double  physicalValue() const;
    QString unit()          const;

private:
    QDoubleSpinBox* m_valueSpin = nullptr;
    QComboBox*      m_unitCombo = nullptr;
};
