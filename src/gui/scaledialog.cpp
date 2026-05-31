#include "scaledialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QPushButton>

ScaleDialog::ScaleDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("Scale Calibration");
    setModal(true);
    setMinimumWidth(320);

    auto* root = new QVBoxLayout(this);
    root->setSpacing(12);

    // Instruction
    auto* instruction = new QLabel(
        "Draw a line on the live view representing a known distance.\n"
        "Enter that distance below, then click Measure.", this);
    instruction->setWordWrap(true);
    root->addWidget(instruction);

    // Value + unit row
    auto* inputRow = new QHBoxLayout;
    inputRow->addWidget(new QLabel("Line represents:", this));
    m_valueSpin = new QDoubleSpinBox(this);
    m_valueSpin->setDecimals(3);
    m_valueSpin->setRange(0.001, 1e6);
    m_valueSpin->setValue(1.0);
    m_valueSpin->setSingleStep(0.1);
    inputRow->addWidget(m_valueSpin);
    m_unitCombo = new QComboBox(this);
    m_unitCombo->addItem("mm");
    m_unitCombo->addItem("µm");
    m_unitCombo->addItem("cm");
    m_unitCombo->addItem("inch");
    inputRow->addWidget(m_unitCombo);
    root->addLayout(inputRow);

    // Buttons
    auto* buttons = new QDialogButtonBox(this);
    auto* cancelBtn  = buttons->addButton(QDialogButtonBox::Cancel);
    auto* measureBtn = buttons->addButton("Measure →", QDialogButtonBox::AcceptRole);
    measureBtn->setDefault(true);
    Q_UNUSED(cancelBtn)
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    root->addWidget(buttons);
}

double ScaleDialog::physicalValue() const
{
    return m_valueSpin->value();
}

QString ScaleDialog::unit() const
{
    return m_unitCombo->currentText();
}
