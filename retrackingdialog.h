#ifndef RETRACKINGDIALOG_H
#define RETRACKINGDIALOG_H

#include <QDialog>
#include "trackingcommon.h"

QT_BEGIN_NAMESPACE
namespace Ui { class RetrackingDialog; }
QT_END_NAMESPACE

struct RetrackingParameters {
    int fixBlobId;
    int startFrame;
    int endFrame;
    bool autoRange;
    int rangeSize;
    bool replaceExisting;
    bool extendTrack;
    
    RetrackingParameters() 
        : fixBlobId(-1), startFrame(0), endFrame(0), autoRange(true), 
          rangeSize(50), replaceExisting(false), extendTrack(true) {}
};

class RetrackingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RetrackingDialog(QWidget *parent = nullptr);
    ~RetrackingDialog();

    // Setup the dialog with Fix blob information
    void setFixBlobInfo(const TableItems::ClickedItem& fixBlob, int totalFrames);
    
    // Get the retracking parameters set by user
    ::RetrackingParameters getRetrackingParameters() const;

public slots:
    void onAutoRangeToggled(bool enabled);
    void updateFrameRanges();
    void accept() override;

private slots:
    void onRangeSizeChanged();

private:
    Ui::RetrackingDialog *ui;
    TableItems::ClickedItem m_fixBlob;
    int m_totalFrames;
    
    void updateAutoRange();
    void validateFrameRanges();
    void updateStatusLabel();
};

#endif // RETRACKINGDIALOG_H