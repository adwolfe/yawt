#ifndef THRESHOLDVIDEOWORKER_H
#define THRESHOLDVIDEOWORKER_H

#include <QThread>
#include <QString>
#include <QVariantMap>
#include <QMutex>

// Forward declare OpenCV types
namespace cv {
class Mat;
class VideoCapture;
class VideoWriter;
}

// Forward declare ThresholdAlgorithm from VideoLoader.h or define it in a shared header
// For simplicity, assuming VideoLoader.h is included where this is used, or define locally if preferred.
// If VideoLoader.h is included, ensure no circular dependencies.
// A better way is a shared enum header: e.g. "ProcessingCommon.h"
#ifndef VIDEOLOADER_H // Basic guard if VideoLoader.h might include this indirectly
enum class ThresholdAlgorithm { Global, Otsu, AdaptiveMean, AdaptiveGaussian }; // Duplicate for now if not shared
#endif


class ThresholdVideoWorker : public QThread {
    Q_OBJECT
public:
    explicit ThresholdVideoWorker(const QString& inputVideoPath,
                                  const QString& outputVideoPath,
                                  const QVariantMap& thresholdParameters,
                                  QObject *parent = nullptr);
    ~ThresholdVideoWorker();

    void stopProcessing();

signals:
    void progressUpdated(int percentage);
    void processingFinished(const QString& outputPath, bool success, const QString& message);

protected:
    void run() override;

private:
    bool initializeIO();
    cv::Mat applyThresholdingLogic(const cv::Mat& inputFrame); // Internal thresholding logic

    QString m_inputVideoPath;
    QString m_outputVideoPath;
    QVariantMap m_thresholdParameters;

    cv::VideoCapture* m_videoCapture;
    cv::VideoWriter* m_videoWriter;

    bool m_running;
    QMutex m_mutex; // To protect m_running flag
};

#endif // THRESHOLDVIDEOWORKER_H
