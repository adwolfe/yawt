#include "src/utils/thresholdingutils.h"
#include "src/gui/mainwindow.h"
#include "src/gui/widgets/videoloader.h"
#include "src/core/processing/videoprocessor.h"
#include <QApplication>
#include <QCheckBox>
#include <QElapsedTimer>
#include <QGridLayout>
#include <QLoggingCategory>
#include <QTemporaryDir>
#include <QThread>
#include <cstdio>
#include <stdexcept>

static void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
static void waitForFrame(VideoLoader& loader) {
    QElapsedTimer timer;
    timer.start();
    while (loader.getCurrentFrameNumber() < 0 && timer.elapsed() < 10000) {
        QApplication::processEvents();
        QThread::msleep(1);
    }
    check(loader.getCurrentFrameNumber() == 0, "video first frame ready");
}
static cv::Mat frame(bool light, int index) {
    cv::Mat image(80, 160, CV_8UC3, cv::Scalar::all(light ? 255 : 0));
    cv::rectangle(image, cv::Rect(10, 10, 9, 9), cv::Scalar::all(light ? 0 : 255), cv::FILLED);
    cv::rectangle(image, cv::Rect(40 + index * 3, 40, 9, 9), cv::Scalar::all(light ? 0 : 255), cv::FILLED);
    return image;
}
static QString makeVideo(const QString& directory, bool light) {
    QString path = directory + (light ? "/light.avi" : "/dark.avi");
    cv::VideoWriter writer(path.toStdString(), cv::VideoWriter::fourcc('M','J','P','G'), 20, cv::Size(160, 80));
    check(writer.isOpened(), "create video fixture");
    for (int i = 0; i < 31; ++i) writer.write(frame(light, i));
    return path;
}
static void testThresholding(bool light) {
    std::vector<cv::Mat> samples;
    for (int i = 0; i < 31; ++i) samples.push_back(frame(light, i));
    auto background = ThresholdingUtils::computeMedianBackground(samples);
    check(background.at<uchar>(14, 14) == (light ? 0 : 255), "median retains stationary debris");
    check(background.at<uchar>(44, 44) == (light ? 255 : 0), "median excludes moving object");
    Thresholding::ThresholdSettings settings;
    settings.assumeLightBackground = light;
    settings.enableBackgroundSubtraction = true;
    settings.medianBackground = background;
    settings.globalThresholdValue = 128;
    settings.adaptiveBlockSize = 21;
    settings.adaptiveCValue = 2;
    const auto original = samples.front().clone();
    for (auto algorithm : {Thresholding::ThresholdAlgorithm::Global, Thresholding::ThresholdAlgorithm::Otsu,
                           Thresholding::ThresholdAlgorithm::AdaptiveMean, Thresholding::ThresholdAlgorithm::AdaptiveGaussian}) {
        settings.algorithm = algorithm;
        for (bool blur : {false, true}) {
            settings.enableBlur = blur;
            cv::Mat mask;
            ThresholdingUtils::applyThresholding(samples.front(), mask, settings);
            check(!mask.empty(), "threshold returns mask");
            check(mask.at<uchar>(14, 14) == 0, "stationary debris removed for each algorithm and polarity");
            check(mask.at<uchar>(44, 44) == 255, "moving foreground survives for each algorithm and polarity");
            check(mask.at<uchar>(70, 150) == 0, "background remains unselected");
        }
    }
    check(cv::norm(original, samples.front(), cv::NORM_INF) == 0, "input frames remain unchanged");
    settings.algorithm = Thresholding::ThresholdAlgorithm::Global;
    settings.enableBlur = false;
    settings.enableBackgroundSubtraction = false;
    cv::Mat mask;
    ThresholdingUtils::applyThresholding(samples.front(), mask, settings);
    check(mask.at<uchar>(14, 14) == 255, "disabled subtraction restores original thresholding");
    settings.enableBackgroundSubtraction = true;
    settings.medianBackground.release();
    ThresholdingUtils::applyThresholding(samples.front(), mask, settings);
    check(mask.empty(), "missing model cannot silently produce ordinary thresholds");
    std::vector<cv::Mat> rois;
    for (const auto& sample : samples) rois.push_back(sample(cv::Rect(1, 1, 150, 70)));
    check(!ThresholdingUtils::computeMedianBackground(rois).empty(), "median accepts noncontiguous frames");
    check(ThresholdingUtils::computeMedianBackground({samples.front(), cv::Mat()}).empty(), "median rejects empty sample");
}
static void testIntegration(const QString& directory) {
    const auto lightPath = makeVideo(directory, true);
    const auto darkPath = makeVideo(directory, false);
    MainWindow window;
    auto* video = window.findChild<VideoLoader*>();
    auto* checkbox = window.findChild<QCheckBox*>("backgroundSubtractCheck");
    auto* blur = window.findChild<QCheckBox*>("blurCheck");
    check(video && checkbox && blur, "background checkbox present");
    auto* layout = qobject_cast<QGridLayout*>(checkbox->parentWidget()->layout());
    int row, column, rows, columns, blurRow;
    layout->getItemPosition(layout->indexOf(checkbox), &row, &column, &rows, &columns);
    layout->getItemPosition(layout->indexOf(blur), &blurRow, &column, &rows, &columns);
    check(row == blurRow + 1, "background checkbox directly below blur");
    check(!checkbox->isChecked(), "subtraction defaults off");
    check(video->loadVideo(lightPath), "load light video");
    waitForFrame(*video);
    video->setThresholdValue(128);
    video->setViewModeOption(VideoLoader::ViewModeOption::Threshold, true);
    checkbox->setChecked(true);
    auto settings = video->getCurrentThresholdSettings();
    check(settings.enableBackgroundSubtraction && !settings.medianBackground.empty(), "checkbox builds model");
    check(video->getCurrentQImageFrame().pixelColor(14, 14).red() == 0, "preview removes debris");
    check(video->getCurrentQImageFrame().pixelColor(44, 44).red() == 255, "preview retains moving foreground");
    VideoProcessor processor;
    std::vector<cv::Mat> masks;
    QObject::connect(&processor, &VideoProcessor::rangeProcessingComplete,
                     [&](int, const std::vector<cv::Mat>& frames, bool) { masks = frames; });
    processor.processFrameRange(lightPath, settings, 0, 2, 0, true);
    check(masks.size() == 2 && masks[0].at<uchar>(14, 14) == 0 && masks[0].at<uchar>(44, 44) == 255,
          "tracking uses same model as preview");
    check(video->loadVideo(darkPath), "switch videos with subtraction enabled");
    waitForFrame(*video);
    video->setAssumeLightBackground(false);
    video->setViewModeOption(VideoLoader::ViewModeOption::Threshold, true);
    settings = video->getCurrentThresholdSettings();
    check(settings.medianBackground.at<uchar>(14, 14) > 240, "new video rebuilds background");
    check(video->getCurrentQImageFrame().pixelColor(14, 14).red() == 0, "dark preview removes debris");
    check(video->getCurrentQImageFrame().pixelColor(44, 44).red() == 255, "dark preview retains foreground");
    checkbox->setChecked(false);
    check(!video->getCurrentThresholdSettings().enableBackgroundSubtraction, "unchecking restores ordinary thresholding");
    window.resize(1200, 900);
    window.show();
    QApplication::processEvents();
    window.grab().save("/tmp/yawt-background-ui.png");
}
int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QLoggingCategory::setFilterRules("*.debug=false");
    QTemporaryDir temp;
    try {
        check(temp.isValid(), "create fixtures");
        testThresholding(true);
        testThresholding(false);
        testIntegration(temp.path());
        std::puts("Background subtraction regressions passed.");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAILED: %s\n", error.what());
        return 1;
    }
}
