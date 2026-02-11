#include "analysisdialog.h"

#include "trackingdatastorage.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLineF>
#include <QPainter>
#include <QPainterPath>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <limits>

TrackXYPlotWidget::TrackXYPlotWidget(TrackingDataStorage *storage, PlotMode mode, QWidget *parent)
    : QWidget(parent)
    , m_storage(storage)
    , m_plotMode(mode)
{
    setMinimumSize(QSize(320, 240));
    if (m_storage) {
        connect(m_storage, &TrackingDataStorage::itemsChanged,
                this, [this](const QList<TableItems::ClickedItem> &items) {
                    refreshItems(items);
                    update();
                });
        connect(m_storage, &TrackingDataStorage::allDataChanged,
                this, [this]() { update(); });
        connect(m_storage, &TrackingDataStorage::trackAdded,
                this, [this](int) { update(); });
        connect(m_storage, &TrackingDataStorage::trackRemoved,
                this, [this](int) { update(); });
        refreshItems(m_storage->getAllItems());
    }
}

void TrackXYPlotWidget::setArenaShape(int index)
{
    m_arenaShape = (index == 1) ? ArenaShape::Square : ArenaShape::Round;
    update();
}

void TrackXYPlotWidget::setArenaSizeMm(int sizeMm)
{
    if (sizeMm <= 0) {
        return;
    }
    m_arenaSizeMm = static_cast<double>(sizeMm);
    update();
}

void TrackXYPlotWidget::setPixelSizeUmPerPixel(double umPerPixel)
{
    if (umPerPixel < 0.0) {
        umPerPixel = 0.0;
    }
    m_umPerPixel = umPerPixel;
    update();
}

void TrackXYPlotWidget::setVideoFps(double fps)
{
    if (fps < 0.0) {
        fps = 0.0;
    }
    m_videoFps = fps;
    update();
}

void TrackXYPlotWidget::setSpeedLutRange(bool enabled, double minSpeed, double maxSpeed)
{
    m_speedRangeEnabled = enabled;
    m_speedRangeMin = minSpeed;
    m_speedRangeMax = maxSpeed;
    update();
}

void TrackXYPlotWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)
    QPainter painter(this);
    painter.fillRect(rect(), palette().window());
    painter.setRenderHint(QPainter::Antialiasing, true);

    if (!m_storage) {
        return;
    }

    const Tracking::AllWormTracks &tracks = m_storage->getAllTracks();
    const bool hasTracks = !tracks.empty();
    const bool hasPoints = !m_items.isEmpty();
    if (!hasTracks && !hasPoints && m_umPerPixel <= 0.0) {
        painter.setPen(palette().text().color());
        painter.drawText(rect(), Qt::AlignCenter, tr("No tracks or points to display"));
        return;
    }

    QRectF bounds;
    bool hasBounds = false;
    if (hasTracks) {
        for (const auto &pair : tracks) {
            const auto &points = pair.second;
            for (const auto &point : points) {
                if (point.quality == Tracking::TrackPointQuality::Lost) {
                    continue;
                }
                const QPointF pos(point.position.x, point.position.y);
                if (!hasBounds) {
                    bounds = QRectF(pos, QSizeF(1.0, 1.0));
                    hasBounds = true;
                } else {
                    bounds.setLeft(std::min(bounds.left(), pos.x()));
                    bounds.setRight(std::max(bounds.right(), pos.x()));
                    bounds.setTop(std::min(bounds.top(), pos.y()));
                    bounds.setBottom(std::max(bounds.bottom(), pos.y()));
                }
            }
        }
    }

    QPointF centerPoint;
    bool hasCenterPoint = false;
    for (const auto &item : m_items) {
        if (item.type != TableItems::ItemType::ROI
            && item.type != TableItems::ItemType::StartPoint
            && item.type != TableItems::ItemType::EndPoint
            && item.type != TableItems::ItemType::ControlPoint
            && item.type != TableItems::ItemType::CenterPoint) {
            continue;
        }
        const QPointF pos = item.initialCentroid;
        if (!hasCenterPoint && item.type == TableItems::ItemType::CenterPoint) {
            centerPoint = pos;
            hasCenterPoint = true;
        }
        if (!hasBounds) {
            bounds = QRectF(pos, QSizeF(1.0, 1.0));
            hasBounds = true;
        } else {
            bounds.setLeft(std::min(bounds.left(), pos.x()));
            bounds.setRight(std::max(bounds.right(), pos.x()));
            bounds.setTop(std::min(bounds.top(), pos.y()));
            bounds.setBottom(std::max(bounds.bottom(), pos.y()));
        }
    }

    QRectF fieldBounds;
    if (m_umPerPixel > 0.0 && m_arenaSizeMm > 0.0) {
        const double fieldDiameterPx = (m_arenaSizeMm * 1000.0) / m_umPerPixel;
        if (fieldDiameterPx > 0.0) {
            QPointF center = hasCenterPoint ? centerPoint
                                            : (hasBounds ? bounds.center() : QPointF(0.0, 0.0));
            fieldBounds = QRectF(center.x() - fieldDiameterPx / 2.0,
                                 center.y() - fieldDiameterPx / 2.0,
                                 fieldDiameterPx,
                                 fieldDiameterPx);
            if (!hasBounds) {
                bounds = fieldBounds;
                hasBounds = true;
            } else {
                bounds.setLeft(std::min(bounds.left(), fieldBounds.left()));
                bounds.setRight(std::max(bounds.right(), fieldBounds.right()));
                bounds.setTop(std::min(bounds.top(), fieldBounds.top()));
                bounds.setBottom(std::max(bounds.bottom(), fieldBounds.bottom()));
            }
        }
    }

    if (!hasBounds || bounds.width() <= 0.0 || bounds.height() <= 0.0) {
        painter.setPen(palette().text().color());
        painter.drawText(rect(), Qt::AlignCenter, tr("No valid points"));
        return;
    }

    const qreal padding = 18.0;
    QRectF availableRect = rect().adjusted(padding, padding, -padding, -padding);
    const qreal side = std::max(0.0, std::min(availableRect.width(), availableRect.height()));
    const qreal left = availableRect.left() + (availableRect.width() - side) / 2.0;
    const qreal top = availableRect.top() + (availableRect.height() - side) / 2.0;
    QRectF plotRect(left, top, side, side);

    painter.setPen(QPen(palette().mid().color(), 1.0));
    painter.drawRect(plotRect);

    auto mapPoint = [&bounds, &plotRect](const QPointF &pos) {
        const qreal xRatio = (pos.x() - bounds.left()) / bounds.width();
        const qreal yRatio = (pos.y() - bounds.top()) / bounds.height();
        const qreal x = plotRect.left() + xRatio * plotRect.width();
        const qreal y = plotRect.bottom() - yRatio * plotRect.height();
        return QPointF(x, y);
    };

    painter.save();
    painter.setClipRect(plotRect);

    if (fieldBounds.isValid() && fieldBounds.width() > 0.0 && fieldBounds.height() > 0.0) {
        const QPointF topLeft = mapPoint(fieldBounds.topLeft());
        const QPointF bottomRight = mapPoint(fieldBounds.bottomRight());
        const QRectF mappedField = QRectF(topLeft, bottomRight).normalized();
        painter.setPen(QPen(palette().dark().color(), 1.25, Qt::DashLine));
        if (m_arenaShape == ArenaShape::Square) {
            painter.drawRect(mappedField);
        } else {
            painter.drawEllipse(mappedField);
        }
    }

    if (hasTracks) {
        struct SpeedSample {
            QPointF pos;
            double speed;
            bool hasSpeed;
        };

        auto buildTurboLut = []() {
            QVector<QColor> lut(256);
            cv::Mat ramp(1, 256, CV_8UC1);
            for (int i = 0; i < 256; ++i) {
                ramp.at<uchar>(0, i) = static_cast<uchar>(i);
            }
            cv::Mat colored;
            cv::applyColorMap(ramp, colored, cv::COLORMAP_TURBO);
            for (int i = 0; i < 256; ++i) {
                const cv::Vec3b bgr = colored.at<cv::Vec3b>(0, i);
                lut[i] = QColor(bgr[2], bgr[1], bgr[0]);
            }
            return lut;
        };

        static const QVector<QColor> turboLut = buildTurboLut();

        const bool speedMode = (m_plotMode == PlotMode::SpeedColor);
        const bool speedEnabled = speedMode && m_videoFps > 0.0;
        const double effectiveFps = speedEnabled ? m_videoFps : 0.0;
        const double unitScale = (m_umPerPixel > 0.0) ? m_umPerPixel : 1.0;
        const int windowFrames = speedEnabled ? std::max(1, static_cast<int>(std::round(2.0 * effectiveFps))) : 0;

        double minSpeed = std::numeric_limits<double>::max();
        double maxSpeed = std::numeric_limits<double>::lowest();

        std::map<int, std::vector<SpeedSample>> speedSamplesByTrack;
        if (speedMode) {
            for (const auto &pair : tracks) {
                const int wormId = pair.first;
                const auto &points = pair.second;
                std::vector<SpeedSample> samples;
                samples.reserve(points.size());

                bool hasPrev = false;
                QPointF prevPos;
                int prevFrame = 0;
                std::deque<std::pair<int, double>> window;
                double windowSum = 0.0;

                for (const auto &point : points) {
                    if (point.quality == Tracking::TrackPointQuality::Lost) {
                        hasPrev = false;
                        window.clear();
                        windowSum = 0.0;
                        continue;
                    }

                    const QPointF pos(point.position.x, point.position.y);
                    if (!hasPrev || !speedEnabled) {
                        samples.push_back({pos, 0.0, false});
                        hasPrev = true;
                        prevPos = pos;
                        prevFrame = point.frameNumberOriginal;
                        continue;
                    }

                    const int frameDelta = point.frameNumberOriginal - prevFrame;
                    if (frameDelta <= 0) {
                        samples.push_back({pos, 0.0, false});
                        prevPos = pos;
                        prevFrame = point.frameNumberOriginal;
                        continue;
                    }

                    const double distPx = QLineF(prevPos, pos).length();
                    const double distUnits = distPx * unitScale;
                    const double dtSeconds = static_cast<double>(frameDelta) / effectiveFps;
                    if (dtSeconds <= 0.0) {
                        samples.push_back({pos, 0.0, false});
                        prevPos = pos;
                        prevFrame = point.frameNumberOriginal;
                        continue;
                    }
                    const double speed = distUnits / dtSeconds;

                    window.emplace_back(point.frameNumberOriginal, speed);
                    windowSum += speed;
                    while (!window.empty()
                           && (point.frameNumberOriginal - window.front().first) > windowFrames) {
                        windowSum -= window.front().second;
                        window.pop_front();
                    }

                    const double avgSpeed = window.empty() ? speed : (windowSum / window.size());
                    samples.push_back({pos, avgSpeed, true});
                    minSpeed = std::min(minSpeed, avgSpeed);
                    maxSpeed = std::max(maxSpeed, avgSpeed);

                    prevPos = pos;
                    prevFrame = point.frameNumberOriginal;
                }
                speedSamplesByTrack[wormId] = std::move(samples);
            }
        }

        if (speedMode && m_speedRangeEnabled && m_speedRangeMax > m_speedRangeMin) {
            minSpeed = m_speedRangeMin;
            maxSpeed = m_speedRangeMax;
        } else if (speedMode && (!speedEnabled || minSpeed >= maxSpeed)) {
            minSpeed = 0.0;
            maxSpeed = std::max(1.0, maxSpeed);
        }

        auto colorForSpeed = [&](double speed) {
            if (maxSpeed <= minSpeed) {
                return palette().highlight().color();
            }
            const double t = std::clamp((speed - minSpeed) / (maxSpeed - minSpeed), 0.0, 1.0);
            const int idx = static_cast<int>(std::round(t * 255.0));
            return turboLut[std::clamp(idx, 0, 255)];
        };

        for (const auto &pair : tracks) {
            const int wormId = pair.first;
            const auto &points = pair.second;
            if (!speedMode) {
                QPainterPath path;
                bool hasPath = false;
                for (const auto &point : points) {
                    if (point.quality == Tracking::TrackPointQuality::Lost) {
                        continue;
                    }
                    const QPointF mapped = mapPoint(QPointF(point.position.x, point.position.y));
                    if (!hasPath) {
                        path.moveTo(mapped);
                        hasPath = true;
                    } else {
                        path.lineTo(mapped);
                    }
                }
                if (hasPath) {
                    QColor color = colorForItem(wormId);
                    if (!color.isValid()) {
                        color = palette().highlight().color();
                    }
                    painter.setPen(QPen(color, 1.5));
                    painter.drawPath(path);
                }
                continue;
            }

            const auto samplesIt = speedSamplesByTrack.find(wormId);
            if (samplesIt == speedSamplesByTrack.end()) {
                continue;
            }
            const auto &samples = samplesIt->second;
            if (samples.size() < 2) {
                continue;
            }

            QColor fallbackColor = colorForItem(wormId);
            if (!fallbackColor.isValid()) {
                fallbackColor = palette().highlight().color();
            }

            for (size_t i = 1; i < samples.size(); ++i) {
                const QPointF mappedPrev = mapPoint(samples[i - 1].pos);
                const QPointF mappedCur = mapPoint(samples[i].pos);
                QColor color = fallbackColor;
                if (samples[i].hasSpeed && speedEnabled) {
                    color = colorForSpeed(samples[i].speed);
                }
                painter.setPen(QPen(color, 1.5));
                painter.drawLine(mappedPrev, mappedCur);
            }
        }

        if (speedMode && !speedEnabled) {
            painter.setPen(palette().text().color());
            painter.drawText(plotRect.adjusted(8, 8, -8, -8),
                             Qt::AlignTop | Qt::AlignLeft,
                             tr("FPS unavailable; speed colors disabled"));
        }
    }

    const qreal pointRadius = 4.0;
    painter.setPen(QPen(palette().shadow().color(), 1.0));
    for (const auto &item : m_items) {
        if (item.type != TableItems::ItemType::ROI
            && item.type != TableItems::ItemType::StartPoint
            && item.type != TableItems::ItemType::EndPoint
            && item.type != TableItems::ItemType::ControlPoint
            && item.type != TableItems::ItemType::CenterPoint) {
            continue;
        }
        const QPointF mapped = mapPoint(item.initialCentroid);
        QColor color = item.color.isValid() ? item.color : palette().highlight().color();
        painter.setBrush(color);
        painter.drawEllipse(mapped, pointRadius, pointRadius);
    }

    painter.restore();
}

void TrackXYPlotWidget::refreshItems(const QList<TableItems::ClickedItem> &items)
{
    m_items = items;
}

QColor TrackXYPlotWidget::colorForItem(int itemId) const
{
    for (const auto &item : m_items) {
        if (item.id == itemId) {
            return item.color;
        }
    }
    return QColor();
}

AnalysisDialog::AnalysisDialog(TrackingDataStorage *storage, QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Analysis"));
    setAttribute(Qt::WA_DeleteOnClose, true);
    resize(640, 480);

    m_tabWidget = new QTabWidget(this);
    m_tracksWidget = new TrackXYPlotWidget(storage, TrackXYPlotWidget::PlotMode::TrackColor, m_tabWidget);
    m_speedTracksWidget = new TrackXYPlotWidget(storage, TrackXYPlotWidget::PlotMode::SpeedColor, m_tabWidget);
    m_tabWidget->addTab(buildSetupTab(), tr("Setup"));
    m_tabWidget->addTab(m_tracksWidget, tr("Tracks XY"));
    m_tabWidget->addTab(m_speedTracksWidget, tr("Tracks XY (Speed)"));

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->addWidget(m_tabWidget);
    setLayout(layout);
}

QWidget *AnalysisDialog::buildSetupTab()
{
    auto *tab = new QWidget(m_tabWidget);
    auto *formLayout = new QFormLayout(tab);
    formLayout->setContentsMargins(12, 12, 12, 12);
    formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_arenaShapeCombo = new QComboBox(tab);
    m_arenaShapeCombo->addItem(tr("Round"));
    m_arenaShapeCombo->addItem(tr("Square"));

    m_arenaSizeSpin = new QSpinBox(tab);
    m_arenaSizeSpin->setRange(1, 1000);
    m_arenaSizeSpin->setValue(50);
    m_arenaSizeSpin->setSuffix(tr(" mm"));

    m_speedRangeCheck = new QCheckBox(tr("Manual"), tab);
    m_speedRangeCheck->setChecked(false);

    m_speedRangeMinSpin = new QDoubleSpinBox(tab);
    m_speedRangeMinSpin->setRange(0.0, 1e6);
    m_speedRangeMinSpin->setDecimals(3);
    m_speedRangeMinSpin->setValue(0.0);

    m_speedRangeMaxSpin = new QDoubleSpinBox(tab);
    m_speedRangeMaxSpin->setRange(0.0, 1e6);
    m_speedRangeMaxSpin->setDecimals(3);
    m_speedRangeMaxSpin->setValue(1.0);

    formLayout->addRow(tr("Arena shape"), m_arenaShapeCombo);
    formLayout->addRow(tr("Size (diameter, width)"), m_arenaSizeSpin);
    formLayout->addRow(tr("Speed LUT range"), m_speedRangeCheck);
    formLayout->addRow(tr("Speed LUT min"), m_speedRangeMinSpin);
    formLayout->addRow(tr("Speed LUT max"), m_speedRangeMaxSpin);

    syncTrackWidgetSetup(m_tracksWidget);
    syncTrackWidgetSetup(m_speedTracksWidget);

    tab->setLayout(formLayout);
    return tab;
}

void AnalysisDialog::setPixelSizeUmPerPixel(double umPerPixel)
{
    if (m_tracksWidget) {
        m_tracksWidget->setPixelSizeUmPerPixel(umPerPixel);
    }
    if (m_speedTracksWidget) {
        m_speedTracksWidget->setPixelSizeUmPerPixel(umPerPixel);
    }
}

void AnalysisDialog::setVideoFps(double fps)
{
    if (m_tracksWidget) {
        m_tracksWidget->setVideoFps(fps);
    }
    if (m_speedTracksWidget) {
        m_speedTracksWidget->setVideoFps(fps);
    }
}

void AnalysisDialog::syncTrackWidgetSetup(TrackXYPlotWidget *widget)
{
    if (!widget) {
        return;
    }
    widget->setArenaShape(m_arenaShapeCombo->currentIndex());
    widget->setArenaSizeMm(m_arenaSizeSpin->value());
    widget->setSpeedLutRange(m_speedRangeCheck->isChecked(),
                             m_speedRangeMinSpin->value(),
                             m_speedRangeMaxSpin->value());
    connect(m_arenaShapeCombo, qOverload<int>(&QComboBox::currentIndexChanged),
            widget, &TrackXYPlotWidget::setArenaShape);
    connect(m_arenaSizeSpin, qOverload<int>(&QSpinBox::valueChanged),
            widget, &TrackXYPlotWidget::setArenaSizeMm);
    connect(m_speedRangeCheck, &QCheckBox::toggled, widget,
            [this, widget](bool enabled) {
                widget->setSpeedLutRange(enabled,
                                         m_speedRangeMinSpin->value(),
                                         m_speedRangeMaxSpin->value());
            });
    connect(m_speedRangeMinSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), widget,
            [this, widget](double value) {
                Q_UNUSED(value)
                widget->setSpeedLutRange(m_speedRangeCheck->isChecked(),
                                         m_speedRangeMinSpin->value(),
                                         m_speedRangeMaxSpin->value());
            });
    connect(m_speedRangeMaxSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), widget,
            [this, widget](double value) {
                Q_UNUSED(value)
                widget->setSpeedLutRange(m_speedRangeCheck->isChecked(),
                                         m_speedRangeMinSpin->value(),
                                         m_speedRangeMaxSpin->value());
            });
}
