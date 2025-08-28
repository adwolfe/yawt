#ifndef CACHESTATUSWIDGET_H
#define CACHESTATUSWIDGET_H

#include <QWidget>
#include <QSet>
#include <QMutex>
#include <QMutexLocker>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QSizePolicy>
#include <QEvent>
#include <QMutableSetIterator>
#include <QVector>
#include <QColor>
#include <QToolTip>
#include <algorithm>

/*
 * CacheStatusWidget
 *
 * Small status-bar widget that visualizes which frames are currently cached.
 * - The widget maps the video's frame range [0, totalFrames-1] to the widget width.
 * - Each horizontal pixel column represents one or more frames. If any frame
 *   in the corresponding sub-range is cached, that column is drawn in the
 *   "cached" color; otherwise the "uncached" color is used.
 *
 * API / Slots:
 *  - void setTotalFrames(int totalFrames)          // set or update total frame count
 *  - void onFrameCached(int frameNumber)           // mark a frame as cached
 *  - void onFrameEvicted(int frameNumber)          // mark a frame as evicted/removed
 *  - void setCachedFrames(const QSet<int>& frames) // replace full cache set
 *  - void clear()                                  // clear cache visualization
 *  - void setCurrentFrame(int frame)               // optional: highlight current frame
 *
 * Signals:
 *  - void frameClicked(int frameNumber)            // emitted when user clicks in widget
 *
 * Thread-safety:
 *  - Slots lock an internal mutex so they may be called from other threads
 *    (though typical Qt GUI threading rules still apply when connecting).
 *
 * Integration notes:
 *  - Connect VideoLoader::frameCached(int) -> CacheStatusWidget::onFrameCached(int)
 *  - VideoLoader should emit a complementary signal when a frame is removed/evicted
 *    from its cache (if available), or MainWindow/VideoLoader can call
 *    CacheStatusWidget::onFrameEvicted(int) when they explicitly remove frames.
 *  - MainWindow can add an instance of this widget to its statusBar() (e.g.,
 *    statusBar()->addPermanentWidget(cacheStatusWidget, 0);
 *
 * Painting behavior:
 *  - When total frame count is 0 or unknown, the widget draws a disabled bar.
 *  - A thin border is drawn and optionally the current frame is highlighted.
 *
 */

class CacheStatusWidget : public QWidget {
    Q_OBJECT
public:
    explicit CacheStatusWidget(QWidget* parent = nullptr)
        : QWidget(parent),
          m_totalFrames(0),
          m_currentFrame(-1),
          m_cachedColor(0x4CAF50),       // Material green
          m_uncachedColor(0x212121),     // Dark gray for unused
          m_borderColor(Qt::black),
          m_currentFrameColor(Qt::yellow)
    {
        setMinimumHeight(12);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        setToolTip("Frame cache status");
    }

    QSize sizeHint() const {
        return QSize(200, 14);
    }

public slots:
    // Set the total number of frames in the video (used to map positions)
    void setTotalFrames(int totalFrames) {
        QMutexLocker locker(&m_mutex);
        if (totalFrames < 0) totalFrames = 0;
        if (m_totalFrames == totalFrames) return;
        m_totalFrames = totalFrames;
        // Remove any cached frame indices that are now out of range
        if (m_totalFrames > 0) {
            QMutableSetIterator<int> it(m_cachedFrames);
            while (it.hasNext()) {
                int f = it.next();
                if (f < 0 || f >= m_totalFrames) it.remove();
            }
        } else {
            m_cachedFrames.clear();
        }
        locker.unlock();
        update();
    }

    // Mark a frame as cached
    void onFrameCached(int frameNumber) {
        QMutexLocker locker(&m_mutex);
        if (frameNumber < 0) return;
        if (m_totalFrames > 0 && frameNumber >= m_totalFrames) return;
        m_cachedFrames.insert(frameNumber);
        locker.unlock();
        update(); // repaint to show newly cached frame
    }

    // Mark a frame as evicted/removed from cache
    void onFrameEvicted(int frameNumber) {
        QMutexLocker locker(&m_mutex);
        if (m_cachedFrames.remove(frameNumber) > 0) {
            locker.unlock();
            update();
        }
    }

    // Replace the entire cached-set (useful if caller computes full set)
    void setCachedFrames(const QSet<int>& frames) {
        QMutexLocker locker(&m_mutex);
        m_cachedFrames.clear();
        if (m_totalFrames > 0) {
            for (int f : frames) {
                if (f >= 0 && f < m_totalFrames) m_cachedFrames.insert(f);
            }
        } else {
            // If total unknown, accept non-negative indices
            for (int f : frames) if (f >= 0) m_cachedFrames.insert(f);
        }
        locker.unlock();
        update();
    }

    // Clear visualization entirely
    void clear() {
        QMutexLocker locker(&m_mutex);
        m_cachedFrames.clear();
        m_currentFrame = -1;
        locker.unlock();
        update();
    }

    // Optional: highlight a current frame position
    void setCurrentFrame(int frame) {
        QMutexLocker locker(&m_mutex);
        if (m_currentFrame == frame) return;
        m_currentFrame = frame;
        locker.unlock();
        update();
    }

    // Colors can be customized
    void setCachedColor(const QColor& c) { m_cachedColor = c; update(); }
    void setUncachedColor(const QColor& c) { m_uncachedColor = c; update(); }
    void setBorderColor(const QColor& c) { m_borderColor = c; update(); }
    void setCurrentFrameColor(const QColor& c) { m_currentFrameColor = c; update(); }

signals:
    void frameClicked(int frameNumber);

protected:
    void paintEvent(QPaintEvent* event) {
        Q_UNUSED(event);
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        QRect r = rect().adjusted(1, 1, -1, -1); // leave space for border
        p.fillRect(rect(), palette().color(QPalette::Window));

        // Draw background bar
        p.setPen(Qt::NoPen);
        p.setBrush(m_uncachedColor);
        p.drawRect(r);

        // Quick exit if nothing to show
        QMutexLocker locker(&m_mutex);
        int totalFrames = m_totalFrames;
        if (totalFrames <= 0 || r.width() <= 0) {
            // Draw border and return
            p.setPen(m_borderColor);
            p.setBrush(Qt::NoBrush);
            p.drawRect(rect().adjusted(0, 0, -1, -1));
            return;
        }

        // Prepare sorted vector of cached frames for efficient queries
        QVector<int> cached;
        cached.reserve(m_cachedFrames.size());
        for (int f : m_cachedFrames) cached.append(f);
        std::sort(cached.begin(), cached.end());

        // For each pixel column, determine whether any frame in the corresponding range is cached.
        int w = r.width();
        int h = r.height();
        int rx = r.x();
        int ry = r.y();
        for (int x = 0; x < w; ++x) {
            // map pixel column x -> frameRange [sf, ef]
            // use integer arithmetic; ensure coverage of all frames
            int sf = static_cast<long long>(x) * totalFrames / w;
            int ef = static_cast<long long>(x + 1) * totalFrames / w - 1;
            if (ef < sf) ef = sf;
            // binary search: find first cached >= sf
            bool anyCached = false;
            if (!cached.isEmpty()) {
                auto it = std::lower_bound(cached.constBegin(), cached.constEnd(), sf);
                if (it != cached.constEnd() && *it <= ef) anyCached = true;
            }
            if (anyCached) {
                p.setPen(Qt::NoPen);
                p.setBrush(m_cachedColor);
            } else {
                p.setPen(Qt::NoPen);
                p.setBrush(m_uncachedColor);
            }
            p.drawRect(rx + x, ry, 1, h);
        }

        // Draw current frame indicator (thin vertical line)
        if (m_currentFrame >= 0 && m_currentFrame < totalFrames) {
            // compute x coordinate for the frame
            int cx = rx + (static_cast<long long>(m_currentFrame) * w) / totalFrames;
            p.setPen(QPen(m_currentFrameColor, 1));
            p.drawLine(cx, ry, cx, ry + h - 1);
        }

        // Border
        p.setPen(m_borderColor);
        p.setBrush(Qt::NoBrush);
        p.drawRect(rect().adjusted(0, 0, -1, -1));
        locker.unlock();
    }

    void mousePressEvent(QMouseEvent* ev) {
        if (ev->button() != Qt::LeftButton) {
            QWidget::mousePressEvent(ev);
            return;
        }
        QMutexLocker locker(&m_mutex);
        int totalFrames = m_totalFrames;
        if (totalFrames <= 0 || width() <= 0) {
            QWidget::mousePressEvent(ev);
            return;
        }
        int x = ev->pos().x();
        // map x to frame
        int w = rect().width() - 2; // consider border space similar to paintEvent
        if (w <= 0) w = rect().width();
        int frame = static_cast<long long>(x) * totalFrames / qMax(1, rect().width());
        if (frame < 0) frame = 0;
        if (frame >= totalFrames) frame = totalFrames - 1;
        locker.unlock();
        emit frameClicked(frame);
    }

    void enterEvent(QEvent* ev) {
        Q_UNUSED(ev);
        setCursor(Qt::PointingHandCursor);
    }

    void leaveEvent(QEvent* ev) {
        Q_UNUSED(ev);
        unsetCursor();
    }

private:
    mutable QMutex m_mutex;
    QSet<int> m_cachedFrames;
    int m_totalFrames;
    int m_currentFrame;

    QColor m_cachedColor;
    QColor m_uncachedColor;
    QColor m_borderColor;
    QColor m_currentFrameColor;
};

#endif // CACHESTATUSWIDGET_H