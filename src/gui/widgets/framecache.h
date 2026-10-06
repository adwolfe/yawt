#pragma once

#include <QDateTime>
#include <QMutex>
#include <QMap>
#include <QAtomicInt>
#include <opencv2/core.hpp>

// Cached frame structure
struct CachedFrame {
    int frameNumber;
    cv::Mat rawFrame;
    cv::Mat thresholdedFrame;
    QDateTime lastAccessed;
    bool isValid;

    CachedFrame() : frameNumber(-1), isValid(false) {}
    CachedFrame(int fn, const cv::Mat& raw)
        : frameNumber(fn), rawFrame(raw), isValid(true), lastAccessed(QDateTime::currentDateTime()) {}
};

// Thread-safe LRU frame cache
class FrameCache {
public:
    explicit FrameCache(int maxCacheSize = 50);
    ~FrameCache();

    // Cache operations
    void insertFrame(int frameNumber, const cv::Mat& frame);
    bool getFrame(int frameNumber, cv::Mat& outFrame);
    bool hasFrame(int frameNumber) const;
    void clear();
    void setMaxSize(int maxSize);

    // Statistics
    int size() const;
    int maxSize() const;
    double hitRate() const;

private:
    void evictLRU();
    void updateAccessTime(int frameNumber);

    mutable QMutex m_mutex;
    QMap<int, CachedFrame> m_frames;
    int m_maxSize;
    mutable QAtomicInt m_hits;
    mutable QAtomicInt m_requests;
};

