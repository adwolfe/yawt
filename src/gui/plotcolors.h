#pragma once

#include <QColor>
#include <algorithm>
#include <cmath>

// Inferno colormap (perceptually uniform, matplotlib) with slight desaturation
// for chart readability. t = 0 (dark purple-black) → 1 (pale cream-yellow).
inline QColor infernoColor(double t)
{
    t = std::max(0.0, std::min(1.0, t));

    static const struct { double t; int r, g, b; } lut[] = {
        {0.00,   0,   0,   4},
        {0.10,  22,  11,  48},
        {0.20,  66,  10, 104},
        {0.30, 107,  19,  99},
        {0.40, 148,  37,  73},
        {0.50, 188,  55,  35},
        {0.60, 221,  81,   5},
        {0.70, 245, 127,   6},
        {0.80, 250, 185,  49},
        {0.90, 252, 231, 134},
        {1.00, 252, 255, 164},
    };
    constexpr int n = static_cast<int>(sizeof(lut) / sizeof(lut[0]));

    int lo = 0;
    for (int i = 0; i < n - 2; ++i) {
        if (t < lut[i + 1].t) { lo = i; break; }
        lo = i + 1;
    }
    const int hi = std::min(lo + 1, n - 1);
    const double span = lut[hi].t - lut[lo].t;
    const double f = span > 0.0 ? (t - lut[lo].t) / span : 0.0;

    const int r = static_cast<int>(lut[lo].r + f * (lut[hi].r - lut[lo].r) + 0.5);
    const int g = static_cast<int>(lut[lo].g + f * (lut[hi].g - lut[lo].g) + 0.5);
    const int b = static_cast<int>(lut[lo].b + f * (lut[hi].b - lut[lo].b) + 0.5);

    QColor col(r, g, b);
    float h, s, v, a;
    col.getHsvF(&h, &s, &v, &a);
    col.setHsvF(h, s * 0.82f, v, a);
    return col;
}

// Map index i of n total items to an Inferno color.
// Avoids the near-black and near-cream extremes of the LUT.
inline QColor plotColor(int i, int n)
{
    if (n <= 1) return infernoColor(0.50);
    return infernoColor(0.12 + 0.72 * static_cast<double>(i) / (n - 1));
}
