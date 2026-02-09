#include "debugutils.h"

// Static member definition
bool DebugUtils::s_trackingDebugEnabled = true;

bool DebugUtils::isTrackingDebugEnabled() {
    return s_trackingDebugEnabled;
}

void DebugUtils::setTrackingDebugEnabled(bool enabled) {
    s_trackingDebugEnabled = enabled;
}

QDebug DebugUtils::trackingDebug() {
    return qDebug();
}
