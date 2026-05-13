#include "debugutils.h"

// Static member definition
bool DebugUtils::s_trackingDebugEnabled = false;
bool DebugUtils::s_debugCaptureEnabled = false;

bool DebugUtils::isTrackingDebugEnabled() {
    return s_trackingDebugEnabled;
}

bool DebugUtils::isDebugCaptureEnabled() {
    return s_debugCaptureEnabled;
}

void DebugUtils::setTrackingDebugEnabled(bool enabled) {
    s_trackingDebugEnabled = enabled;
}

void DebugUtils::setDebugCaptureEnabled(bool enabled) {
    s_debugCaptureEnabled = enabled;
}

QDebug DebugUtils::trackingDebug() {
    return qDebug();
}
