#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#include <QDebug>

/**
 * @brief Global debug utilities for controlling verbose debug output
 * 
 * This utility provides a centralized way to control debug output across
 * different components, particularly for performance-sensitive tracking operations.
 */
class DebugUtils {
public:
    /**
     * @brief Check if tracking debug messages should be displayed
     * @return True if tracking debug is enabled
     */
    static bool isTrackingDebugEnabled();
    
    /**
     * @brief Set the tracking debug state
     * @param enabled True to enable tracking debug messages
     */
    static void setTrackingDebugEnabled(bool enabled);
    
    /**
     * @brief Convenience macro for conditional tracking debug output
     * Usage: TRACKING_DEBUG() << "Your debug message";
     */
    static QDebug trackingDebug();
    
private:
    static bool s_trackingDebugEnabled;
};

// Convenience macro for tracking debug messages
#define TRACKING_DEBUG() \
    if (!DebugUtils::isTrackingDebugEnabled()) {} else DebugUtils::trackingDebug()

#endif // DEBUGUTILS_H