#pragma once
// yawt/src/utils/loggingcategories.h
//
// Centralized logging categories and debug-mode helpers for YAWT.
// Usage:
//   1) In exactly one .cpp (e.g., main.cpp) define LOGGING_CATEGORIES_DEFINE
//      BEFORE including this header to create the category definitions:
//          #define LOGGING_CATEGORIES_DEFINE
//          #include "utils/loggingcategories.h"
//   2) In all other .cpp files, simply:
//          #include "utils/loggingcategories.h"
//   3) At app startup, call Yawt::Logging::installDefaultMessagePattern();
//      and Yawt::Logging::applyDebugMode(<bool>).
//
// Notes:
// - This header enables fine-grained filtering (by category & level) and a unified
//   message format that includes time, level, category, file, and line.
// - Prefer qCDebug/qCInfo/qCWarning/qCCritical with categories (see macros below).
// - You can also inject rules via QT_LOGGING_RULES env var; rules set here take effect
//   when applyDebugMode(...) is called.

#include <QLoggingCategory>
#include <QtGlobal>
#include <QString>

//
// Declare categories
//
Q_DECLARE_LOGGING_CATEGORY(lcCoreTrackingManager)
Q_DECLARE_LOGGING_CATEGORY(lcCoreWormTracker)
Q_DECLARE_LOGGING_CATEGORY(lcCoreAppController)
Q_DECLARE_LOGGING_CATEGORY(lcCoreVideoProcessor)
Q_DECLARE_LOGGING_CATEGORY(lcCoreCenterlineWorker)

Q_DECLARE_LOGGING_CATEGORY(lcDataStorage)
Q_DECLARE_LOGGING_CATEGORY(lcDataCommon)

Q_DECLARE_LOGGING_CATEGORY(lcGuiMainWindow)
Q_DECLARE_LOGGING_CATEGORY(lcGuiTrackingDialog)
Q_DECLARE_LOGGING_CATEGORY(lcGuiVideoLoader)
Q_DECLARE_LOGGING_CATEGORY(lcGuiMiniLoader)

Q_DECLARE_LOGGING_CATEGORY(lcModelsBlobTable)
Q_DECLARE_LOGGING_CATEGORY(lcModelsAnnotationTable)

Q_DECLARE_LOGGING_CATEGORY(lcUtilsThresholding)
Q_DECLARE_LOGGING_CATEGORY(lcUtilsLogging)

//
// Define categories in exactly ONE translation unit by defining LOGGING_CATEGORIES_DEFINE
//
#ifdef LOGGING_CATEGORIES_DEFINE
Q_LOGGING_CATEGORY(lcCoreTrackingManager,    "yawt.core.trackingmanager")
Q_LOGGING_CATEGORY(lcCoreWormTracker,        "yawt.core.wormtracker")
Q_LOGGING_CATEGORY(lcCoreAppController,      "yawt.core.appcontroller")
Q_LOGGING_CATEGORY(lcCoreVideoProcessor,     "yawt.core.processing.videoprocessor")
Q_LOGGING_CATEGORY(lcCoreCenterlineWorker,   "yawt.core.centerlineworker")

Q_LOGGING_CATEGORY(lcDataStorage,            "yawt.data.storage")
Q_LOGGING_CATEGORY(lcDataCommon,             "yawt.data.common")

Q_LOGGING_CATEGORY(lcGuiMainWindow,          "yawt.gui.mainwindow")
Q_LOGGING_CATEGORY(lcGuiTrackingDialog,      "yawt.gui.trackingdialog")
Q_LOGGING_CATEGORY(lcGuiVideoLoader,         "yawt.gui.videoloader")
Q_LOGGING_CATEGORY(lcGuiMiniLoader,          "yawt.gui.miniloader")

Q_LOGGING_CATEGORY(lcModelsBlobTable,        "yawt.models.blobtable")
Q_LOGGING_CATEGORY(lcModelsAnnotationTable,  "yawt.models.annotationtable")

Q_LOGGING_CATEGORY(lcUtilsThresholding,      "yawt.utils.thresholding")
Q_LOGGING_CATEGORY(lcUtilsLogging,           "yawt.utils.logging")
#endif

//
// Convenience macros for consistent logging style (with .noquote() commonly desired in this codebase)
//
#define YAWT_DEBUG(cat)     qCDebug(cat).noquote()
#define YAWT_INFO(cat)      qCInfo(cat).noquote()
#define YAWT_WARN(cat)      qCWarning(cat).noquote()
#define YAWT_CRIT(cat)      qCCritical(cat).noquote()

namespace Yawt {
namespace Logging {

// Install a readable global message pattern that includes time, level, category, file, and line.
inline void installDefaultMessagePattern(bool withThreadId = true, bool withTime = true)
{
    // Example output:
    // 12:34:56.789 D yawt.core.trackingmanager [0x70000abc] trackingmanager.cpp:123 - Message text
    const char* timePart   = withTime ? "%{time hh:mm:ss.zzz} " : "";
    const char* threadPart = withThreadId ? "[%{threadid}] " : "";
    // Map Qt types to single letters: D/I/W/C
    // %{if-debug}D%{endif}%{if-info}I%{endif}%{if-warning}W%{endif}%{if-critical}C%{endif}
    const QString pattern = QString::fromLatin1(
        "%1%{if-debug}D%{endif}%{if-info}I%{endif}%{if-warning}W%{endif}%{if-critical}C%{endif} "
        "%{category} %2%{file}:%{line} - %{message}")
        .arg(timePart)
        .arg(threadPart);
    qSetMessagePattern(pattern);
}

// Build filter rules string for the single debug on/off mode.
// We keep warnings/criticals enabled always. We control info/debug granularity here.
//
// Category groups (prefixes):
//  - yawt.core.*          Core orchestration and trackers
//  - yawt.data.*          Central storage/data structures
//  - yawt.gui.*           UI shell, dialogs, widgets
//  - yawt.models.*        Qt models feeding views
//  - yawt.utils.*         Utilities and CV helpers
inline QString filterRulesForDebugMode(bool enabled)
{
    QString rules =
        "*.warning=true\n"
        "*.critical=true\n"
        "*.debug=false\n"
        "*.info=false\n";

    if (enabled) {
        rules +=
        "yawt.*.info=true\n"
        "yawt.*.debug=true\n"
        "qt.*.info=false\n"
        "qt.*.debug=false\n";
    } else {
        rules +=
        "yawt.*.info=true\n"
        "yawt.*.debug=false\n"
        "qt.*.info=false\n"
        "qt.*.debug=false\n";
    }
    return rules;
}

inline void applyDebugMode(bool enabled)
{
    QLoggingCategory::setFilterRules(filterRulesForDebugMode(enabled));
}

} // namespace Logging
} // namespace Yawt
