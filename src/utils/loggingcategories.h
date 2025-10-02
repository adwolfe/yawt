#pragma once
// yawt/src/utils/loggingcategories.h
//
// Centralized logging categories and verbosity helpers for YAWT.
// Usage:
//   1) In exactly one .cpp (e.g., main.cpp) define LOGGING_CATEGORIES_DEFINE
//      BEFORE including this header to create the category definitions:
//          #define LOGGING_CATEGORIES_DEFINE
//          #include "utils/loggingcategories.h"
//   2) In all other .cpp files, simply:
//          #include "utils/loggingcategories.h"
//   3) At app startup, call Yawt::Logging::installDefaultMessagePattern();
//      and Yawt::Logging::applyVerbosity(<1..3>);
//      Verbosity aliases: 1 = basic, 2 = intermediate, 3 = firehose (all).
//
// Notes:
// - This header enables fine-grained filtering (by category & level) and a unified
//   message format that includes time, level, category, file, and line.
// - Prefer qCDebug/qCInfo/qCWarning/qCCritical with categories (see macros below).
// - You can also inject rules via QT_LOGGING_RULES env var; rules set here take effect
//   when applyVerbosity(...) is called.

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

// Verbosity levels: 1 = least (basic), 2 = intermediate (actions), 3 = most (firehose)
enum class Verbosity : int {
    Basic       = 1, // status updates, high-level flow; debug mostly off
    Normal      = 2, // include action-level debug (state transitions, split/merge decisions)
    Firehose    = 3  // include per-frame debug and all messages from all categories
};

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

// Build filter rules string for a given verbosity.
// We keep warnings/criticals enabled always. We control info/debug granularity here.
//
// Category groups (prefixes):
//  - yawt.core.*          Core orchestration and trackers
//  - yawt.data.*          Central storage/data structures
//  - yawt.gui.*           UI shell, dialogs, widgets
//  - yawt.models.*        Qt models feeding views
//  - yawt.utils.*         Utilities and CV helpers
inline QString filterRulesForVerbosity(Verbosity v)
{
    // Defaults: enable warnings/criticals; disable debug globally unless overridden.
    QString rules =
        "*.warning=true\n"
        "*.critical=true\n"
        "*.debug=false\n"
        "*.info=false\n";

    switch (v) {
    case Verbosity::Basic:
        // Enable only high-level info on core components; keep debug off.
        rules +=
        "yawt.core.*.info=true\n"
        "yawt.data.*.info=true\n";
        break;
    case Verbosity::Normal:
        // Enable info for most and debug for core components (but keep GUI debug off).
        rules +=
        "*.info=true\n"
        "yawt.core.*.debug=true\n"
        "yawt.data.*.debug=true\n"
        "yawt.models.*.debug=true\n"
        "yawt.utils.*.debug=true\n";
        break;
    case Verbosity::Firehose:
        // Everything on: info + debug everywhere (per-frame messages will flow).
        rules +=
        "*.info=true\n"
        "*.debug=true\n";
        break;
    }
    return rules;
}

// Apply verbosity filtering (1..3). Values outside range are clamped.
inline void applyVerbosity(int level)
{
    if (level < 1) level = 1;
    if (level > 3) level = 3;
    const Verbosity v = static_cast<Verbosity>(level);
    QLoggingCategory::setFilterRules(filterRulesForVerbosity(v));
}

// Friendly alias for --firehose -> verbosity=3
inline void applyFirehose() { applyVerbosity(3); }

// Parse a "--verbosity=N" or "--firehose" pair from a pre-parsed string list and apply.
// This is optional sugar if you don't use QCommandLineParser in main yet.
inline void applyVerbosityFromArgs(const QStringList& args)
{
    // --firehose alias
    if (args.contains("--firehose")) {
        applyFirehose();
        return;
    }

    // --verbosity=<n>
    for (const QString& a : args) {
        if (a.startsWith("--verbosity=")) {
            bool ok = false;
            const int lvl = a.section('=', 1, 1).toInt(&ok);
            if (ok) { applyVerbosity(lvl); }
            return;
        }
    }

    // Default if unspecified
    applyVerbosity(1);
}

} // namespace Logging
} // namespace Yawt
