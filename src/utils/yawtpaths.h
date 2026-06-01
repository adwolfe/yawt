#pragma once

#include <QString>
#include <QStringList>

/**
 * YawtPaths — canonical filesystem paths for user-level and per-project YAWT data.
 *
 * User-level dir (macOS): ~/Library/Application Support/yawt/
 * Per-project dir:        <yawtProjectDir>/plugins/
 *
 * All ensure* helpers create the directory if it does not exist and return
 * true on success. Callers should check the return value before writing files.
 */
class YawtPaths
{
public:
    YawtPaths() = delete;

    /** Root of the user-level YAWT application data directory. */
    static QString userDataDir();

    /** Directory for user-level plot plugins. */
    static QString userPluginDir();

    /** Directory for plot plugins scoped to a specific yawt project. */
    static QString projectPluginDir(const QString& yawtProjectDir);

    /**
     * All plugin directories, in search order: project-level first (higher priority),
     * then user-level. Directories that do not yet exist are still included so callers
     * can display them even before the first plugin is installed.
     */
    static QStringList pluginSearchDirs(const QString& yawtProjectDir = {});

    /** Create the user data and user plugin directories. Returns true if both exist. */
    static bool ensureUserDirsExist();

    /** Create the project plugin directory. Returns true if it exists afterwards. */
    static bool ensureProjectPluginDir(const QString& yawtProjectDir);

    /**
     * Plugins bundled inside the macOS .app bundle (Contents/Resources/plugins/).
     * Returns an empty string on non-bundle builds or non-macOS platforms.
     * These are read-only; users cannot write here.
     */
    static QString bundledPluginDir();
};
