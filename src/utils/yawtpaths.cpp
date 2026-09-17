#include "yawtpaths.h"

#include <QDir>
#include <QCoreApplication>
#include <QStandardPaths>

QString YawtPaths::userDataDir()
{
    // QStandardPaths::AppDataLocation → ~/Library/Application Support/yawt on macOS
    const QString base = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    return base;
}

QString YawtPaths::userPluginDir()
{
    return QDir(userDataDir()).filePath("plugins");
}

QString YawtPaths::projectPluginDir(const QString& dataDir)
{
    if (dataDir.isEmpty()) return {};
    return QDir(dataDir).filePath("plugins");
}

QString YawtPaths::bundledPluginDir()
{
#ifdef Q_OS_MACOS
    // On macOS, applicationDirPath() is <App>.app/Contents/MacOS/
    // Resources live one level up at Contents/Resources/
    const QString resourcesDir =
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath("../Resources");
    const QString bundledDir = QDir(resourcesDir).absoluteFilePath("plugins");
    if (QDir(bundledDir).exists())
        return QDir::cleanPath(bundledDir);
#endif
    return {};
}

QStringList YawtPaths::pluginSearchDirs(const QString& dataDir)
{
    QStringList dirs;
    // Priority: project-level > user-level > bundled (read-only examples)
    if (!dataDir.isEmpty())
        dirs << projectPluginDir(dataDir);
    dirs << userPluginDir();
    const QString bundled = bundledPluginDir();
    if (!bundled.isEmpty())
        dirs << bundled;
    return dirs;
}

bool YawtPaths::ensureUserDirsExist()
{
    return QDir().mkpath(userDataDir()) && QDir().mkpath(userPluginDir());
}

bool YawtPaths::ensureProjectPluginDir(const QString& dataDir)
{
    if (dataDir.isEmpty()) return false;
    return QDir().mkpath(projectPluginDir(dataDir));
}
