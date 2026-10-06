#include "yawtpaths.h"

#include <QDir>
#include <QCoreApplication>
#include <QStandardPaths>
#include <QFileInfo>
#include <QDebug>

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

QString YawtPaths::ensureVideoDataDirectory(const QString& videoFilePath) {
    QFileInfo videoInfo(videoFilePath);
    QString videoDirectory = videoInfo.absolutePath();
    QString dataDirPath = QDir(videoDirectory).absoluteFilePath("yawt");

    // Try to create the directory in the same folder as the video
    QDir dataDir(dataDirPath);
    if (!dataDir.exists()) {
        if (QDir().mkpath(dataDirPath)) {
            qDebug() << "Created data directory:" << dataDirPath;
            return dataDirPath;
        } else {
            qWarning() << "Failed to create data directory in video folder:" << dataDirPath;
            qWarning() << "Falling back to user's home directory";

            // Fallback to user's home directory
            QString homeDirectory = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
            QString fallbackDataDir = QDir(homeDirectory).absoluteFilePath("yawt");

            QDir fallbackDir(fallbackDataDir);
            if (!fallbackDir.exists()) {
                if (QDir().mkpath(fallbackDataDir)) {
                    qDebug() << "Created data directory in home:" << fallbackDataDir;
                    return fallbackDataDir;
                } else {
                    qWarning() << "Failed to create data directory in home folder:" << fallbackDataDir;
                    return QString(); // Return empty string if all attempts fail
                }
            } else {
                qDebug() << "Using existing data directory in home:" << fallbackDataDir;
                return fallbackDataDir;
            }
        }
    } else {
        qDebug() << "Using existing data directory:" << dataDirPath;
        return dataDirPath;
    }
}
