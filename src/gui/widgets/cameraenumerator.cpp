#include "cameraenumerator.h"
#include "opencvcamerasource.h"

#ifdef YAWT_HAS_PYLON
#  include "pyloncamerasource.h"
#  include <pylon/PylonIncludes.h>
#endif

#include <opencv2/videoio.hpp>
#include <QDebug>

QList<CameraInfo> CameraEnumerator::enumerate()
{
    QList<CameraInfo> results;

    // -----------------------------------------------------------------------
    // OpenCV: probe indices 0-4
    // -----------------------------------------------------------------------
    for (int i = 0; i < 5; ++i) {
        cv::VideoCapture probe;
        if (probe.open(i)) {
            CameraInfo info;
            info.backend     = QStringLiteral("opencv");
            info.id          = QString::number(i);
            info.displayName = QString("Camera %1 (OpenCV)").arg(i);
            results.append(info);
            probe.release();
        }
    }

#ifdef YAWT_HAS_PYLON
    // -----------------------------------------------------------------------
    // Pylon: ask the transport-layer factory for all visible Basler devices
    // -----------------------------------------------------------------------
    try {
        Pylon::PylonInitialize(); // safe to call multiple times; Pylon ref-counts

        Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();
        Pylon::DeviceInfoList_t devices;
        tlFactory.EnumerateDevices(devices);

        for (const auto& dev : devices) {
            CameraInfo info;
            info.backend = QStringLiteral("pylon");
            // Use the serial number as a stable unique ID
            info.id = QString::fromStdString(
                std::string(dev.GetSerialNumber()));
            // Display name: "Basler acA1920-40uc (SN:12345678)" or similar
            QString model  = QString::fromStdString(std::string(dev.GetModelName()));
            QString serial = QString::fromStdString(std::string(dev.GetSerialNumber()));
            QString user   = QString::fromStdString(std::string(dev.GetUserDefinedName()));
            info.displayName = user.isEmpty()
                ? QString("%1 [%2] (Pylon)").arg(model, serial)
                : QString("%1 — %2 [%3] (Pylon)").arg(user, model, serial);
            results.append(info);
        }
    } catch (const Pylon::GenericException& e) {
        qWarning() << "[Pylon] Enumeration failed:" << e.GetDescription();
    }
#endif

    return results;
}

std::unique_ptr<ICameraSource> CameraEnumerator::create(const CameraInfo& info)
{
    if (info.backend == QStringLiteral("opencv")) {
        bool ok = false;
        int index = info.id.toInt(&ok);
        if (!ok) index = 0;
        return std::make_unique<OpenCVCameraSource>(index, info.displayName);
    }

#ifdef YAWT_HAS_PYLON
    if (info.backend == QStringLiteral("pylon")) {
        try {
            Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();
            Pylon::DeviceInfoList_t devices;
            tlFactory.EnumerateDevices(devices);

            for (const auto& dev : devices) {
                if (std::string(dev.GetSerialNumber()) == info.id.toStdString()) {
                    return std::make_unique<PylonCameraSource>(dev, info.displayName);
                }
            }
            qWarning() << "[Pylon] Device with serial" << info.id << "not found.";
        } catch (const Pylon::GenericException& e) {
            qWarning() << "[Pylon] create() failed:" << e.GetDescription();
        }
    }
#endif

    return nullptr;
}
