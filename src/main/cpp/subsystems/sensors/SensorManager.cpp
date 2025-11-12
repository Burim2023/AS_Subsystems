#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/IRRangeSubsystem.h"
#include "subsystems/sensor/LidarSubsystem.h"
#include "subsystems/sensor/LineFollower.h"
#include "web-ds-logger/cpp/networktables/LoggingSystem.h"
#include "subsystems/sensor/SensorManager.h"

#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>

// uncomment if lidar keeps crashing
// #define DISABLE_LIDAR

// thread stopper
std::atomic<bool> SensorManager::EnableSensorThread{false};

SensorManager::SensorManager()
{
    stopThread = false;

    ultraSonic = std::make_unique<frc::UltrasonicSubsystem>(0, 1, 2, 3);
    infraRed = std::make_unique<frc::IRRangeSubsystem>(0, 1);
    lineFollower = std::make_unique<LineFollower>(0, 1, 2, 3, 5.0f);

#ifndef DISABLE_LIDAR
    lidar = std::make_unique<frc::LidarSubsystem>(studica::Lidar::kUSB1);
#else
    std::cerr << "WARNING: LiDAR is DISABLED at compile time" << std::endl;
#endif
}
SensorManager::~SensorManager()
{
    stopThread = true;

    if (workerThread.joinable())
        workerThread.join();
}

void SensorManager::SensorWorker()
{
    while (!stopThread.load())
    {
        try
        {
            std::lock_guard<std::mutex> lock(m_sensorMutex);

            if (lidar)
            {
                lidar->UpdateLidar();
            }
            if (ultraSonic)
            {
                ultraSonic->UpdateUltraSonic();
            }
            if (infraRed)
            {
                infraRed->UpdateInfraRed();
            }
            if (lineFollower)
            {
                lineFollower->update();
            }
        }
        catch (const std::exception &e)
        {
            // Catch any exception to prevent thread termination
            static int errorCount = 0;
            if (++errorCount % 100 == 0) // Log every 100th error to avoid spam
            {
                std::cerr << "SensorWorker exception: " << e.what() << " (count=" << errorCount << ")\n";
            }
        }
        catch (...)
        {
            // Catch-all for non-standard exceptions
            static int unknownErrorCount = 0;
            if (++unknownErrorCount % 100 == 0)
            {
                std::cerr << "SensorWorker unknown exception (count=" << unknownErrorCount << ")\n";
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(Constants::SENSOR_UPDATE_RATE));
    }
}

void SensorManager::InitializeSensors()
{
    if (ultraSonic)
    {
        ultraSonic->Init();
    }
    if (infraRed)
    {
        infraRed->Init();
    }
#ifndef DISABLE_LIDAR
    if (lidar)
    {
        try
        {
            lidar->Init();
        }
        catch (const std::exception &e)
        {
            std::cerr << "SensorManager: LiDAR init failed, continuing without it: "
                      << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "SensorManager: LiDAR init failed with unknown error" << std::endl;
        }
    }
#endif
}

void SensorManager::SensorManagerStartThread()
{
    if (workerThread.joinable())
    {
        return;
    }

    stopThread = false;
    workerThread = std::thread(&SensorManager::SensorWorker, this);
    LOG_THREAD("Sensor Thread initialized.");
}

void SensorManager::SensorManagerStopThread()
{
    if (!workerThread.joinable())
    {
        return;
    }
    stopThread = true;

    if (workerThread.joinable())
    {
        workerThread.join();
    }
}

frc::UltrasonicSubsystem *SensorManager::GetUltrasonicSubsystem()
{
    std::lock_guard<std::mutex> lock(m_sensorMutex);
    return ultraSonic.get();
}

frc::IRRangeSubsystem *SensorManager::GetIRRangeSubsystem()
{
    std::lock_guard<std::mutex> lock(m_sensorMutex);
    return infraRed.get();
}

frc::LidarSubsystem *SensorManager::GetLidarSubsystem()
{
    std::lock_guard<std::mutex> lock(m_sensorMutex);
    return lidar.get();
}

LineFollower *SensorManager::GetLineFollower()
{
    std::lock_guard<std::mutex> lock(m_sensorMutex);
    return lineFollower.get();
}