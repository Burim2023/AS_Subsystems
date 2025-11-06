#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/IRRangeSubsystem.h"
#include "subsystems/sensor/Lidar.h"
#include "subsystems/sensor/LineFollower.h"
#include "web-ds-logger/cpp/networktables/LoggingSystem.h"
#include "subsystems/sensor/SensorManager.h"

#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>

SensorManager::SensorManager()
{
    stopThread = false;

    ultraSonic = std::make_unique<frc::UltrasonicSubsystem>(0, 1, 2, 3);
    infraRed = std::make_unique<frc::IRRangeSubsystem>(0, 1);

    // LiDAR will be initialized in the sensor thread to avoid blocking
    // Just create the object here without Init/StartScan
    lidar = nullptr; // Will be created in InitializeSensors()

    std::cout << "SensorManager: Constructor complete (LiDAR will init in background)" << std::endl;
}
SensorManager::~SensorManager()
{
    stopThread = true;

    // Stop LiDAR scanning before destroying
    if (lidar)
    {
        lidar->StopScan();
        std::cout << "SensorManager: LiDAR scan stopped on shutdown" << std::endl;
    }

    if (workerThread.joinable())
        workerThread.join();
}

void SensorManager::SensorWorker()
{
    while (!stopThread.load())
    {
        if (ultraSonic)
        {
            ultraSonic->UpdateUltraSonic();
        }
        if (infraRed)
        {
            infraRed->UpdateInfraRed();
        }
        if (lidar)
        {
            lidar->Periodic();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(Constants::SENSOR_UPDATE_RATE));
    }
}

void SensorManager::InitializeSensors()
{
    // Initialize Ultrasonic and IR sensors (these are fast)
    if (ultraSonic)
    {
        ultraSonic->Init();
    }
    if (infraRed)
    {
        infraRed->Init();
    }

    // Initialize LiDAR in background thread with try-catch
    // This can take 1-2 seconds, so we do it here to avoid blocking robot init
    try
    {
        std::cout << "SensorManager: Starting LiDAR initialization in background..." << std::endl;
        lidar = std::make_unique<frc::LidarSubsystem>(studica::Lidar::kUSB1);
        lidar->Init();
        lidar->StartScan();
        std::cout << "SensorManager: LiDAR initialized successfully" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << "SensorManager: LiDAR initialization failed: " << e.what() << std::endl;
        std::cout << "SensorManager: Continuing without LiDAR functionality" << std::endl;
        lidar = nullptr;
    }
}
void SensorManager::SensorManagerStartThread()
{
    InitializeSensors();
    workerThread = std::thread(&SensorManager::SensorWorker, this);
    LOG_THREAD("Sensor Thread initialized.");
}

frc::UltrasonicSubsystem *SensorManager::GetUltrasonicSubsystem()
{
    return ultraSonic.get();
}

frc::IRRangeSubsystem *SensorManager::GetIRRangeSubsystem()
{
    return infraRed.get();
}

frc::LidarSubsystem *SensorManager::GetLidarSubsystem()
{
    return lidar.get();
}
