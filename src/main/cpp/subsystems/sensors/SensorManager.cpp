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
    infraRedLeft = std::make_unique<frc::IRRangeSubsystem>(0);
    infraRedRight = std::make_unique<frc::IRRangeSubsystem>(1);
    lidar = std::make_unique<frc::LidarSubsystem>();
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
        ultraSonic->UpdateUltraSonic();
        // infraRed->UpdateInfraRed();
        // lidar->UpdateLidar();

        std::this_thread::sleep_for(std::chrono::milliseconds(Constants::SENSOR_UPDATE_RATE));
    }
}

void SensorManager::SensorManagerStartThread()
{
    workerThread = std::thread(&SensorManager::SensorWorker, this);
    LOG_THREAD("Sensor Thread initialized.");
}

frc::UltrasonicSubsystem *SensorManager::GetUltrasonicSubsystem()
{
    return ultraSonic.get();
}

frc::IRRangeSubsystem *SensorManager::GetInfraRedSubsystem()
{
    return infraRed.get();
}

// LidarSubsystem *SensorManager::GetLidarSubsystem()
// {
//     return lidar.get();
// }



