#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/IRRangeSubsystem.h"
#include "subsystems/sensor/LidarSubsystem.h"
#include "subsystems/sensor/LineFollower.h"
#include "web-ds-logger/cpp/networktables/LoggingSystem.h"
#include "subsystems/sensor/SensorManager.h"

#include <algorithm>
#include <iostream>
#include <frc2/command/CommandScheduler.h>

SensorManager::SensorManager()
{
    SetName("SensorManager");

    // Create sensor subsystems
    ultraSonic = std::make_unique<frc::UltrasonicSubsystem>(0, 1, 2, 3);
    infraRed = std::make_unique<frc::IRRangeSubsystem>(0, 1);
    lineFollower = std::make_unique<LineFollower>(0, 1, 2, 3, 5.0f);
    lidar = nullptr;

    std::cout << "SensorManager: Constructor complete" << std::endl;
}

SensorManager::~SensorManager()
{
    if (lidar)
    {
        lidar->StopScan();
        std::cout << "SensorManager: Stopping Lidar Scanning" << std::endl;
    }
}

void SensorManager::InitializeSensors()
{
    std::cout << "SensorManager: Initializing sensor subsystems..." << std::endl;

    // Initialize and register sensor subsystems with scheduler
    if (ultraSonic)
    {
        ultraSonic->Init();
        frc2::CommandScheduler::GetInstance().RegisterSubsystem(ultraSonic.get());
        std::cout << "SensorManager: Ultrasonic registered" << std::endl;
    }

    if (infraRed)
    {
        infraRed->Init();
        frc2::CommandScheduler::GetInstance().RegisterSubsystem(infraRed.get());
        std::cout << "SensorManager: IR Range registered" << std::endl;
    }

    if (lineFollower)
    {
        lineFollower->Init();
        frc2::CommandScheduler::GetInstance().RegisterSubsystem(lineFollower.get());
        std::cout << "SensorManager: Line Follower registered" << std::endl;
    }

    // Initialize LiDAR with error handling
    try
    {
        std::cout << "SensorManager: Starting LiDAR initialization..." << std::endl;
        std::lock_guard<std::mutex> lock(m_sensorMutex);
        lidar = std::make_unique<frc::LidarSubsystem>(studica::Lidar::kUSB1);
        lidar->Init();
        lidar->StartScan();
        frc2::CommandScheduler::GetInstance().RegisterSubsystem(lidar.get());
        m_lidarReady.store(true);
        std::cout << "SensorManager: LiDAR initialized and registered successfully" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::lock_guard<std::mutex> lock(m_sensorMutex);
        std::cout << "SensorManager: LiDAR initialization failed: " << e.what() << std::endl;
        lidar = nullptr;
        m_lidarReady.store(false);
    }
}

void SensorManager::Periodic()
{
    // Periodic updates for sensor management
    // Individual sensor subsystems have their own Periodic() methods
    // This can be used for aggregate sensor data processing if needed
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
    std::lock_guard<std::mutex> lock(m_sensorMutex);
    return m_lidarReady.load() ? lidar.get() : nullptr;
}

LineFollower *SensorManager::GetLineFollower()
{
    return lineFollower.get();
}
