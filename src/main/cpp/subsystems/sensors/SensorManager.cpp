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

SensorManager::SensorManager()
{
    stopThread = false;

    ultraSonic = std::make_unique<frc::UltrasonicSubsystem>(0, 1, 2, 3);
    infraRed = std::make_unique<frc::IRRangeSubsystem>(0, 1);
    lineFollower = std::make_unique<LineFollower>(0, 1, 2, 3, 5.0f);
    lidar = nullptr; 
}
SensorManager::~SensorManager()
{
    stopThread = true;

    if (lidar)
    {
        lidar->StopScan();
        std::cout << "SensorManager: Stopping Lidar Scanning" << std::endl;
    }

    if (workerThread.joinable())
        workerThread.join();
}

void SensorManager::SensorWorker()
{
    int initCounter = 0;
    const int initAttemptInterval = 200; // attempt every N loops (~200*update_rate ms)
    while (!stopThread.load())
    {
        // Try LiDAR initialization in the worker thread (isolates JNI/native threads)
        if (!lidar)
        {
            if ((++initCounter % initAttemptInterval) == 0)
            {
                try
                {
                    std::lock_guard<std::mutex> lock(m_sensorMutex);
                    if (!lidar) // double-check under lock
                    {
                        std::cout << "SensorManager: Attempting LiDAR init in worker thread..." << std::endl;
                        lidar = std::make_unique<frc::LidarSubsystem>(studica::Lidar::kUSB1);
                        lidar->Init();
                        lidar->StartScan();
                        m_lidarReady.store(true);
                        std::cout << "SensorManager: LiDAR initialized successfully in worker." << std::endl;
                    }
                }
                catch (const std::exception &e)
                {
                    std::lock_guard<std::mutex> lock(m_sensorMutex);
                    std::cout << "SensorManager: LiDAR init failed in worker: " << e.what() << std::endl;
                    lidar = nullptr;
                    m_lidarReady.store(false);
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(m_sensorMutex);
            if (lidar && m_lidarReady.load())
            {
                try
                {
                    lidar->Periodic();
                }
                catch (const std::exception &e)
                {
                    std::cout << "LiDAR periodic error: " << e.what() << std::endl;
                    // Mark not ready and let init logic try restart later
                    m_lidarReady.store(false);
                    // stop the scan to allow restart attempt next time
                    try { lidar->StopScan(); } catch (...) {}
                    lidar = nullptr;
                }
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
                lineFollower->UpdateShuffleboard(10);
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

    // Do NOT initialize LiDAR here on the main thread - initialization will be attempted
    // inside the worker thread to isolate JNI threads and allow retries.
    {
        std::lock_guard<std::mutex> lock(m_sensorMutex);
        lidar = nullptr;
        m_lidarReady.store(false);
    }
}

void SensorManager::SensorManagerStartThread()
{
    // Start the worker thread. LiDAR initialization will be attempted inside the worker.
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
    std::lock_guard<std::mutex> lock(m_sensorMutex);
    return m_lidarReady.load() ? lidar.get() : nullptr;
}

LineFollower *SensorManager::GetLineFollower()
{
    return lineFollower.get();
}
