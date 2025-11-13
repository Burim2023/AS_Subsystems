#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/IRRangeSubsystem.h"
#include "subsystems/sensor/LineFollower.h"
#include "web-ds-logger/cpp/networktables/LoggingSystem.h"
#include "subsystems/sensor/SensorManager.h"

#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>

// thread stopper
std::atomic<bool> SensorManager::EnableSensorThread{false};

SensorManager::SensorManager()
{
    stopThread = false;

    ultraSonic = std::make_unique<frc::UltrasonicSubsystem>(0, 1, 2, 3);
    infraRed = std::make_unique<frc::IRRangeSubsystem>(0, 1);
    lineFollower = std::make_unique<LineFollower>(0, 1, 2, 3, 5.0f);
    
    std::cout << "SensorManager: Constructed (no LiDAR)" << std::endl;
}

SensorManager::~SensorManager()
{
    stopThread = true;

    if (workerThread.joinable())
        workerThread.join();
    
    std::cout << "SensorManager: Destroyed" << std::endl;
}

void SensorManager::SensorWorker()
{
    std::cout << "SensorManager: Worker thread started" << std::endl;
    
    while (!stopThread.load())
    {
        try
        {
            // Update sensors and cache their values atomically
            if (ultraSonic)
            {
                ultraSonic->UpdateUltraSonic();
                m_sensorCache.ultrasonicLeft.store(ultraSonic->GetLeftDistance(), std::memory_order_relaxed);
                m_sensorCache.ultrasonicRight.store(ultraSonic->GetRightDistance(), std::memory_order_relaxed);
            }

            if (infraRed)
            {
                infraRed->UpdateInfraRed();
                m_sensorCache.irLeft.store(infraRed->GetIRLeftDistance(), std::memory_order_relaxed);
                m_sensorCache.irRight.store(infraRed->GetIRRightDistance(), std::memory_order_relaxed);
            }

            // ✅ Update LineFollower
            if (lineFollower)
            {
                lineFollower->update();
                lineFollower->UpdateShuffleboard(5);  // Update every 5 cycles
            }
        }
        catch (const std::exception &e)
        {
            static int errorCount = 0;
            if (++errorCount % 100 == 0)
            {
                std::cerr << "SensorWorker exception: " << e.what() << " (count=" << errorCount << ")\n";
            }
        }
        catch (...)
        {
            static int unknownErrorCount = 0;
            if (++unknownErrorCount % 100 == 0)
            {
                std::cerr << "SensorWorker unknown exception (count=" << unknownErrorCount << ")\n";
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(Constants::SENSOR_UPDATE_RATE));
    }
    
    std::cout << "SensorManager: Worker thread stopped" << std::endl;
}

void SensorManager::InitializeSensors()
{
    std::cout << "SensorManager: Initializing sensors..." << std::endl;
    
    if (ultraSonic)
    {
        ultraSonic->Init();
        std::cout << "  ✓ Ultrasonic initialized" << std::endl;
    }
    
    if (infraRed)
    {
        infraRed->Init();
        std::cout << "  ✓ Infrared initialized" << std::endl;
    }
    
    // ✅ Initialize LineFollower
    if (lineFollower)
    {
        lineFollower->InitShuffleboard("Line Follower");
        
        // Try to load saved calibration
        if (LoadLineFollowerCalibration())
        {
            std::cout << "  ✓ LineFollower calibration loaded from file" << std::endl;
        }
        else
        {
            std::cout << "  ⚠ LineFollower using default calibration" << std::endl;
            std::cout << "    Run calibration: Y (white) → A (black) → B (save)" << std::endl;
        }
    }
    
    std::cout << "SensorManager: All sensors initialized" << std::endl;
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
    return ultraSonic.get();
}

frc::IRRangeSubsystem *SensorManager::GetIRRangeSubsystem()
{
    return infraRed.get();
}

LineFollower *SensorManager::GetLineFollower()
{
    return lineFollower.get();
}

// ✅ NEW: LineFollower calibration methods
void SensorManager::CalibrateLineFollowerWhite()
{
    if (!lineFollower)
    {
        std::cerr << "SensorManager: ERROR - LineFollower not initialized!" << std::endl;
        return;
    }
    
    lineFollower->CaptureWhite();
    std::cout << "SensorManager: ✓ White surface calibration captured" << std::endl;
}

void SensorManager::CalibrateLineFollowerBlack()
{
    if (!lineFollower)
    {
        std::cerr << "SensorManager: ERROR - LineFollower not initialized!" << std::endl;
        return;
    }
    
    lineFollower->CaptureBlack();
    std::cout << "SensorManager: ✓ Black line calibration captured" << std::endl;
}

bool SensorManager::SaveLineFollowerCalibration()
{
    if (!lineFollower)
    {
        std::cerr << "SensorManager: ERROR - LineFollower not initialized!" << std::endl;
        return false;
    }
    
    std::cout << "SensorManager: Saving LineFollower calibration..." << std::endl;
    
    if (lineFollower->SaveCalibrationToFile(CALIBRATION_FILE))
    {
        std::cout << "SensorManager: ✓ Calibration saved to " << CALIBRATION_FILE << std::endl;
        return true;
    }
    else
    {
        std::cerr << "SensorManager: ✗ Failed to save calibration!" << std::endl;
        return false;
    }
}

bool SensorManager::LoadLineFollowerCalibration()
{
    if (!lineFollower)
    {
        std::cerr << "SensorManager: ERROR - LineFollower not initialized!" << std::endl;
        return false;
    }
    
    return lineFollower->LoadCalibrationFromFile(CALIBRATION_FILE);
}