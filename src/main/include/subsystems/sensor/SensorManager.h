#pragma once

#include <vector>
#include <atomic>
#include <thread>
#include <frc/Ultrasonic.h>

#include "Constants.h"
#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/IRRangeSubsystem.h"
// #include "subsystems/sensor/LidarSubsystem.h"
#include "subsystems/sensor/LineFollower.h"

// Cached sensor values using atomics - thread-safe without locks!
struct SensorCache
{
    std::atomic<double> ultrasonicLeft{0.0};
    std::atomic<double> ultrasonicRight{0.0};
    std::atomic<double> irLeft{0.0};
    std::atomic<double> irRight{0.0};
    // std::atomic<double> lidarFront{0.0};
};

class SensorManager
{
public:
    SensorManager();
    ~SensorManager();
    void SensorManagerStartThread();
    void SensorManagerStopThread();
    void SensorWorker();
    void InitializeSensors();

    frc::UltrasonicSubsystem *GetUltrasonicSubsystem();
    frc::IRRangeSubsystem *GetIRRangeSubsystem();
    // frc::LidarSubsystem *GetLidarSubsystem();
    LineFollower *GetLineFollower();

    // Get cached sensor values (lock-free access)
    SensorCache *GetSensorCache() { return &m_sensorCache; }

    // Simple flag to enable/disable sensor thread
    static std::atomic<bool> EnableSensorThread;

private:
    std::mutex m_sensorMutex;
    std::thread workerThread;
    std::atomic<bool> stopThread;

    std::unique_ptr<frc::UltrasonicSubsystem> ultraSonic;
    std::unique_ptr<frc::IRRangeSubsystem> infraRed;
    // std::unique_ptr<frc::LidarSubsystem> lidar;
    std::unique_ptr<LineFollower> lineFollower;

    // Cached sensor values for lock-free dashboard updates
    SensorCache m_sensorCache;
};
