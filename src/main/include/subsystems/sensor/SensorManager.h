#pragma once

#include <vector>
#include <atomic>
#include <thread>
#include <frc/Ultrasonic.h>

#include "Constants.h"
#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/IRRangeSubsystem.h"
#include "subsystems/sensor/LidarSubsystem.h"
#include "subsystems/sensor/LineFollower.h"

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
    frc::LidarSubsystem *GetLidarSubsystem();
    LineFollower *GetLineFollower();

    // Simple flag to enable/disable sensor thread
    static std::atomic<bool> EnableSensorThread;

private:
    std::mutex m_sensorMutex;
    std::thread workerThread;
    std::atomic<bool> stopThread;

    std::unique_ptr<frc::UltrasonicSubsystem> ultraSonic;
    std::unique_ptr<frc::IRRangeSubsystem> infraRed;
    std::unique_ptr<frc::LidarSubsystem> lidar;
    std::unique_ptr<LineFollower> lineFollower;
};
