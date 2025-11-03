#pragma once

#include <vector>
#include <atomic>
#include <thread>
#include <frc/Ultrasonic.h>

#include "Constants.h"
#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/IRRangeSubsystem.h"
#include "subsystems/sensor/Lidar.h"

class SensorManager
{
public:
    SensorManager();
    ~SensorManager();
    void SensorManagerStartThread();
    void SensorWorker();

    // Getter methods to access sensor instances
    frc::UltrasonicSubsystem *GetUltrasonicSubsystem();
    frc::IRRangeSubsystem *GetIRRangeSubsystem();
    frc::LidarSubsystem *GetLidarSubsystem();

private:
    std::thread workerThread;
    std::atomic<bool> stopThread;
     // Example analog port 0

    std::unique_ptr<frc::UltrasonicSubsystem> ultraSonic;
    std::unique_ptr<frc::IRRangeSubsystem> infraRedLeft;
    std::unique_ptr<frc::IRRangeSubsystem> infraRedRight;
    std::unique_ptr<frc::LidarSubsystem> lidar;
};
