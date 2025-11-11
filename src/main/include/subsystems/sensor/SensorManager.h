#pragma once

#include <vector>
#include <atomic>
#include <mutex>
#include <memory>
#include <frc/Ultrasonic.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/IRRangeSubsystem.h"
#include "subsystems/sensor/LidarSubsystem.h"
#include "subsystems/sensor/LineFollower.h"

class SensorManager : public frc2::SubsystemBase
{
public:
    SensorManager();
    ~SensorManager();

    /**
     * Initialize all sensor subsystems
     * Should be called once during robot initialization
     */
    void InitializeSensors();

    /**
     * Periodic update - called automatically by scheduler
     */
    void Periodic() override;

    // Sensor subsystem accessors
    frc::UltrasonicSubsystem *GetUltrasonicSubsystem();
    frc::IRRangeSubsystem *GetIRRangeSubsystem();
    frc::LidarSubsystem *GetLidarSubsystem();
    LineFollower *GetLineFollower();

private:
    std::mutex m_sensorMutex;
    std::atomic<bool> m_lidarReady{false};

    std::unique_ptr<frc::UltrasonicSubsystem> ultraSonic;
    std::unique_ptr<frc::IRRangeSubsystem> infraRed;
    std::unique_ptr<frc::LidarSubsystem> lidar;
    std::unique_ptr<LineFollower> lineFollower;
};
