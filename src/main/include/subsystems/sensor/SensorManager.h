#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <string>
#include <frc2/command/SubsystemBase.h>  // ✅ ADD THIS LINE

#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/IRRangeSubsystem.h"
#include "subsystems/sensor/LineFollower.h"
#include "Constants.h"

class SensorManager : public frc2::SubsystemBase
{
public:
    SensorManager();
    ~SensorManager();

    void InitializeSensors();
    void SensorManagerStartThread();
    void SensorManagerStopThread();

    frc::UltrasonicSubsystem *GetUltrasonicSubsystem();
    frc::IRRangeSubsystem *GetIRRangeSubsystem();
    LineFollower *GetLineFollower();

    // ✅ NEW: LineFollower calibration helpers
    void CalibrateLineFollowerWhite();
    void CalibrateLineFollowerBlack();
    bool SaveLineFollowerCalibration();
    bool LoadLineFollowerCalibration();

    // Sensor cache for thread-safe access
    struct SensorCache
    {
        std::atomic<double> ultrasonicLeft{0.0};
        std::atomic<double> ultrasonicRight{0.0};
        std::atomic<double> irLeft{0.0};
        std::atomic<double> irRight{0.0};
    };

    SensorCache& GetCache() { return m_sensorCache; }  // ✅ Changed m_sensorCache → m_cache
    const SensorCache& GetCache() const { return m_sensorCache; }  // ✅ Changed m_sensorCache → m_cache

    static std::atomic<bool> EnableSensorThread;

private:
    void SensorWorker();

    std::unique_ptr<frc::UltrasonicSubsystem> ultraSonic;
    std::unique_ptr<frc::IRRangeSubsystem> infraRed;
    std::unique_ptr<LineFollower> lineFollower;

    std::thread workerThread;
    std::atomic<bool> stopThread;

    SensorCache m_sensorCache;  // ✅ Changed m_sensorCache → m_cache

    // ✅ NEW: Calibration file path
    static constexpr const char *CALIBRATION_FILE = "/home/lvuser/linefollower_cal.json";
};
