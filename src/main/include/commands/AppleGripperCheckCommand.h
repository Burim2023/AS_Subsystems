#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/vision/CameraSubsystem.h"

/**
 * Simplified command to detect apple presence using Orbbec 3D depth camera
 * - Detects apple using RGB processing
 * - Gets distance using depth data (200mm min, 2.5m max)
 * - Logs results to console
 */
class AppleGripperCheckCommand : public frc2::CommandHelper<frc2::CommandBase, AppleGripperCheckCommand> {
public:
    enum class CheckMode {
        QUICK_CHECK,            // Single check - finish immediately
        CONTINUOUS_MONITOR      // Continuous monitoring
    };

    /**
     * Constructor for apple detection
     * @param cameraSubsystem Pointer to the CameraSubsystem
     * @param mode Check mode to use
     * @param timeoutSeconds Maximum time to run (0 = no timeout)
     */
    AppleGripperCheckCommand(CameraSubsystem* cameraSubsystem, 
                            CheckMode mode = CheckMode::QUICK_CHECK, 
                            double timeoutSeconds = 3.0);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

    // Getter methods
    bool WasAppleDetected() const { return m_appleDetected; }
    double GetAppleDistance() const { return m_appleDistance; }

private:
    CameraSubsystem* m_camera;
    CheckMode m_mode;
    double m_timeoutSeconds;
    frc::Timer m_timer;
    
    // Detection results
    bool m_appleDetected;        // RGB camera detects apple
    double m_appleDistance;      // Distance from depth camera (mm)
    
    // Orbbec depth camera constants
    static constexpr double MIN_DEPTH_MM = 200.0;     // 200mm minimum
    static constexpr double MAX_DEPTH_MM = 2500.0;    // 2.5m maximum
    
    // Helper methods
    std::string GetModeString() const;
    void LogAppleStatus() const;
};