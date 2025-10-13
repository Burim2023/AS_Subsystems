#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

// Forward declaration to avoid including AMCU.h in header
class AMCU;

/**
 * Simple drive command to test the command-based setup.
 * Drives the robot in a specified direction for a given duration.
 */
class DriveForDuration : public frc2::CommandHelper<frc2::CommandBase, DriveForDuration> {
public:
    /**
     * Constructor
     * @param amcu Pointer to the AMCU drivetrain
     * @param x X velocity (forward/backward)
     * @param y Y velocity (left/right) 
     * @param rotation Rotation velocity
     * @param duration Duration to drive (in seconds)
     */
    DriveForDuration(AMCU* amcu, double x, double y, double rotation, double duration);
    
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    AMCU* m_amcu;
    frc::Timer m_timer;
    double m_x, m_y, m_rotation;
    double m_duration;
};