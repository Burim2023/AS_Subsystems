#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/ExtenderSubsystem.h"

/**
 * Command to run the extender servo for a specific duration.
 * Since the ExtenderSubsystem is a continuous servo with no position feedback,
 * this command uses time-based control.
 */
class ExtendForDuration : public frc2::CommandHelper<frc2::CommandBase, ExtendForDuration> {
public:
    /**
     * Constructor
     * @param subsystem Pointer to the ExtenderSubsystem
     * @param duration Duration to run the servo (in seconds)
     * @param clockwise True for clockwise rotation, false for counter-clockwise
     */
    ExtendForDuration(ExtenderSubsystem* subsystem, double duration, bool clockwise);
    
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    ExtenderSubsystem* m_extender;
    frc::Timer m_timer;
    double m_duration;
    bool m_isClockwise;
};