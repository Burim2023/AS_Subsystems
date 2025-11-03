#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/elevator/ElevatorSubsystem.h"

/**
 * Command to calibrate the elevator using limit switches.
 * This command has a timeout to prevent infinite calibration.
 */
class CalibrateElevator : public frc2::CommandHelper<frc2::CommandBase, CalibrateElevator> {
public:
    /**
     * Constructor
     * @param elevator Pointer to elevator subsystem
     * @param timeoutSeconds Maximum time to wait for calibration (default 15 seconds)
     */
    CalibrateElevator(ElevatorSubsystem* elevator, double timeoutSeconds = 15.0);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    ElevatorSubsystem* m_elevator;
    frc::Timer m_timer;
    double m_timeoutSeconds;
    float m_startPosition;
    bool m_calibrationStarted;
};