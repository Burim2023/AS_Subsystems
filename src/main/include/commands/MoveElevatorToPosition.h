#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "subsystems/elevator/ElevatorSubsystem.h"
#include <frc/Timer.h>

/**
 * Command to move the elevator to a specific position and wait for completion.
 */
class MoveElevatorToPosition : public frc2::CommandHelper<frc2::CommandBase, MoveElevatorToPosition> {
public:
    /**
     * Constructor
     * @param elevator Pointer to elevator subsystem
     * @param targetPosition Target position in mm
     * @param tolerance Position tolerance in mm (default 2.0mm)
     */
    MoveElevatorToPosition(ElevatorSubsystem* elevator, float targetPosition, float tolerance = 2.0f);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    ElevatorSubsystem* m_elevator;
    float m_targetPosition;
    float m_tolerance;
    double m_startTime;
    double m_timeout;  // Add timeout
};