#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ArmSubsystem.h"

/**
 * Command to move the arm to a specific angle position.
 * The command finishes when the arm stops moving (reaches the target).
 */
class MoveArmToPosition : public frc2::CommandHelper<frc2::CommandBase, MoveArmToPosition> {
public:
    /**
     * Constructor
     * @param subsystem Pointer to the ArmSubsystem
     * @param targetAngle Target angle in degrees
     */
    MoveArmToPosition(ArmSubsystem* subsystem, double targetAngle);
    
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    ArmSubsystem* m_arm;
    double m_targetAngle;
};