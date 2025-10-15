#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/GripperJointSubsystem.h"

/**
 * Command to move the gripper joint to a specific angle position.
 * The command finishes when the gripper stops moving (reaches the target).
 */
class MoveGripperJointToPosition : public frc2::CommandHelper<frc2::CommandBase, MoveGripperJointToPosition> {
public:
    /**
     * Constructor
     * @param subsystem Pointer to the GripperJointSubsystem
     * @param targetAngle Target angle in degrees
     */
    MoveGripperJointToPosition(GripperJointSubsystem* subsystem, double targetAngle);
    
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    GripperJointSubsystem* m_gripperJoint;
    double m_targetAngle;
};