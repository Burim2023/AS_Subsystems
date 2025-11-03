#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/elevator/ArmSubsystem.h"
#include "subsystems/gripper/GripperJointSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"
#include "subsystems/elevator/ElevatorSubsystem.h"

/**
 * A complete sequence to pick up an object:
 * 1. Open the gripper
 * 2. Move arm to pick position
 * 3. Extend the arm
 * 4. Close the gripper
 * 5. Retract the arm
 * 6. Move arm to safe position
 */
class FullPickSequence : public frc2::SequentialCommandGroup {
public:
    /**
     * Constructor
     * @param arm Pointer to the ArmSubsystem
     * @param gripper Pointer to the ExtenderSubsystem
     * @param gripperJoint Pointer to the GripperJointSubsystem
     * @param elevator Pointer to the ElevatorSubsystem
     */
    FullPickSequence(ArmSubsystem* arm, GripperSubsystem* gripper, GripperJointSubsystem* gripperJoint, ElevatorSubsystem* elevator);
};