#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ExtenderSubsystem.h"
#include "subsystems/GripperJointSubsystem.h"

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
     * @param extender Pointer to the ExtenderSubsystem
     * @param gripper Pointer to the GripperJointSubsystem
     */
    FullPickSequence(ArmSubsystem* arm, ExtenderSubsystem* extender, GripperJointSubsystem* gripper);
};