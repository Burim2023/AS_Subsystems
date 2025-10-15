#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
#include "commands/MoveGripperJointToPosition.h"
#include "subsystems/GripperJointSubsystem.h"
#include "subsystems/GripperSubsystem.h"

/**
 * Command sequence that demonstrates gripper operation at different levels:
 * 1. Move to DOWN position, open gripper, wait 1s, close gripper
 * 2. Move to MID position, open gripper, wait 1s, close gripper  
 * 3. Move to UP position, open gripper, wait 1s, close gripper
 */
class GripperPickupSequence : public frc2::SequentialCommandGroup {
public:
    /**
     * Constructor
     * @param gripperJoint Pointer to the GripperJointSubsystem
     * @param gripper Pointer to the GripperSubsystem
     */
    GripperPickupSequence(GripperJointSubsystem* gripperJoint, GripperSubsystem* gripper);
};