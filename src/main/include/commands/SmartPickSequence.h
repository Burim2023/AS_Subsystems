#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>

// Subsystems
#include "subsystems/ArmSubsystem.h"
#include "subsystems/GripperSubsystem.h"
#include "subsystems/GripperJointSubsystem.h"
#include "subsystems/CameraSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

// Commands
#include "commands/AppleGripperCheckCommand.h"
#include "commands/MoveArmToPosition.h"
#include "commands/MoveGripperJointToPosition.h"
#include "commands/ElevatorPresets.h"
#include "commands/CalibrateElevator.h"

/**
 * Smart Pick Sequence with Apple Detection
 * 
 * This sequence:
 * 1. Moves arm to pick position
 * 2. Checks for apple presence using camera
 * 3. If apple detected: moves elevator down while setting gripper to down angle and open
 * 4. Moves elevator to MEDIUM position (60mm)
 * 5. If no apple: aborts sequence with warning
 */
class SmartPickSequence : public frc2::SequentialCommandGroup {
public:
    /**
     * Constructor
     * @param arm Pointer to ArmSubsystem
     * @param gripper Pointer to GripperSubsystem
     * @param gripperJoint Pointer to GripperJointSubsystem
     * @param camera Pointer to CameraSubsystem
     * @param elevator Pointer to ElevatorSubsystem
     */
    SmartPickSequence(ArmSubsystem* arm, 
                     GripperSubsystem* gripper, 
                     GripperJointSubsystem* gripperJoint,
                     CameraSubsystem* camera,
                     ElevatorSubsystem* elevator);
};