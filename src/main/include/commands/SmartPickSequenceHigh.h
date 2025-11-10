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
#include "subsystems/elevator/ArmSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"
#include "subsystems/gripper/GripperJointSubsystem.h"
#include "subsystems/vision/CameraSubsystem.h"
#include "subsystems/elevator/ElevatorSubsystem.h"

// Commands
#include "commands/AppleGripperCheckCommand.h"
#include "commands/MoveArmToPosition.h"
#include "commands/MoveGripperJointToPosition.h"
#include "commands/ElevatorPresets.h"
#include "commands/CalibrateElevator.h"

class SmartPickSequenceHigh : public frc2::SequentialCommandGroup {
public:
    SmartPickSequenceHigh(ArmSubsystem* arm,
                         GripperSubsystem* gripper,
                         GripperJointSubsystem* gripperJoint,
                         CameraSubsystem* camera,
                         ElevatorSubsystem* elevator);
};