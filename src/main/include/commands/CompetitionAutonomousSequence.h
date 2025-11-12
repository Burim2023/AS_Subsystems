#pragma once

#include <frc2/command/SequentialCommandGroup.h>

// Subsystem includes
class AMCU;
#include "subsystems/elevator/ArmSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"
#include "subsystems/gripper/GripperJointSubsystem.h"
#include "subsystems/vision/CameraSubsystem.h"
#include "subsystems/elevator/ElevatorSubsystem.h"
#include "subsystems/elevator/ExtenderSubsystem.h"

// Command includes
#include "commands/DriveSmartPickupGround.h"
#include "commands/StoreAppleCommand.h"
#include "commands/DriveForDuration.h"
#include "commands/GripperOperate.h"
#include "commands/DriveDistanceCommand.h"
#include "commands/SmartPickSequence.h"
#include "commands/SpeedDriveCommand.h"
#include "commands/CalibrateExtender.h"

/**
 * CompetitionAutonomousSequence
 *
 * A ready-to-use autonomous sequence for competition that composes existing
 * commands. Sequence:
 *  - DriveSmartPickupGround (locate & pick an apple)
 *  - StoreAppleCommand (store picked apple into the given slot)
 *  - DriveForDuration forward to a scoring area
 *  - GripperOperate to open and release any held object
 *  - DriveForDuration backward to retreat
 */
class CompetitionAutonomousSequence : public frc2::SequentialCommandGroup {
public:
    /**
     * Constructor
     * @param amcu Drive subsystem pointer
     * @param arm Arm subsystem pointer
     * @param gripper Gripper subsystem pointer
     * @param gripperJoint Gripper joint subsystem pointer
     * @param camera Camera subsystem pointer
     * @param elevator Elevator subsystem pointer
     * @param extender Extender subsystem pointer
     * @param storeSlot Slot index to store apple (0..2)
     */
    CompetitionAutonomousSequence(AMCU* amcu,
                                  ArmSubsystem* arm,
                                  GripperSubsystem* gripper,
                                  GripperJointSubsystem* gripperJoint,
                                  CameraSubsystem* camera,
                                  ElevatorSubsystem* elevator,
                                  ExtenderSubsystem* extender);
};
