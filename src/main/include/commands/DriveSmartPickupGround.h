#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

// Subsystem includes
#include "subsystems/elevator/ArmSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"
#include "subsystems/gripper/GripperJointSubsystem.h"
#include "subsystems/vision/CameraSubsystem.h"
#include "subsystems/elevator/ElevatorSubsystem.h"

// Command includes
#include "commands/AppleGripperCheckCommand.h"
#include "commands/SpeedDriveCommand.h"
#include "commands/DriveDistanceCommand.h"

// Forward declaration for AMCU
class AMCU;

/**
 * @brief Command sequence that drives to find an apple, turns 180°, and executes smart pickup
 * 
 * This command combines autonomous driving with apple detection and a complete
 * gripper sequence including calibration and pickup operations.
 */
class DriveSmartPickupGround : public frc2::SequentialCommandGroup {
public:
    /**
     * @brief Constructor for DriveSmartPickupGround command
     * 
     * @param arm Pointer to the arm subsystem
     * @param gripper Pointer to the gripper subsystem  
     * @param gripperJoint Pointer to the gripper joint subsystem
     * @param camera Pointer to the camera subsystem for apple detection
     * @param elevator Pointer to the elevator subsystem
     * @param amcu Pointer to the AMCU for driving operations
     */
    DriveSmartPickupGround(ArmSubsystem* arm, 
                          GripperSubsystem* gripper, 
                          GripperJointSubsystem* gripperJoint,
                          CameraSubsystem* camera,
                          ElevatorSubsystem* elevator, 
                          AMCU* amcu);

    /**
     * @brief Get the name of this command
     * @return Command name as string
     */
    std::string GetName() const override { return "DriveSmartPickupGround"; }

private:
    // Store subsystem pointers for use in command sequence
    AMCU* m_amcu;
    ArmSubsystem* m_arm;
    GripperSubsystem* m_gripper;
    GripperJointSubsystem* m_gripperJoint;
    CameraSubsystem* m_camera;
    ElevatorSubsystem* m_elevator;
};