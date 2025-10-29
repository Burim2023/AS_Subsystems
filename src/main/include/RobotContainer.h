#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/smartdashboard/SendableChooser.h>


// Subsystem includes
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ExtenderSubsystem.h"
#include "subsystems/GripperSubsystem.h"
#include "subsystems/GripperJointSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/CameraSubsystem.h"
#include "gamepad/OI.h"
//#include "subsystems/Drivetrain.h"

// Command includes
#include "commands/MoveArmToPosition.h"
#include "commands/MoveGripperJointToPosition.h"
#include "commands/ExtendForDuration.h"
#include "commands/DriveForDuration.h"
#include "commands/SimpleDrive.h"
#include "commands/GripperOperate.h"
#include "commands/MoveElevatorToPosition.h"
#include "commands/CalibrateElevator.h"
#include "commands/ElevatorPresets.h"
#include "commands/ElevatorTestSequence.h"
#include "commands/FullPickSequence.h"
#include "commands/RetractAndLift.h"
#include "commands/TestCommandSequence.h"
#include "commands/GripperPickupSequence.h"
#include "commands/ExtenderCalibrationSequence.h"
#include "commands/CalibrateExtender.h"
#include "commands/AppleGripperCheckCommand.h"
#include "commands/SmartPickSequence.h"
#include "commands/PickupAndDeliverSequence.h"

#include "commands/DriveSmartPickupGround.h"
// Non-command-based subsystems
#include "AMCU.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  
  void SetAMCU(AMCU* amcu);
  //public getter methods
  GripperJointSubsystem& GetGripperJoint() { return m_gripperJoint; }
  GripperSubsystem& GetGripper() { return m_gripper; }
  ElevatorSubsystem& GetElevator() { return m_elevator; }
  ExtenderSubsystem& GetExtender() { return m_extender; }
  CameraSubsystem& GetCamera() {return m_camera; }

 private:
  // Subsystems
  ArmSubsystem m_arm;
  ExtenderSubsystem m_extender;
  GripperSubsystem m_gripper;
  GripperJointSubsystem m_gripperJoint;
  ElevatorSubsystem m_elevator;
  CameraSubsystem m_camera;
  //Drivetrain m_drivetrain;
  // 
  OI m_oi;

 

  // Autonomous Chooser
  frc::SendableChooser<frc2::Command*> m_chooser;

  // Non-command-based subsystems
  AMCU* m_amcu;

 // Commands (Reordered to match initialization in .cpp)
  FullPickSequence m_autoPickSequence;
  RetractAndLift m_autoRetractAndLift;
  CalibrateExtender m_calibrateExtenderOnly;
  ExtenderCalibrationSequence m_demoExtender;
  SimpleDrive m_simpleDrive;
  TestCommandSequence m_testSequence;
  GripperOperate m_gripperOperateUp;
  GripperOperate m_gripperOperateDown;
  GripperOperate m_gripperPickup;
  GripperPickupSequence m_gripperPickupSequence;
  CalibrateElevator m_calibrateElevator;
  ElevatorPresets m_elevatorGround;
  ElevatorPresets m_elevatorLow;
  ElevatorPresets m_elevatorHigh;
  MoveElevatorToPosition m_elevatorCustom;
  ElevatorTestSequence m_elevatorTestSequence;
  SmartPickSequence m_smartPickSequence;
  DriveSmartPickupGround m_driveSmartPickupGround;
  AppleGripperCheckCommand m_checkAppleGrip;
  AppleGripperCheckCommand m_waitForGrip;
  AppleGripperCheckCommand m_monitorGrip;
  PickupAndDeliverSequence m_pickupAndDerliverSequence;



  void ConfigureButtonBindings();
};
