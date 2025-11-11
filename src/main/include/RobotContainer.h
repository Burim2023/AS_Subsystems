#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <memory>

// Subsystem includes
#include "subsystems/elevator/ArmSubsystem.h"
#include "subsystems/elevator/ExtenderSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"
#include "subsystems/gripper/GripperJointSubsystem.h"
#include "subsystems/elevator/ElevatorSubsystem.h"
#include "subsystems/vision/CameraSubsystem.h"
#include "subsystems/joystick/Gamepad.h"
#include "subsystems/sensor/SensorManager.h"

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
#include "commands/SmartPickSequenceMid.h"
#include "commands/SmartPickSequenceHigh.h"
#include "commands/QRCodeReaderCommand.h"
#include "commands/WallAlignDriveCommand.h"
#include "commands/Drive/DriveUntilWallCommand.h"
#include "commands/Drive/CobraLineFollowCommand.h"
#include "commands/DriveSmartPickupGround.h"
#include "commands/StoreAppleCommand.h"

// Hardware subsystems
#include "Constants.h"
#include "subsystems/amcu/AMCU.h"

class RobotContainer
{
public:
  RobotContainer();
  ~RobotContainer();

  // Accessors for Robot.cpp initialization
  frc2::Command *GetAutonomousCommand();

  // Subsystem accessors for Robot.cpp
  AMCU *GetAMCU() { return &m_amcu; }
  SensorManager *GetSensorManager() { return &m_sensorManager; }

  // Sensor subsystem accessors
  frc::UltrasonicSubsystem *GetUltrasonic() { return m_sensorManager.GetUltrasonicSubsystem(); }
  frc::IRRangeSubsystem *GetIRRange() { return m_sensorManager.GetIRRangeSubsystem(); }
  LineFollower *GetLineFollower() { return m_sensorManager.GetLineFollower(); }

  // Robot subsystem accessors
  GripperJointSubsystem &GetGripperJoint() { return m_gripperJoint; }
  GripperSubsystem &GetGripper() { return m_gripper; }
  ElevatorSubsystem &GetElevator() { return m_elevator; }
  ExtenderSubsystem &GetExtender() { return m_extender; }
  CameraSubsystem &GetCamera() { return m_camera; }
  Gamepad *GetGamepad() { return &m_gamepad; }

private:
  // === SUBSYSTEMS (owned by container) ===
  // Drive system
  AMCU m_amcu; // Now a proper subsystem, not a pointer

  // Sensor system
  SensorManager m_sensorManager; // Now a proper subsystem, not a pointer

  // Manipulator subsystems
  ArmSubsystem m_arm;
  ExtenderSubsystem m_extender;
  GripperSubsystem m_gripper;
  GripperJointSubsystem m_gripperJoint;
  ElevatorSubsystem m_elevator;

  // Vision system
  CameraSubsystem m_camera;

  // Operator interface
  Gamepad m_gamepad;

  // Autonomous Chooser
  frc::SendableChooser<frc2::Command *> m_chooser;

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
  SmartPickSequenceMid m_smartPickSequenceMid;
  SmartPickSequenceHigh m_smartPickSequenceHigh;
  DriveSmartPickupGround m_driveSmartPickupGround;
  AppleGripperCheckCommand m_checkAppleGrip;
  AppleGripperCheckCommand m_waitForGrip;
  AppleGripperCheckCommand m_monitorGrip;
  WallAlignDriveCommand m_wallAlignDriveCommand;
  DriveUntilWallCommand m_driveUntilWallCommand;
  CobraLineFollowCommand m_cobraLineFollowCommand;
  QRCodeReaderCommand m_qrCodeReaderCommandSingle;
  QRCodeReaderCommand m_qrCodeReaderCommandTimed;
  QRCodeReaderCommand m_qrCodeReaderCommandContinuous;
  storage::StoreAppleCommand m_storeAppleAuto;   // Auto-detection
  storage::StoreAppleCommand m_storeAppleRed;    // Manual red
  storage::StoreAppleCommand m_storeAppleYellow; // Manual yellow
  storage::StoreAppleCommand m_storeAppleGreen;  // Manual green
  // PickupAndDeliverSequence m_pickupAndDerliverSequence;

  void ConfigureButtonBindings();
};
