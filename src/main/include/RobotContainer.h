#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/smartdashboard/SendableChooser.h>

// Subsystem includes
#include "subsystems/elevator/ArmSubsystem.h"
#include "subsystems/elevator/ExtenderSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"
#include "subsystems/gripper/GripperJointSubsystem.h"
#include "subsystems/elevator/ElevatorSubsystem.h"
#include "subsystems/vision/CameraSubsystem.h"
#include "subsystems/joystick/Gamepad.h"
#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/Lidar.h"
#include "subsystems/sensor/SensorManager.h"
#include "subsystems/sensor/IRRangeSubsystem.h"
// #include "subsystems/Drivetrain.h"
#include "subsystems/sensor/LineFollower.h"

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
// #include "commands/PickupAndDeliverSequence.h"
#include "commands/WallAlignDriveCommand.h"
#include "commands/Drive/DriveUntilWallCommand.h"
#include "commands/Drive/CobraLineFollowCommand.h"

#include "commands/DriveSmartPickupGround.h"
// Non-command-based subsystems
#include "Constants.h"
#include "subsystems/amcu/AMCU.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer();
  ~RobotContainer();

  void SetAMCU(AMCU *amcu_ptr);
  void SetSensorManager(SensorManager *sensor_ptr);

  // frc
  frc2::Command *GetAutonomousCommand();
  frc::UltrasonicSubsystem  *GetUltrasonic(){ return m_sensorManager ? m_sensorManager->GetUltrasonicSubsystem() : nullptr; }
  frc::IRRangeSubsystem     *GetIRRange(){ return m_sensorManager ? m_sensorManager->GetIRRangeSubsystem() : nullptr; }
  
  GripperJointSubsystem   &GetGripperJoint() { return m_gripperJoint; }
  GripperSubsystem        &GetGripper() { return m_gripper; }
  ElevatorSubsystem       &GetElevator() { return m_elevator; }
  ExtenderSubsystem       &GetExtender() { return m_extender; }
  CameraSubsystem         &GetCamera() { return m_camera; }
  LineFollower            *GetLineFollower() { return &m_lineFollower; }
  Gamepad                 *GetGamepad() { return &m_gamepad; }
  SensorManager           *GetSensorManager() { return m_sensorManager; }

private:
  ArmSubsystem          m_arm;
  ExtenderSubsystem     m_extender;
  GripperSubsystem      m_gripper;
  GripperJointSubsystem m_gripperJoint;
  ElevatorSubsystem     m_elevator;
  CameraSubsystem       m_camera;
  LineFollower          m_lineFollower{0, 1, 2, 3, 5.0f};
  Gamepad               m_gamepad;
  SensorManager         *m_sensorManager = nullptr;
  AMCU                  *m_amcu = nullptr;

  // Autonomous Chooser
  frc::SendableChooser<frc2::Command *> m_chooser;

  // Commands (Reordered to match initialization in .cpp)
  FullPickSequence            m_autoPickSequence;
  RetractAndLift              m_autoRetractAndLift;
  CalibrateExtender           m_calibrateExtenderOnly;
  ExtenderCalibrationSequence m_demoExtender;
  SimpleDrive                 m_simpleDrive;
  TestCommandSequence         m_testSequence;
  GripperOperate              m_gripperOperateUp;
  GripperOperate              m_gripperOperateDown;
  GripperOperate              m_gripperPickup;
  GripperPickupSequence       m_gripperPickupSequence;
  CalibrateElevator           m_calibrateElevator;
  ElevatorPresets             m_elevatorGround;
  ElevatorPresets             m_elevatorLow;
  ElevatorPresets             m_elevatorHigh;
  MoveElevatorToPosition      m_elevatorCustom;
  ElevatorTestSequence        m_elevatorTestSequence;
  SmartPickSequence           m_smartPickSequence;
  DriveSmartPickupGround      m_driveSmartPickupGround;
  AppleGripperCheckCommand    m_checkAppleGrip;
  AppleGripperCheckCommand    m_waitForGrip;
  AppleGripperCheckCommand    m_monitorGrip;    
  WallAlignDriveCommand       m_wallAlignDriveCommand;
  DriveUntilWallCommand       m_driveUntilWallCommand;
  CobraLineFollowCommand      m_cobraLineFollowCommand;
  // PickupAndDeliverSequence m_pickupAndDerliverSequence;

  void ConfigureButtonBindings();
};
