#include "RobotContainer.h"
#include <frc2/command/button/JoystickButton.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() 
    : m_amcu(nullptr),
      m_autoPickSequence(&m_arm, &m_extender, &m_gripperJoint),
      m_autoRetractAndLift(&m_arm, &m_extender),
      m_testSequence(m_amcu, &m_arm, &m_extender),
      m_simpleDrive(nullptr, 0.3, 0.0, 0.0),
      m_gripperOperateUp(&m_gripperJoint, &m_gripper, GripperOperate::Position::UP, true, 2.0),
      m_gripperOperateDown(&m_gripperJoint, &m_gripper, GripperOperate::Position::DOWN, false, 2.0),
      m_gripperPickup(&m_gripperJoint, &m_gripper, GripperOperate::Position::MID, false, 2.0) {
  
  // Initialize all subsystems
  m_arm.Init();
  m_extender.Init();
  m_gripper.Init();
  m_gripperJoint.Init();
  
  
  // Configure the button bindings
  ConfigureButtonBindings();
  
  // Setup autonomous chooser
  m_chooser.SetDefaultOption("Test Command Sequence", &m_testSequence);
  m_chooser.AddOption("Simple Drive Forward", &m_simpleDrive);
  m_chooser.AddOption("Gripper Up & Open", &m_gripperOperateUp);
  m_chooser.AddOption("Gripper Down & Close", &m_gripperOperateDown);
  m_chooser.AddOption("Gripper Pickup (Mid & Close)", &m_gripperPickup);
  //m_chooser.AddOption("Full Pick Sequence", &m_autoPickSequence);
  //m_chooser.AddOption("Retract and Lift", &m_autoRetractAndLift);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void RobotContainer::ConfigureButtonBindings() {
  // Since your OI class uses direct button methods instead of a controller object,
  // we'll skip button bindings for now and rely on teleop periodic control
  // 
  // To add button bindings, you would need to either:
  // 1. Add a GetDriveController() method to your OI class that returns frc::Joystick&
  // 2. Or use a different approach with triggers based on your existing OI methods
  
  // For now, commands can be triggered manually or through autonomous mode
}

void RobotContainer::SetAMCU(AMCU* amcu) {
  m_amcu = amcu;
  // Update SimpleDrive command with the actual AMCU instance
  m_simpleDrive = SimpleDrive(amcu, 0.3, 0.0, 0.0); // Forward at 30% speed
  // Reinitialize the test sequence with the actual AMCU instance
  // Note: This is a bit of a hack - ideally we'd pass AMCU in the constructor
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Return the selected command from the chooser
  return m_chooser.GetSelected();
}
