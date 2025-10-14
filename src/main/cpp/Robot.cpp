#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

// Optional includes for non-command-based subsystems if needed
#include "AMCU.h"
#include "Constants.h"
#include "utilities/LoggingSystem.h"
#include "subsystems/UltrasonicSubsystem.h"

// Global instances for non-command-based subsystems
constexpr int kWheelRadius = 55;
constexpr int kRobotRadius = 162;
constexpr Motor kMotorLeft  = MOTOR_1;
constexpr Motor kMotorRight = MOTOR_2;
constexpr Motor kMotorBack  = MOTOR_3;
OI oi;
AMCU amcu;
frc::UltrasonicSubsystem m_ultrasonic(0, 1, 2, 3);



void Robot::RobotInit() {
  // Initialize logging system
  SetupLogging();
  
  // Initialize AMCU drivetrain (non-command-based)
  amcu.initOmniDriveBase(kWheelRadius, kRobotRadius, kMotorLeft, kMotorRight, kMotorBack);
  
  // Initialize ultrasonic subsystem (non-command-based)
  m_ultrasonic.Init();
  
  // Pass AMCU instance to RobotContainer
  m_container.SetAMCU(&amcu);
  
  // All command-based subsystems are initialized in RobotContainer constructor
}
void Robot::RobotPeriodic() { 
  // This is the most important call - it runs all command-based subsystems and commands
  frc2::CommandScheduler::GetInstance().Run();
  
  // Update non-command-based subsystems
  m_ultrasonic.Periodic();


  //irRange.Periodic();
  //lidar.Periodic();
  // lf.update();
  // lf.getVoltage();
  // lf.isLineDetected();
}

void Robot::DisabledInit() {
  amcu.stop();
  
  //lidar.StopScan();  // Stop LiDAR scanning when disabled
  
}
void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
  // Get the autonomous command from the container
  m_autonomousCommand = m_container.GetAutonomousCommand();

  // Schedule the autonomous command (if one was selected)
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  // The CommandScheduler (called in RobotPeriodic) handles running the autonomous command
  // Add any additional autonomous logic here if needed
  
  // Get ultrasonic distance for autonomous navigation (in centimeters)
  double leftDistanceCm = m_ultrasonic.GetLeftDistance();
  double rightDistanceCm = m_ultrasonic.GetRightDistance();
  bool leftWallDetected = m_ultrasonic.IsLeftWallDetected(); // 10cm threshold
  bool rightWallDetected = m_ultrasonic.IsRightWallDetected();
  
  // Example autonomous logic using distance in centimeters
  if (leftDistanceCm > 0 && rightDistanceCm > 0) { // Valid reading
    if (rightWallDetected || leftWallDetected) {
      std::cout << "WALL DETECTED! Left: " << leftDistanceCm << " cm, Right: " << rightDistanceCm << " cm - STOPPING" << std::endl;
      amcu.stop();
    } else {
      std::cout << "Clear path. Left: " << leftDistanceCm << " cm, Right: " << rightDistanceCm << " cm - CONTINUING" << std::endl;
      // Create different drive patterns
      
      //SimpleDrive(amcu, 0.5, 0.0, 0.0);  // Forward at 50%
    }
  }
}

void Robot::TeleopInit() {
  // Cancel any autonomous commands when teleop starts
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  
}

void Robot::TeleopPeriodic() {
  // Use the existing subsystem from RobotContainer instead of creating new one
  auto& gripperJoint = m_container.GetGripperJoint();
  auto& gripper = m_container.GetGripper();
  
  // Different buttons for different actions
  if(oi.GetDriveXButton()) {           // X button
    gripperJoint.SetSpeedNormal();
    gripperJoint.SetServoAngleZero();
  }
  if(oi.GetDriveSquareButton()) {      // Square button
    gripperJoint.SetSpeedNormal();
    gripperJoint.SetGripperDownAngle();
    gripper.SetOpenGripper();
    
  }
  if(oi.GetDriveCircleButton()) {      // Circle button
    gripperJoint.SetSpeedNormal();
    gripperJoint.SetGripperMidAngle();
    gripper.SetClosedGripper();
  }
  if(oi.GetDriveTriangleButton()) {    // Triangle button
    gripperJoint.SetSpeedNormal();
    gripperJoint.SetGripperUpAngle();
    gripper.SetOpenGripper();
  }
  
  // Call Periodic() to actually move the servo
  gripperJoint.Periodic();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif