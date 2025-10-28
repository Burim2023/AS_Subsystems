#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cmath>

// Optional includes for non-command-based subsystems if needed
#include "AMCU.h"
#include "Constants.h"
#include "utilities/LoggingSystem.h"
#include "subsystems/UltrasonicSubsystem.h"

// Global instances for non-command-based subsystems
OI oi;
AMCU amcu;
frc::UltrasonicSubsystem m_ultrasonic(0, 1, 2, 3);

void Robot::RobotInit() {
  // Initialize logging system
  SetupLogging();
  
  // Initialize ultrasonic subsystem (non-command-based)
  m_ultrasonic.Init();
  
  // Pass AMCU instance to RobotContainer
  m_container.SetAMCU(&amcu);
  
  // All command-based subsystems are initialized in RobotContainer constructor
}

void Robot::RobotPeriodic() { 
  // CRITICAL FIX: Only run command scheduler during autonomous and disabled
  // NOT during teleop to prevent conflicts
  // if (IsAutonomous()) {
  //   frc2::CommandScheduler::GetInstance().Run();
  // }
  frc2::CommandScheduler::GetInstance().Run();
  // Update non-command-based subsystems
  m_ultrasonic.Periodic();
}

void Robot::DisabledInit() {
  std::cout << "🛑 DISABLING ROBOT" << std::endl;
  
  amcu.stop();
  
  // FORCE STOP all individual motors
  amcu.setSpeed(MOTOR_1, 0);
  amcu.setSpeed(MOTOR_2, 0);
  amcu.setSpeed(MOTOR_3, 0);
  amcu.setSpeed(MOTOR_0, 0);
  
  // Cancel all running commands
  frc2::CommandScheduler::GetInstance().CancelAll();
  
  std::cout << "✅ Robot disabled - all motors stopped" << std::endl;
}

void Robot::DisabledPeriodic() {
 
  amcu.setSpeed(MOTOR_1, 0);
  amcu.setSpeed(MOTOR_2, 0);
  amcu.setSpeed(MOTOR_3, 0);
  
}

void Robot::AutonomousInit() {
  std::cout << "🤖 STARTING AUTONOMOUS" << std::endl;
  
  // Get the autonomous command from the container
  m_autonomousCommand = m_container.GetAutonomousCommand();

  // Schedule the autonomous command (if one was selected)
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
    std::cout << "✅ Autonomous command scheduled" << std::endl;
  }
  
  // Stop motors initially
  amcu.stop();
  amcu.setSpeed(MOTOR_1, 0);
  amcu.setSpeed(MOTOR_2, 0);
  amcu.setSpeed(MOTOR_3, 0);
}

void Robot::AutonomousPeriodic() {
  // The CommandScheduler (called in RobotPeriodic) handles running the autonomous command
  
  // Get ultrasonic distance for autonomous navigation (in centimeters)
  double leftDistanceCm = m_ultrasonic.GetLeftDistance();
  double rightDistanceCm = m_ultrasonic.GetRightDistance();
  bool leftWallDetected = m_ultrasonic.IsLeftWallDetected();
  bool rightWallDetected = m_ultrasonic.IsRightWallDetected();
  
  // Example autonomous logic using distance in centimeters
  if (leftDistanceCm > 0 && rightDistanceCm > 0) { // Valid reading
    if (rightWallDetected || leftWallDetected) {
      amcu.stop();
    }
  }
}

void Robot::TeleopInit() {
  std::cout << "🎮 STARTING TELEOP" << std::endl;
  
  // Cancel any autonomous commands when teleop starts
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
    std::cout << "❌ Cancelled autonomous command" << std::endl;
  }
  
  // Cancel all commands before switching to manual control
  //frc2::CommandScheduler::GetInstance().CancelAll();
  
  // FORCE STOP all motors immediately
  amcu.stop();
  amcu.setSpeed(MOTOR_1, 0);
  amcu.setSpeed(MOTOR_2, 0);
  amcu.setSpeed(MOTOR_3, 0);
  
  std::cout << "✅ TELEOP READY - Manual control active" << std::endl;
}

void Robot::TeleopPeriodic() {
  try {
    // Get joystick values with safety checks
    double leftY = 0.0;
    double leftX = 0.0;
    double rightY = 0.0;
    
    try { leftY = oi.GetLeftDriveY(); } catch (...) { leftY = 0.0; }
    try { leftX = oi.GetLeftDriveX(); } catch (...) { leftX = 0.0; }
    try { rightY = oi.GetRightDriveY(); } catch (...) { rightY = 0.0; }
    
    // Apply deadband to prevent joystick drift
    const double DEADBAND = 0.2;
    if (std::abs(leftY) < DEADBAND) leftY = 0.0;
    if (std::abs(leftX) < DEADBAND) leftX = 0.0;
    if (std::abs(rightY) < DEADBAND) rightY = 0.0;
    
    // CRITICAL: Check if ALL inputs are zero first
    if (leftY == 0.0 && leftX == 0.0 && rightY == 0.0) {
      // No joystick input - STOP ALL DRIVE MOTORS
      try {
        amcu.setSpeed(MOTOR_1, 0);
        amcu.setSpeed(MOTOR_2, 0);
        amcu.setSpeed(MOTOR_3, 0);
      } catch (...) {
        std::cout << "ERROR: Exception while stopping motors" << std::endl;
      }
      return;
    }

    // IMPORTANT SAFETY LIMITS - much lower speed for testing!
    // Reduce max speeds until you confirm everything works
    int forwardSpeed = static_cast<int>(leftY * 20);   // Reduced from 40 to 20
    int strafeSpeed = static_cast<int>(leftX * 20);    // Reduced from 40 to 20
    int rotationSpeed = static_cast<int>(rightY * 10); // Reduced from 20 to 10
    
    // 3-wheel omni drive calculations
    int motor1Speed = forwardSpeed - strafeSpeed - rotationSpeed;
    int motor2Speed = forwardSpeed + strafeSpeed + rotationSpeed;
    int motor3Speed = strafeSpeed * 2;
    
    // Clamp motor speeds to safe range
    motor1Speed = std::max(-20, std::min(20, motor1Speed)); // Reduced from 40 to 20
    motor2Speed = std::max(-20, std::min(20, motor2Speed)); // Reduced from 40 to 20
    motor3Speed = std::max(-20, std::min(20, motor3Speed)); // Reduced from 40 to 20
    
    // Set the calculated motor speeds with safety checks
    try {
      amcu.setSpeed(MOTOR_1, motor1Speed);
      amcu.setSpeed(MOTOR_2, motor2Speed);
      amcu.setSpeed(MOTOR_3, motor3Speed);
    } catch (const std::exception& e) {
      std::cout << "ERROR setting motor speeds: " << e.what() << std::endl;
      // Try to stop motors in case of error
      try {
        amcu.stop();
      } catch (...) {}
    }
  } catch (const std::exception& e) {
    std::cout << "CRITICAL ERROR in TeleopPeriodic: " << e.what() << std::endl;
  } catch (...) {
    std::cout << "UNKNOWN ERROR in TeleopPeriodic" << std::endl;
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif