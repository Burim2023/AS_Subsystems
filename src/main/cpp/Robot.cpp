#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cmath>

#include "subsystems/amcu/AMCU.h"
#include "Constants.h"
#include "web-ds-logger/cpp/networktables/LoggingSystem.h"
#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/SensorManager.h"

#include "commands/SpeedDriveCommand.h"

Gamepad gamepad;
AMCU amcu;
frc::UltrasonicSubsystem m_ultrasonic(0, 1, 2, 3);
SensorManager sensormanager;


void Robot::RobotInit() {
  InitLogging(&sensormanager);
  SetupLogging();
  sensormanager.InitializeSensors();
  sensormanager.SensorManagerStartThread();
  
  //amcu.init();
  m_container.SetAMCU(&amcu);

  
  amcu.initOmniDriveBase(Constants::kWheelRadius, Constants::kRobotRadius, Constants::kMotorLeft, Constants::kMotorRight, Constants::kMotorBack);
  
}

void Robot::RobotPeriodic() {
  UpdateLogging(&sensormanager);
  // CRITICAL FIX: Only run command scheduler during autonomous and disabled
  // NOT during teleop to prevent conflicts
  if (IsAutonomous()) {
    frc2::CommandScheduler::GetInstance().Run();
  }
  //frc2::CommandScheduler::GetInstance().Run();
  // Update non-command-based subsystems
}

void Robot::DisabledInit() {
  try
  {
    amcu.stop();
    amcu.setSpeed(MOTOR_1, 0);
    amcu.setSpeed(MOTOR_2, 0);
    amcu.setSpeed(MOTOR_3, 0);
    amcu.setSpeed(MOTOR_0, 0);
    
    frc2::CommandScheduler::GetInstance().CancelAll();
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
 
  
  LOG_DISABLED(" Disabled.");
}

void Robot::DisabledPeriodic() {
 
  // amcu.setSpeed(MOTOR_1, 0);
  // amcu.setSpeed(MOTOR_2, 0);
  // amcu.setSpeed(MOTOR_3, 0);
  
}

void Robot::AutonomousInit() {
  
  // Get the autonomous command from the container
  m_autonomousCommand = m_container.GetAutonomousCommand();

  // Schedule the autonomous command (if one was selected)
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
    LOG_INFO("Autonomous command scheduled")
  }

  amcu.stop();
  amcu.setSpeed(MOTOR_1, 0);
  amcu.setSpeed(MOTOR_2, 0);
  amcu.setSpeed(MOTOR_3, 0);

  last_mode = {LOG_PURPLE, "[AUTONOMOUS]"};
  LOG_AUTONOMOUS("Enabled");
}

void Robot::AutonomousPeriodic() {
  // The CommandScheduler (called in RobotPeriodic) handles running the autonomous command
  
  // Get ultrasonic distance for autonomous navigation (in centimeters)
  // double leftDistanceCm = m_ultrasonic.GetLeftDistance();
  // double rightDistanceCm = m_ultrasonic.GetRightDistance();
  // bool leftWallDetected = m_ultrasonic.IsLeftWallDetected();
  // bool rightWallDetected = m_ultrasonic.IsRightWallDetected();
  
  // // Example autonomous logic using distance in centimeters
  // if (leftDistanceCm > 0 && rightDistanceCm > 0) { // Valid reading
  //   if (rightWallDetected || leftWallDetected) {
  //     amcu.stop();
  //   }
  // }
}

void Robot::TeleopInit() {
  
  // Cancel any autonomous commands when teleop starts
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
    LOG_WARN("Cancelled all autonomous commands")
  }
  
  // Cancel all commands before switching to manual control
  //frc2::CommandScheduler::GetInstance().CancelAll();
  
  // FORCE STOP all motors immediately
  amcu.stop();
  amcu.setSpeed(MOTOR_1, 0);
  amcu.setSpeed(MOTOR_2, 0);
  amcu.setSpeed(MOTOR_3, 0);
  
  last_mode = {LOG_CYAN, "[TELEOP]"};
  LOG_TELEOP("Enabled.");
}

void Robot::TeleopPeriodic() {
  
  // amcu.speedDrive(25, 0, 0);
  
  
  try {
    // Get joystick values with safety checks
    double leftY = 0.0;
    double leftX = 0.0;
    double rightY = 0.0;
    double rightX = 0.0;
    
    try { leftY = gamepad.GetLeftStickY(); } catch (...) { leftY = 0.0; }
    try { leftX = gamepad.GetLeftStickX(); } catch (...) { leftX = 0.0; }
    try { rightY = gamepad.GetRightStickY(); } catch (...) { rightY = 0.0; }
    try { rightY = gamepad.GetRightStickX(); } catch (...) { rightX = 0.0; }
    

    
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

void Robot::TestInit() {

  last_mode = {LOG_YELLOW, "[TEST]"};
  LOG_TEST("Enabled.");

}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif