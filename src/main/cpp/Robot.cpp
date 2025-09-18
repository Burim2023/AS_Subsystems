#include "frc/TimedRobot.h"
#include "Robot.h"
#include "AMCU.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

//subsystems
// #include "subsystems/ArmSubsystem.h"
// #include "subsystems/GripperSubsystem.h"
// #include "subsystems/ExtenderSubsystem.h"
// #include "subsystems/GripperJointSubsystem.h"
#include "subsystems/UltrasonicSubsystem.h"
#include "subsystems/IRRangeSubsystem.h"
#include "subsystems/Lidar.h"
#include "utilities/LoggingSystem.h"


constexpr int kWheelRadius = 55;
constexpr int kRobotRadius = 162;
constexpr Motor kMotorLeft  = MOTOR_0;
constexpr Motor kMotorRight = MOTOR_2;
constexpr Motor kMotorBack  = MOTOR_3;

// ArmSubsystem arm;
// GripperSubsystem gripper;
// GripperJointSubsystem joint;
// ExtenderSubsystem extender;
OI oi;
frc::UltrasonicSubsystem ultrasonic;
frc::IRRangeSubsystem irRange(1);  // Using analog port 1
frc::LidarSubsystem lidar(studica::Lidar::kUSB1);  // Using Studica USB1 port
//StartStop StaSto(&lidar, &oi);
AMCU amcu;

void Robot::RobotInit() {
  // arm.Init();
  // gripper.Init();
  // extender.Init();
  // joint.Init();
  ultrasonic.Init();
  irRange.Init();
  lidar.Init();
  lidar.StartScan();  // Start LiDAR scanning
  SetupLogging();
  
  // Register subsystems with SmartDashboard for Sendable widgets
  frc::SmartDashboard::PutData("Ultrasonic Sensor", &ultrasonic);
  frc::SmartDashboard::PutData("IR Range Sensor", &irRange);
  frc::SmartDashboard::PutData("LiDAR Sensor", &lidar);
  
  amcu.initOmniDriveBase(kWheelRadius, kRobotRadius, kMotorLeft, kMotorRight, kMotorBack);
  
}

void Robot::RobotPeriodic() { 
  frc2::CommandScheduler::GetInstance().Run();
  ultrasonic.Periodic();
  irRange.Periodic();
  lidar.Periodic();
  // arm.Periodic();
  // gripper.Periodic();
  // joint.Periodic();
  // extender.Periodic();
}

void Robot::DisabledInit() {
  amcu.stop();
  lidar.StopScan();  // Stop LiDAR scanning when disabled
  
}
void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {

  

  //amcu.setSpeed(MOTOR_1, 10);

  

  amcu.driveDistance(1, 0, 0); // Example: drive 2 meters in x
}

void Robot::AutonomousPeriodic() {
  
  // Get ultrasonic distance for autonomous navigation (in centimeters)
  double distanceCm = ultrasonic.GetDistance();
  bool wallDetected = ultrasonic.IsWallDetected(30.0); // 30cm threshold
  
  //arm.IncreasePosition();
  
  // Example autonomous logic using distance in centimeters
  if (distanceCm > 0) { // Valid reading
    if (wallDetected) {
      std::cout << "WALL DETECTED! Distance: " << distanceCm << " cm - STOPPING" << std::endl;
      amcu.stop();
    } else {
      std::cout << "Clear path. Distance: " << distanceCm << " cm - CONTINUING" << std::endl;
      // Continue driving forward if path is clear
    }
  }
  
  // Display distance in SmartDashboard in centimeters
  frc::SmartDashboard::PutNumber("Auto Distance (cm)", distanceCm);

  wpi::outs() << "example\n";
  std::cout << "test";
  frc::SmartDashboard::PutNumber("Encoder Left", amcu.getEncoder(kMotorLeft));
  frc::SmartDashboard::PutNumber("Encoder Right", amcu.getEncoder(kMotorRight));
  frc::SmartDashboard::PutNumber("Encoder Back", amcu.getEncoder(kMotorBack));
}

void Robot::TeleopInit() {
  // If you use command-based, make sure to stop auto commands here
  /*
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  */

}

void Robot::TeleopPeriodic() {
  
  // Get distance data in centimeters for manual control assistance
  double distanceCm = ultrasonic.GetDistance();
  double irDistanceCm = irRange.GetDistance();
  double lidarFrontCm = lidar.GetFrontDistance();
  double IRVoltage = irRange.GetVoltage();
  
  if (oi.GetDriveXButton()) {
      // Print current distances when X button is pressed
      std::cout << "Ultrasonic distance: " << distanceCm << " cm" << std::endl;
      std::cout << "IR Range distance: " << irDistanceCm << " cm" << std::endl;
      std::cout << "LiDAR front distance: " << lidarFrontCm << " cm" << std::endl;
      std::cout << "IR range Voltage:" << IRVoltage << "cm" << std::endl;
      //arm.SetHomePosition();
  }
  
  if (oi.GetDriveTriangleButton()) {
      // Restart LiDAR when Triangle button is pressed
      std::cout << "Manual LiDAR restart requested" << std::endl;
      lidar.RestartScan();
  }
  
  // Example: Warning system based on all sensors
  if (distanceCm > 0 && distanceCm < 15.0) {
    std::cout << "WARNING: Ultrasonic obstacle at " << distanceCm << " cm!" << std::endl;
    frc::SmartDashboard::PutString("Ultrasonic Status", "OBSTACLE - " + std::to_string(distanceCm) + " cm");
  } else if (distanceCm > 0) {
    frc::SmartDashboard::PutString("Ultrasonic Status", "Clear - " + std::to_string(distanceCm) + " cm");
  }
  
  if (irRange.IsObjectDetected(25.0)) {
    std::cout << "WARNING: IR sensor detects object at " << irDistanceCm << " cm!" << std::endl;
    frc::SmartDashboard::PutString("IR Status", "OBJECT DETECTED - " + std::to_string(irDistanceCm) + " cm");
  } else if (irRange.IsValidReading()) {
    frc::SmartDashboard::PutString("IR Status", "Clear - " + std::to_string(irDistanceCm) + " cm");
  }
  
  // LiDAR-based warnings and navigation assistance
  if (!lidar.IsPathClear(100.0)) {
    std::cout << "WARNING: LiDAR detects obstacle in path at " << lidarFrontCm << " cm!" << std::endl;
    frc::SmartDashboard::PutString("LiDAR Status", "OBSTACLE IN PATH - " + std::to_string(lidarFrontCm) + " cm");
  } else {
    frc::SmartDashboard::PutString("LiDAR Status", "Path Clear - " + std::to_string(lidarFrontCm) + " cm");
  }
  
  // Additional LiDAR directional information
  if (oi.GetDriveSquareButton()) {
    // Print all directional LiDAR readings when Square button is pressed
    std::cout << "LiDAR Directions - Front: " << lidarFrontCm 
              << ", Left: " << lidar.GetDistanceAtAngle(90) 
              << ", Right: " << lidar.GetDistanceAtAngle(270)
              << ", Rear: " << lidar.GetDistanceAtAngle(180) << " cm" << std::endl;
  }
  
  
  // Display distance in centimeters (reuse the existing distanceCm variable)
  // frc::SmartDashboard::PutNumber("Ultrasonic Distance (cm)", distanceCm);
  // frc::SmartDashboard::PutString("Distance Reading", std::to_string(distanceCm) + " cm");


}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif