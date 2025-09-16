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
//StartStop StaSto(&lidar, &oi);
AMCU amcu;

void Robot::RobotInit() {
  // arm.Init();
  // gripper.Init();
  // extender.Init();
  // joint.Init();
  ultrasonic.Init();
  
  // Register ultrasonic subsystem with SmartDashboard for Sendable widgets
  frc::SmartDashboard::PutData("Ultrasonic Sensor", &ultrasonic);
  
  amcu.initOmniDriveBase(kWheelRadius, kRobotRadius, kMotorLeft, kMotorRight, kMotorBack);
  
}

void Robot::RobotPeriodic() { 
  frc2::CommandScheduler::GetInstance().Run();
  ultrasonic.Periodic();
  // arm.Periodic();
  // gripper.Periodic();
  // joint.Periodic();
  // extender.Periodic();
}

void Robot::DisabledInit() {
  amcu.stop();
  
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
  
  if (oi.GetDriveXButton()) {
      // Print current distance in centimeters when X button is pressed
      std::cout << "Current ultrasonic distance: " << distanceCm << " cm" << std::endl;
      //arm.SetHomePosition();
  }
  
  // Example: Warning system based on distance in centimeters
  if (distanceCm > 0 && distanceCm < 15.0) {
    std::cout << "WARNING: Obstacle detected at " << distanceCm << " cm!" << std::endl;
    frc::SmartDashboard::PutString("Status", "OBSTACLE DETECTED - " + std::to_string(distanceCm) + " cm");
  } else if (distanceCm > 0) {
    frc::SmartDashboard::PutString("Status", "Path Clear - " + std::to_string(distanceCm) + " cm");
  }
  
  // if (oi.GetDriveSquareButton()) {
  //     arm.SetDropApplePosition();
  // }
  // if (oi.GetDriveCircleButton()) {
  //     arm.SetPickApplePosition();
  // }
  // if (oi.GetDriveTriangleButton()) {
  //     //amcu.driveDistance(1,0,0);
  //     joint.SetGripperUpAngle();
  // }

  // if (oi.GetDriveRightBumper()) {
  //     gripper.SetOpenGripper();
  // }
  // if (oi.GetDriveLeftBumper()) {
  //     gripper.SetClosedGripper();
  // }

  // if (oi.getDriveLeftTrigger()) {
  //     extender.SetPickPostion();
  // }
  // if (oi.GetDriveRightTrigger()) {
  //     extender.SetDropPostion();
  // }

  // if (oi.GetDriveLeftAnalogButton()) {
  //     joint.SetGripperDownAngle();
  // }

  // if(oi.GetDriveRightAnalogButton()) {
  //     joint.SetGripperMidAngle();
  // }

  // if(oi.GetDriveShareButton()) {
  //     joint.SetGripperUpAngle();}
  
  

  //frc::SmartDashboard::PutNumber("Distance in cm", ultrasonic.GetDistance());
  
  // Display distance in centimeters (reuse the existing distanceCm variable)
  frc::SmartDashboard::PutNumber("Ultrasonic Distance (cm)", distanceCm);
  frc::SmartDashboard::PutString("Distance Reading", std::to_string(distanceCm) + " cm");


}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif