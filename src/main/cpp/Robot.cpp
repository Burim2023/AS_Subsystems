#include "frc/TimedRobot.h"
#include "Robot.h"
#include "AMCU.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

//subsystems
#include "subsystems/ArmSubsystem.h"
#include "subsystems/GripperSubsystem.h"
#include "subsystems/ExtenderSubsystem.h"
#include "subsystems/GripperJointSubsystem.h"


constexpr int kWheelRadius = 50;
constexpr int kRobotRadius = 162;
constexpr Motor kMotorLeft  = MOTOR_0;
constexpr Motor kMotorRight = MOTOR_2;
constexpr Motor kMotorBack  = MOTOR_3;

ArmSubsystem arm;
GripperSubsystem gripper;
GripperJointSubsystem joint;
ExtenderSubsystem extender;
OI oi;
//StartStop StaSto(&lidar, &oi);
AMCU amcu;

void Robot::RobotInit() {
  arm.Init();
  gripper.Init();
  extender.Init();
  joint.Init();
  amcu.initOmniDriveBase(kWheelRadius, kRobotRadius, kMotorLeft, kMotorRight, kMotorBack);
  
}

void Robot::RobotPeriodic() { 
  frc2::CommandScheduler::GetInstance().Run(); 
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
  
  //arm.IncreasePosition();
  

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
  if (oi.GetDriveXButton()) {
      arm.SetHomePosition();
  }
  if (oi.GetDriveSquareButton()) {
      arm.SetDropApplePosition();
  }
  if (oi.GetDriveCircleButton()) {
      arm.SetPickApplePosition();
  }
  if (oi.GetDriveTriangleButton()) {
      //amcu.driveDistance(1,0,0);
      joint.SetGripperUpAngle();
  }

  if (oi.GetDriveRightBumper()) {
      gripper.SetOpenGripper();
  }
  if (oi.GetDriveLeftBumper()) {
      gripper.SetClosedGripper();
  }

  if (oi.getDriveLeftTrigger()) {
      extender.SetPickPostion();
  }
  if (oi.GetDriveRightTrigger()) {
      extender.SetDropPostion();
  }

  if (oi.GetDriveLeftAnalogButton()) {
      joint.SetGripperDownAngle();
  }

  if(oi.GetDriveRightAnalogButton()) {
      joint.SetGripperMidAngle();
  }

  if(oi.GetDriveShareButton()) {
      joint.SetGripperUpAngle();
  }


}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif