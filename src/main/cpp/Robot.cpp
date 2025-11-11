#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cmath>

#include "subsystems/amcu/AMCU.h"
#include "Constants.h"
#include "web-ds-logger/cpp/networktables/LoggingSystem.h"
#include "subsystems/sensor/SensorManager.h"

void Robot::RobotInit()
{
  SetupLogging();
  InitLogging(m_container.GetSensorManager());

  // Initialize drive base configuration
  m_container.GetAMCU()->initOmniDriveBase(
      Constants::kWheelRadius,
      Constants::kRobotRadius,
      Constants::kMotorLeft,
      Constants::kMotorRight,
      Constants::kMotorBack);

  InitializeStorageParameters();

  std::cout << "Robot: Initialization complete (no singleton pattern)" << std::endl;
}

// Storage Parameters
void Robot::InitializeStorageParameters()
{
  auto nt = nt::NetworkTableInstance::GetDefault().GetTable("Storage");

  // Set storage height to 200mm (optimal for apple dropping)
  nt->PutNumber("HeightMM", 200.0);

  // Set slot timing thresholds for pick time classification
  nt->PutNumber("T1", 1.5); // 0-1.5s = Slot 0 (red apples)
  nt->PutNumber("T2", 2.2); // 1.5-2.2s = Slot 1 (yellow apples), >2.2s = Slot 2 (green apples)

  // Set slot position fractions (time-based positioning)
  nt->PutNumber("Slot0Frac", 0.17); // 17% from back (closest to robot)
  nt->PutNumber("Slot1Frac", 0.50); // 50% from back (middle)
  nt->PutNumber("Slot2Frac", 0.83); // 83% from back (furthest from robot)

  // Set operational parameters
  nt->PutNumber("DropDwellMs", 500.0); // 500ms wait after gripper opens

  std::cout << "Storage parameters initialized:" << std::endl;
  std::cout << "  - Storage Height: 200mm" << std::endl;
  std::cout << "  - Slot 0 (Red): 17% extension, <1.5s pick time" << std::endl;
  std::cout << "  - Slot 1 (Yellow): 50% extension, 1.5-2.2s pick time" << std::endl;
  std::cout << "  - Slot 2 (Green): 83% extension, >2.2s pick time" << std::endl;
}

void Robot::RobotPeriodic()
{
  UpdateLogging(m_container.GetSensorManager());

  frc2::CommandScheduler::GetInstance().Run();

  // if (auto *lf = m_container.GetLineFollower())
  // {
  //   try
  //   {
  //     lf->update();
  //     lf->UpdateShuffleboard(10);
  //   }
  //   catch (const std::exception &e)
  //   {
  //     // Silently catch to prevent crashes, log occasionally
  //     static int errorCount = 0;
  //     if (++errorCount % 250 == 0)
  //     { // Log every 5 seconds at 50Hz
  //       std::cout << "LineFollower error: " << e.what() << std::endl;
  //     }
  //   }
  // }
}

void Robot::DisabledInit()
{
  LOG_DISABLED(" Disabled.");
}

void Robot::DisabledPeriodic()
{
}

void Robot::AutonomousInit()
{

  // Get the autonomous command from the container
  m_autonomousCommand = m_container.GetAutonomousCommand();

  // Schedule the autonomous command (if one was selected)
  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Schedule();
    LOG_INFO("Autonomous command scheduled")
  }

  last_mode = {LOG_PURPLE, "[AUTONOMOUS]"};
  LOG_AUTONOMOUS("Enabled");
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  last_mode = {LOG_CYAN, "[TELEOP]"};
  LOG_TELEOP("Enabled.");
}

void Robot::TeleopPeriodic()
{
  try
  {
    if (auto *lf = m_container.GetLineFollower())
    {
      lf->UpdateShuffleboard(10);
    }
    else
    {
      return;
    }
  }
  catch (const std::exception &e)
  {
    std::cout << e.what() << '\n';
  }
}

void Robot::TestInit()
{

  last_mode = {LOG_YELLOW, "[TEST]"};
  LOG_TEST("Enabled.");
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif