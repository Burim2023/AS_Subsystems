#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cmath>

#include "subsystems/amcu/AMCU.h"
#include "Constants.h"
#include "web-ds-logger/cpp/networktables/LoggingSystem.h"
#include "subsystems/sensor/SensorManager.h"

// #include "commands/SpeedDriveCommand.h"

// Define static members of Robot class

Robot *Robot::s_instance = nullptr;

void Robot::RobotInit()
{
  s_instance = this;
  m_sensormanager = std::make_unique<SensorManager>();
  m_amcu = std::make_unique<AMCU>();

  SetupLogging();
  InitLogging(m_sensormanager.get());

  m_sensormanager->InitializeSensors();
  m_sensormanager->SensorManagerStartThread();

  // Forward SensorManager and AMCU to RobotContainer
  m_container.SetSensorManager(m_sensormanager.get());
  m_container.SetAMCU(m_amcu.get());

  m_amcu->initOmniDriveBase(Constants::kWheelRadius, Constants::kRobotRadius, Constants::kMotorLeft, Constants::kMotorRight, Constants::kMotorBack);
}

void Robot::RobotPeriodic()
{
  UpdateLogging(m_sensormanager.get());

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
      lf->update();
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