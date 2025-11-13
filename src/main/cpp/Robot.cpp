#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cmath>
#include <memory>

#include "subsystems/amcu/AMCU.h"
#include "Constants.h"
#include "web-ds-logger/cpp/networktables/LoggingSystem.h"
#include "subsystems/sensor/SensorManager.h"
#include "SignalHandler.h"
#include <networktables/NetworkTableInstance.h>

void Robot::RobotInit()
{
  InstallCrashHandler();
  m_sensormanager = std::make_unique<SensorManager>();
  SetupLogging(); 
  InitLogging();  
  m_sensormanager->InitializeSensors();

  // Create AMCU if not already constructed (Robot.h likely default constructs it)
  if (!m_amcu)
    m_amcu = std::make_unique<AMCU>();

  // Provide AMCU and sensor manager pointers to the container
  m_container.SetAMCU(m_amcu.get());
  m_container.SetSensorManager(m_sensormanager.get());

  m_amcu->initOmniDriveBase(Constants::kWheelRadius,
                            Constants::kRobotRadius,
                            Constants::kMotorLeft,
                            Constants::kMotorRight,
                            Constants::kMotorBack);

  // Initialize storage configuration values in NetworkTables
  InitializeStorageParameters();
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
  static bool lastSensorState = false;
  bool enableSensors = SensorManager::EnableSensorThread.load();

  if (enableSensors != lastSensorState)
  {
    if (enableSensors)
    {
      try
      {
        m_sensormanager->SensorManagerStartThread();
        std::cout << "[Robot] Sensor thread STARTED" << std::endl;
      }
      catch (const std::exception &e)
      {
        std::cerr << "[Robot] Failed to start sensor thread: " << e.what() << std::endl;
      }
    }
    else
    {
      try
      {
        m_sensormanager->SensorManagerStopThread();
        std::cout << "[Robot] Sensor thread STOPPED" << std::endl;
      }
      catch (const std::exception &e)
      {
        std::cerr << "[Robot] Failed to stop sensor thread: " << e.what() << std::endl;
      }
    }
    lastSensorState = enableSensors;
  }

  // Update logging - throttled to avoid mutex contention with sensor thread
  // UpdateLogging() now safely skips sensor reads when sensor thread is active
  static int loggingTick = 0;
  if (++loggingTick >= 10) // Every 200ms (10 * 20ms RobotPeriodic)
  {
    loggingTick = 0;
    UpdateLogging(m_sensormanager.get());
  }

  frc2::CommandScheduler::GetInstance().Run();

  // Only update line follower if sensor thread is NOT enabled
  // Use lastSensorState to avoid race condition during state transitions
  if (!lastSensorState)
  {
    if (auto *lf = m_container.GetLineFollower())
    {
      lf->update();
      lf->UpdateShuffleboard(10);
    }
  }
  else
  {
    // Sensor thread is running - publish NetworkTables from low-priority main loop
    // This moves NT updates OUT of the time-critical sensor thread
    static int shuffleboardTick = 0;
    if (++shuffleboardTick >= 10) // Every 200ms (10 * 20ms RobotPeriodic)
    {
      shuffleboardTick = 0;
      if (auto *lf = m_container.GetLineFollower())
      {
        lf->UpdateShuffleboard(1); // Force update (param=1 means update every call)
      }
    }
  }
}

void Robot::DisabledInit()
{
  try
  {
    if (m_amcu)
    {
      m_amcu->stop();
      m_amcu->setSpeed(MOTOR_1, 0);
      m_amcu->setSpeed(MOTOR_2, 0);
      m_amcu->setSpeed(MOTOR_3, 0);
      m_amcu->setSpeed(MOTOR_0, 0);
    }

    frc2::CommandScheduler::GetInstance().CancelAll();
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }

  LOG_DISABLED(" Disabled.");
}

void Robot::DisabledPeriodic()
{
  // nothing for now
}

void Robot::AutonomousInit()
{
  // Get the autonomous command from the container
  m_autonomousCommand = m_container.GetAutonomousCommand();
  // }
  // Schedule the autonomous command (if one was selected)
  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Schedule();
    LOG_INFO("Autonomous command scheduled");
  }

  if (m_amcu)
  {
    m_amcu->stop();
    m_amcu->setSpeed(MOTOR_1, 0);
    m_amcu->setSpeed(MOTOR_2, 0);
    m_amcu->setSpeed(MOTOR_3, 0);
  }

  last_mode = {LOG_PURPLE, "[AUTONOMOUS]"};
  LOG_AUTONOMOUS("Enabled");
}

void Robot::AutonomousPeriodic()
{
  // CommandScheduler runs from RobotPeriodic
}

void Robot::TeleopInit()
{
  last_mode = {LOG_CYAN, "[TELEOP]"};
  LOG_TELEOP("Enabled.");
}

void Robot::TeleopPeriodic()
{
  // try
  // {
  //   if (auto *lf = m_container.GetLineFollower())
  //   {
  //     lf->update();
  //     lf->UpdateShuffleboard(10);
  //   }
  //   else
  //   {
  //     return;
  //   }
  // }
  // catch (const std::exception &e)
  // {
  //   std::cout << e.what() << '\n';
  // }

  // if (gamepad.GetXButton()) {
  //   arm.SetServoAngleZero();
  // }
}

void Robot::TestInit()
{
  last_mode = {LOG_YELLOW, "[TEST]"};
  LOG_TEST("Enabled.");
}

void Robot::TestPeriodic()
{
  // no-op: keep same robot logic, used by WPILib during Test mode
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif