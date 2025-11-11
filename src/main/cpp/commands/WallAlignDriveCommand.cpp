#include "commands/WallAlignDriveCommand.h"
#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <iostream>

WallAlignDriveCommand::WallAlignDriveCommand(AMCU *amcu,
                                             SensorManager *sensorManager,
                                             double wallThresholdCm,
                                             uint8_t driveSpeedCms,
                                             uint8_t turnSpeedDegPerS,
                                             double turnSeconds)
    : m_amcu(amcu),
      m_sensorManager(sensorManager),
      m_wallThresholdCm(wallThresholdCm),
      m_driveSpeedCms(driveSpeedCms),
      m_turnSpeed(turnSpeedDegPerS),
      m_turnSeconds(turnSeconds)
{
  SetName("WallAlignDriveCommand");

  // Declare subsystem requirements
  AddRequirements({amcu, sensorManager});
}

void WallAlignDriveCommand::Initialize()
{
  m_finished = false;
  m_phase = Phase::DrivingToWall;
  m_childCmdOwned.reset();
  m_childCmdPtr = nullptr;

  // start the first forward-to-wall child
  m_childCmdOwned =
      std::make_unique<DriveUntilWallCommand>(m_amcu, m_sensorManager, m_wallThresholdCm, 28.0, m_driveSpeedCms);
  m_childCmdPtr = m_childCmdOwned.get();
  frc2::CommandScheduler::GetInstance().Schedule(m_childCmdPtr);
  std::cout << "WallAlign: scheduled DriveUntilWallCommand\n";
}

void WallAlignDriveCommand::Execute()
{
  // If no child is scheduled, schedule the next appropriate child for the current phase
  auto &sched = frc2::CommandScheduler::GetInstance();
  if (!m_amcu)
  {
    std::cerr << "ERROR: WallAlignDriveCommand - AMCU is null!" << std::endl;
    return; // Safe early return
  }
  if (!m_sensorManager)
  {
    std::cerr << "ERROR: WallAlignDriveCommand - SensorManager is null!" << std::endl;
    return;
  }

  auto *lidar = m_sensorManager->GetLidarSubsystem();
  if (!lidar)
  {
    std::cerr << "ERROR: LiDAR not available" << std::endl;
    return;
  }

  if (m_sensorManager && m_sensorManager->GetUltrasonicSubsystem())
  {
    // Sensors are updated in main thread via subsystem Periodic methods
    // double leftDistance = m_sensorManager->GetUltrasonicSubsystem()->GetLeftDistance();
    // double rightDistance = m_sensorManager->GetUltrasonicSubsystem()->GetRightDistance();

    // If child still running - nothing to do
    if (m_childCmdPtr && sched.IsScheduled(m_childCmdPtr))
    {
      return;
    }

    // child finished (or none scheduled) -> advance state machine
    switch (m_phase)
    {
    case Phase::DrivingToWall:
      // finished driving: schedule short stop, then turning
      m_phase = Phase::PauseAfterDrive;
      m_childCmdOwned = std::make_unique<SpeedDriveCommand>(m_amcu, m_shortPauseSeconds, 0, 0, 0);
      m_childCmdPtr = m_childCmdOwned.get();
      sched.Schedule(m_childCmdPtr);
      std::cout << "WallAlign: reached wall -> short pause\n";
      break;

    case Phase::PauseAfterDrive:
      // after short pause schedule turn (timed SpeedDriveCommand: rotation positive or negative)
      m_phase = Phase::Turning;
      // turn left by using negative rotation (or positive depending on robot)
      m_childCmdOwned = std::make_unique<SpeedDriveCommand>(m_amcu, m_turnSeconds, 0, 0, static_cast<uint8_t>(m_turnSpeed));
      m_childCmdPtr = m_childCmdOwned.get();
      sched.Schedule(m_childCmdPtr);
      std::cout << "WallAlign: starting turn\n";
      break;

    case Phase::Turning:
      // finished turn -> short pause then go drive again
      m_phase = Phase::PauseAfterTurn;
      m_childCmdOwned = std::make_unique<SpeedDriveCommand>(m_amcu, m_shortPauseSeconds, 0, 0, 0);
      m_childCmdPtr = m_childCmdOwned.get();
      sched.Schedule(m_childCmdPtr);
      std::cout << "WallAlign: finished turn -> short pause\n";
      break;

    case Phase::PauseAfterTurn:
      // after pause restart driving to wall
      m_phase = Phase::DrivingToWall;
      m_childCmdOwned =
          std::make_unique<DriveUntilWallCommand>(m_amcu, m_sensorManager, m_wallThresholdCm, 28.0, m_driveSpeedCms);
      m_childCmdPtr = m_childCmdOwned.get();
      sched.Schedule(m_childCmdPtr);
      std::cout << "WallAlign: restarting DriveUntilWall\n";
      break;

    case Phase::Idle:
    default:
      break;
    }
  }
}

void WallAlignDriveCommand::End(bool interrupted)
{
  // cancel any child
  if (m_childCmdPtr)
  {
    frc2::CommandScheduler::GetInstance().Cancel(m_childCmdPtr);
    m_childCmdPtr = nullptr;
  }
  m_childCmdOwned.reset();
  m_phase = Phase::Idle;
  m_finished = true;
  std::cout << "WallAlign: End (interrupted=" << interrupted << ")\n";
}

bool WallAlignDriveCommand::IsFinished() { return m_finished; }
