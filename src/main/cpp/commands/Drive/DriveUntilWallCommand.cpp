#include "commands/Drive/DriveUntilWallCommand.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

DriveUntilWallCommand::DriveUntilWallCommand(AMCU *amcu,
                                             SensorManager *sensorManager,
                                             double usThresholdCm,
                                             double lidarThresholdCm,
                                             uint8_t driveSpeedCms)
    : m_amcu(amcu),
      m_sensorManager(sensorManager),
      m_usThresholdCm(usThresholdCm),
      m_lidarThresholdCm(lidarThresholdCm),
      m_driveSpeedCms(driveSpeedCms)
{
  SetName("DriveUntilWallCommand");
}

void DriveUntilWallCommand::SetAMCU(AMCU *amcu)
{
  m_amcu = amcu;
}

void DriveUntilWallCommand::SetSensorManager(SensorManager *sensorManager)
{
  m_sensorManager = sensorManager;
}

void DriveUntilWallCommand::Initialize()
{
  m_finished = false;
  if (m_amcu)
  {
    std::cout << "DriveUntilWall: starting forward at " << static_cast<int>(m_driveSpeedCms) << " cm/s\n";
    m_amcu->speedDrive(static_cast<int16_t>(m_driveSpeedCms), 0, 0);
  }
  else
  {
    std::cout << "DriveUntilWall: AMCU is null - cannot start driving\n";
  }
}

void DriveUntilWallCommand::Execute()
{
  double usDist = -1.0;
  double lidarDist = -1.0;

  if (m_sensorManager)
  {
    if (auto *us = m_sensorManager->GetUltrasonicSubsystem())
    {
      usDist = us->GetRightDistance();
      frc::SmartDashboard::PutNumber("DriveUntilWall US Right (cm)", usDist);
    }
    if (auto *lidar = m_sensorManager->GetLidarSubsystem())
    {
      lidarDist = lidar->GetDistanceAtAngle(0);
      frc::SmartDashboard::PutNumber("DriveUntilWall LiDAR (cm)", lidarDist);
    }
  }

  bool usHit = (usDist > 0 && usDist <= m_usThresholdCm);
  bool lidarHit = (lidarDist > 0 && lidarDist <= m_lidarThresholdCm);

  if (usHit || lidarHit)
  {
    std::cout << "DriveUntilWall: wall detected US=" << usDist << " LiDAR=" << lidarDist << " -> stopping\n";
    if (m_amcu)
      m_amcu->stop();
    m_finished = true;
  }
  else
  {
    if (m_amcu)
      m_amcu->speedDrive(static_cast<int16_t>(m_driveSpeedCms), 0, 0);
  }
}

bool DriveUntilWallCommand::IsFinished()
{
  return m_finished;
}

void DriveUntilWallCommand::End(bool interrupted)
{
  if (m_amcu)
    m_amcu->stop();
  std::cout << "DriveUntilWall: End (interrupted=" << (interrupted ? "true" : "false") << ")\n";
}