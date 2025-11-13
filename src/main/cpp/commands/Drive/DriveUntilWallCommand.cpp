#include "commands/Drive/DriveUntilWallCommand.h"
#include "subsystems/sensor/SensorManager.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

DriveUntilWallCommand::DriveUntilWallCommand(AMCU *amcu,
                                             SensorManager *sensorManager,
                                             double usThresholdCm,
                                            //  double lidarThresholdCm,
                                             double irThresholdCm,
                                             uint8_t driveSpeedCms)
    : m_amcu(amcu),
      m_sensorManager(sensorManager),
      m_usThresholdCm(usThresholdCm),
      // m_lidarThresholdCm(lidarThresholdCm),
      m_irThresholdCm(irThresholdCm),
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
  SensorManager::EnableSensorThread.store(true);
  std::this_thread::sleep_for(200ms);
  m_finished = false;
  if (m_amcu)
  {
    std::cout << "DriveUntilWall: starting forward at " << static_cast<int>(m_driveSpeedCms) << " cm/s\n";
    m_amcu->speedDrive(static_cast<int16_t>(-m_driveSpeedCms), 0, 0);
  }
  else
  {
    std::cout << "DriveUntilWall: AMCU is null - cannot start driving\n";
  }
}

void DriveUntilWallCommand::Execute()
{
  double usDistRight = -1.0;
  double usDistLeft = -1.0;
  double irDistRight = -1.0;
  double irDistLeft = -1.0;
  // double lidarDist = -1.0;

  if (m_sensorManager)
  {
    if (auto *us = m_sensorManager->GetUltrasonicSubsystem())
    {
      usDistRight = us->GetRightDistance();
      usDistLeft = us->GetLeftDistance();
      frc::SmartDashboard::PutNumber("DriveUntilWall US Right (cm)", usDistRight);
      frc::SmartDashboard::PutNumber("DriveUntilWall US Left (cm)", usDistLeft);

    }
    // if (auto *lidar = m_sensorManager->GetLidarSubsystem())
    // {
    //   lidarDist = lidar->GetDistanceAtAngle(0);
    //   frc::SmartDashboard::PutNumber("DriveUntilWall LiDAR (cm)", lidarDist);
    // }
    if (auto *ir = m_sensorManager->GetIRRangeSubsystem())
    {
      irDistRight = ir->GetIRRightDistance();
      irDistLeft = ir->GetIRLeftDistance();
    }
  }

  bool irHit = (irDistRight > 0 && irDistRight <= m_irThresholdCm || irDistLeft > 0 && irDistLeft <= m_irThresholdCm);
  bool usHit = (usDistRight > 0 && usDistRight <= m_usThresholdCm || usDistLeft > 0 && usDistLeft <= m_usThresholdCm);
  // bool lidarHit = (lidarDist > 0 && lidarDist <= m_lidarThresholdCm);

  if (usHit || irHit)
  {
    std::cout << "DriveUntilWall: wall detected US-Right=" << usDistRight << "US-Left=" << usDistLeft << "IR-Right=" << irDistRight << "IR-Left=" << irDistLeft << " -> stopping\n";
    if (m_amcu)
      m_amcu->stop();
    m_finished = true;
  }
  else
  {
    if (m_amcu)
      m_amcu->speedDrive(static_cast<int16_t>(-m_driveSpeedCms), 0, 0);
  }
}

bool DriveUntilWallCommand::IsFinished()
{
  SensorManager::EnableSensorThread.store(true);
  return m_finished;
}

void DriveUntilWallCommand::End(bool interrupted)
{
  if (m_amcu)
    m_amcu->stop();
  std::cout << "DriveUntilWall: End (interrupted=" << (interrupted ? "true" : "false") << ")\n";
}