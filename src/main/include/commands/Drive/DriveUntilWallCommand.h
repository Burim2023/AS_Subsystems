#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "subsystems/amcu/AMCU.h"
#include "subsystems/sensor/SensorManager.h"

class DriveUntilWallCommand
    : public frc2::CommandHelper<frc2::CommandBase, DriveUntilWallCommand>
{
public:
  DriveUntilWallCommand(AMCU *amcu,
                        SensorManager *sensorManager,
                        double usThresholdCm = 15.0,
                        double lidarThresholdCm = 28.0,
                        uint8_t driveSpeedCms = 20);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  AMCU *m_amcu{nullptr};
  SensorManager *m_sensorManager{nullptr};

  double m_usThresholdCm{15.0};
  double m_lidarThresholdCm{28.0};
  uint8_t m_driveSpeedCms{20};

  bool m_finished{false};
};