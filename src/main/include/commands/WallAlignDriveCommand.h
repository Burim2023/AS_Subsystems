#pragma once

#include <memory>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "AMCU.h"
#include "subsystems/UltrasonicSubsystem.h"
#include "subsystems/Lidar.h"
#include "commands/Drive/DriveUntilWallCommand.h"
#include "commands/SpeedDriveCommand.h"

class WallAlignDriveCommand
    : public frc2::CommandHelper<frc2::CommandBase, WallAlignDriveCommand> {
 public:
  WallAlignDriveCommand(AMCU* amcu,
                        frc::UltrasonicSubsystem* ultrasonic,
                        frc::LidarSubsystem* lidar,
                        double wallThresholdCm,
                        uint8_t driveSpeedCms,
                        uint8_t turnSpeedDegPerS,
                        double turnSeconds = 3.0);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  void SetAMCU(AMCU* amcu);

 private:
  enum class Phase { Idle, DrivingToWall, PauseAfterDrive, Turning, PauseAfterTurn };

  AMCU* m_amcu{nullptr};
  frc::UltrasonicSubsystem* m_ultrasonic{nullptr};
  frc::LidarSubsystem* m_lidar{nullptr};

  double m_wallThresholdCm{28.0};
  uint8_t m_driveSpeedCms{20};
  uint8_t m_turnSpeed{30};
  double m_turnSeconds{3.0};

  Phase m_phase{Phase::Idle};

  // currently scheduled child command (owned so it lives while scheduled)
  std::unique_ptr<frc2::Command> m_childCmdOwned;
  frc2::Command* m_childCmdPtr{nullptr};

  // small pause timers (use SpeedDriveCommand timeout for short stops)
  double m_shortPauseSeconds{0.12};

  bool m_finished{false};
};