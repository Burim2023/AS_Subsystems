#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/sensor/UltrasonicSubsystem.h"
#include "subsystems/sensor/Lidar.h"
#include "subsystems/amcu/AMCU.h"

// Forward-declare WPILib sensors to avoid heavy includes
namespace frc { class UltrasonicSubsystem; class LidarSubsystem; }

class WallAlignDriveCommand : public frc2::SequentialCommandGroup {
 public:
  // Constructor: keep signature so RobotContainer can construct it
  WallAlignDriveCommand(AMCU* amcu, frc::UltrasonicSubsystem* ultrasonic, frc::LidarSubsystem* lidar,
                        double wallThresholdCm, uint8_t driveSpeed, uint8_t turnSpeed);

  // Method to set the AMCU instance (injection after Robot starts)
  void SetAMCU(AMCU* amcu);

 private:
  AMCU* m_amcu = nullptr;
  bool m_turnLeft = false; // store turn decision for runtime use
};