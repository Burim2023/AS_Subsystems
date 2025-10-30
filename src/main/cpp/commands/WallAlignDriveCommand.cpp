#include "commands/WallAlignDriveCommand.h"
#include "commands/SpeedDriveCommand.h"
#include <frc2/command/InstantCommand.h>
#include <iostream>

WallAlignDriveCommand::WallAlignDriveCommand(AMCU* amcu, frc::UltrasonicSubsystem* ultrasonic, frc::LidarSubsystem* lidar, 
                                             double wallThresholdCm, uint8_t driveSpeed, uint8_t turnSpeed) {
    SetName("WallAlignDriveCommand");

    // store the provided pointer (may be nullptr; RobotContainer should call SetAMCU later)
    m_amcu = amcu;
    m_turnLeft = false;

    // Phase 1: Check sensors and decide turn direction, then create & append the SpeedDriveCommands at runtime
    AddCommands(
        frc2::InstantCommand([this, ultrasonic, lidar, wallThresholdCm, driveSpeed, turnSpeed]() {
            // decide turn direction
            if (!ultrasonic || !lidar) {
                std::cout << "WallAlignDriveCommand: sensors missing, defaulting to no turn\n";
                m_turnLeft = false;
            } else {
                double leftDist = ultrasonic->GetLeftDistance();
                double rightDist = ultrasonic->GetRightDistance();
                double frontDist = lidar->GetFrontDistance();

                bool leftWall = leftDist > 0 && leftDist < wallThresholdCm;
                bool rightWall = rightDist > 0 && rightDist < wallThresholdCm;
                bool frontWall = frontDist > 0 && frontDist < wallThresholdCm;

                if (frontWall) {
                    m_turnLeft = leftWall && !rightWall;
                } else if (leftWall && rightWall) {
                    m_turnLeft = true;
                } else {
                    m_turnLeft = false;
                }

                std::cout << "WallAlignDriveCommand: Left=" << leftDist << " Right=" << rightDist
                          << " Front=" << frontDist << " => turnLeft=" << m_turnLeft << std::endl;
            }

            // Build SpeedDriveCommand objects now that turn decision is known and (hopefully) m_amcu is set
            if (!m_amcu) {
                std::cout << "WallAlignDriveCommand: warning: AMCU is null when creating SpeedDriveCommand\n";
            }

            int16_t rot = m_turnLeft ? -static_cast<int16_t>(turnSpeed) : static_cast<int16_t>(turnSpeed);

            // Append the turn command then forward command to this SequentialCommandGroup
            this->AddCommands(
                SpeedDriveCommand(m_amcu, 3.0, 0, 0, rot),           // timed turn
                SpeedDriveCommand(m_amcu, 10.0, static_cast<int16_t>(driveSpeed), 0, static_cast<int16_t>(0)) // forward
            );
        })
    );
}

// Method to set the AMCU instance (can be called after construction)
void WallAlignDriveCommand::SetAMCU(AMCU* amcu) {
    m_amcu = amcu;
}