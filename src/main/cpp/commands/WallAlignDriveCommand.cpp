#include "commands/WallAlignDriveCommand.h"
#include "commands/SpeedDriveCommand.h"
#include "commands/Drive/DriveUntilWallCommand.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/PerpetualCommand.h>
#include <iostream>

WallAlignDriveCommand::WallAlignDriveCommand(
    AMCU* amcu,
    frc::UltrasonicSubsystem* ultrasonic,
    frc::LidarSubsystem* lidar,
    double wallThresholdCm,
    uint8_t driveSpeed,
    uint8_t turnSpeed /* unused – we always send 30 as requested */) {

    SetName("WallAlignDriveCommand");
    m_amcu = amcu;
    m_turnLeft = true;

    AddCommands(
        // Build the repeating cycle at runtime so we use the live m_amcu
        frc2::InstantCommand([this, ultrasonic, lidar, wallThresholdCm, driveSpeed]() {
            if (!ultrasonic || !lidar) {
                std::cout << "[WallAlign] sensors missing\n";
                return;
            }

            // Use DriveUntilWallCommand for the forward phase (uses US + LiDAR)
            // lidarThreshold is left adjustable (use 28cm here as a good default)
            auto forwardCmd = DriveUntilWallCommand(m_amcu, ultrasonic, lidar,
                                                   wallThresholdCm, 28.0, static_cast<uint8_t>(driveSpeed));

            // short stop command (tiny duration) implemented with SpeedDriveCommand(0)
            auto stopShort = SpeedDriveCommand(m_amcu, 0.2, 0, 0, 0);

            // fixed left turn: 3s at w=30deg/s -> 90deg
            auto turnCmd = SpeedDriveCommand(m_amcu, 3.0, 0, 0, static_cast<uint8_t>(30));
            //auto stopAfterTurn = SpeedDriveCommand(m_amcu, 0.2, 0, 0, 0);

            frc2::SequentialCommandGroup cycle(
                // drive until wall detected
                forwardCmd,

                // stop briefly
                stopShort,

                // execute 90° left turn (timed)
                turnCmd,

                
                // stop after turn
                stopAfterTurn
            );

            // Repeat the cycle while scheduled
            this->AddCommands(frc2::PerpetualCommand(std::move(cycle)));
        })
    );
}

void WallAlignDriveCommand::SetAMCU(AMCU* amcu) {
    m_amcu = amcu;
}
