#include "commands/TestCommandSequence.h"
#include "subsystems/amcu/AMCU.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PrintCommand.h>

TestCommandSequence::TestCommandSequence(AMCU *amcu) : m_amcu(amcu)
{
    // Declare subsystem requirement for proper scheduler management
    AddRequirements({amcu});

    AddCommands(
        frc2::PrintCommand("🎯 Starting AMCU Drive Test Sequence..."),
        SpeedDriveCommand(m_amcu, 2.0, 50, 0, 0), // Forward at 50 cm/s for 2s
        frc2::WaitCommand(1.0_s),
        SpeedDriveCommand(m_amcu, 2.0, -50, 0, 0), // Backward
        frc2::PrintCommand("✅ AMCU Drive Test Complete!"));
}