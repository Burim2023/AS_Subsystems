#include "commands/TestCommandSequence.h"
//#include "commands/DriveForDuration.h"
#include "AMCU.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PrintCommand.h>

TestCommandSequence::TestCommandSequence(AMCU* amcu) : m_amcu(amcu) {
    AddCommands(
        frc2::PrintCommand("🎯 Starting AMCU Drive Test Sequence..."),
        // Replace SimpleDrive with SpeedDriveCommand for testing
        SpeedDriveCommand(m_amcu, 2.0, 50, 0, 0),  // Forward at 50 cm/s for 2s
        frc2::WaitCommand(1.0_s),
        // Add more tests if needed, e.g., backward
        SpeedDriveCommand(m_amcu, 2.0, -50, 0, 0),  // Backward
        frc2::PrintCommand("✅ AMCU Drive Test Complete!")
    );
}

void TestCommandSequence::SetAMCU(AMCU* amcu) {
    m_amcu = amcu;
}