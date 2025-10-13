#include "commands/TestCommandSequence.h"
#include "commands/DriveForDuration.h"
#include "AMCU.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PrintCommand.h>

TestCommandSequence::TestCommandSequence(AMCU* amcu, ArmSubsystem* arm, ExtenderSubsystem* extender) {
    // Simple test sequence to verify AMCU drive commands work
    AddCommands(
        // Start message
        frc2::PrintCommand("🎯 Starting AMCU Drive Test Sequence..."),

        // 1. Drive forward for 2 seconds
        DriveForDuration(amcu, 2.0, 0.3, 0.0, 0.0), // forward at 30% speed

        // 2. Wait for 1 second
        frc2::WaitCommand(1.0_s),

        // 3. Drive backward for 2 seconds
        DriveForDuration(amcu, 2.0, -0.3, 0.0, 0.0), // backward at 30% speed

        // 4. Wait for 1 second
        frc2::WaitCommand(1.0_s),

        // 5. Strafe right for 2 seconds
        DriveForDuration(amcu, 2.0, 0.0, 0.3, 0.0), // strafe right at 30% speed

        // 6. Wait for 1 second
        frc2::WaitCommand(1.0_s),

        // 7. Strafe left for 2 seconds
        DriveForDuration(amcu, 2.0, 0.0, -0.3, 0.0), // strafe left at 30% speed

        // 8. Wait for 1 second
        frc2::WaitCommand(1.0_s),

        // 9. Rotate clockwise for 1.5 seconds
        DriveForDuration(amcu, 1.5, 0.0, 0.0, 0.3), // rotate clockwise at 30% speed

        // 10. Wait for 0.5 seconds
        frc2::WaitCommand(0.5_s),

        // 11. Rotate counter-clockwise for 1.5 seconds
        DriveForDuration(amcu, 1.5, 0.0, 0.0, -0.3), // rotate counter-clockwise at 30% speed

        // End message
        frc2::PrintCommand("✅ AMCU Drive Test Complete! Command framework is working!")
    );
}