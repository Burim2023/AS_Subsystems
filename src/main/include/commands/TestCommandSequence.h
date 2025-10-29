#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include "commands/SpeedDriveCommand.h"
#include "commands/SimpleDrive.h"

// Forward declarations to avoid multiple includes
class AMCU;
//class ArmSubsystem;
//class ExtenderSubsystem;

/**
 * Simple test sequence to verify the AMCU drive commands are working:
 * 1. Drive forward for 2 seconds
 * 2. Drive backward for 2 seconds  
 * 3. Strafe right for 2 seconds
 * 4. Strafe left for 2 seconds
 * 5. Rotate clockwise for 1.5 seconds
 * 6. Rotate counter-clockwise for 1.5 seconds
 */
class TestCommandSequence : public frc2::SequentialCommandGroup {
public:
    /**
     * Constructor - Only requires AMCU for drive testing
     * @param amcu Pointer to the AMCU drivetrain
     * Pointer to the ArmSubsystem (unused in this test)
     * Pointer to the ExtenderSubsystem (unused in this test)
     */
    //TestCommandSequence(AMCU* amcu, ArmSubsystem* arm, ExtenderSubsystem* extender);
    TestCommandSequence(AMCU* amcu);
    void SetAMCU(AMCU* amcu);

private: 
    AMCU* m_amcu;
};