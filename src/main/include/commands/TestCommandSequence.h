#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include "commands/SpeedDriveCommand.h"
#include "commands/SimpleDrive.h"

// Forward declarations to avoid multiple includes
class AMCU;

/**
 * Simple test sequence to verify the AMCU drive commands are working
 *
 * REFACTORED: Now properly declares subsystem requirements
 */
class TestCommandSequence : public frc2::SequentialCommandGroup
{
public:
    /**
     * Constructor
     * @param amcu Pointer to the AMCU drivetrain subsystem
     */
    explicit TestCommandSequence(AMCU *amcu);

private:
    AMCU *m_amcu;
};