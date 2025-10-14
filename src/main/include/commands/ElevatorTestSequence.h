#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PrintCommand.h>
#include "commands/CalibrateElevator.h"
#include "commands/ElevatorPresets.h"
#include "subsystems/ElevatorSubsystem.h"

/**
 * Command sequence that demonstrates elevator operation:
 * 1. Calibrate elevator (find zero position)
 * 2. Move to LOW position (25mm)
 * 3. Move to MEDIUM position (50mm) 
 * 4. Move to HIGH position (75mm)
 * 
 * Each movement includes a brief pause to show the position clearly.
 */
class ElevatorTestSequence : public frc2::SequentialCommandGroup {
public:
    /**
     * Constructor
     * @param elevator Pointer to the ElevatorSubsystem
     */
    ElevatorTestSequence(ElevatorSubsystem* elevator);
};