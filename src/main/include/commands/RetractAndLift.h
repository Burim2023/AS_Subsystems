#pragma once

#include <frc2/command/ParallelCommandGroup.h>
#include "subsystems/elevator/ArmSubsystem.h"
#include "subsystems/elevator/ExtenderSubsystem.h"

/**
 * A parallel command group that retracts the extender while moving the arm to home.
 * Both actions happen simultaneously, making the sequence more efficient.
 */
class RetractAndLift : public frc2::ParallelCommandGroup {
public:
    /**
     * Constructor
     * @param arm Pointer to the ArmSubsystem
     * @param extender Pointer to the ExtenderSubsystem
     */
    RetractAndLift(ArmSubsystem* arm, ExtenderSubsystem* extender);
};