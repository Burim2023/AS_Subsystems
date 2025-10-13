#include "commands/RetractAndLift.h"
#include "commands/MoveArmToPosition.h"
#include "commands/ExtendForDuration.h"
#include "Constants.h"

RetractAndLift::RetractAndLift(ArmSubsystem* arm, ExtenderSubsystem* extender) {
    // These two commands will start at the same time
    // The group will finish when BOTH commands have finished
    AddCommands(
        // Move arm to home position
        MoveArmToPosition(arm, HOME_ANGLE),
        
        // Retract extender for 1.5 seconds
        ExtendForDuration(extender, 1.5, false) // false = counter-clockwise (retract)
    );
}