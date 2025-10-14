#include "commands/FullPickSequence.h"
#include "commands/MoveArmToPosition.h"
#include "commands/MoveGripperToPosition.h"
#include "commands/ExtendForDuration.h"
#include "Constants.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PrintCommand.h>

FullPickSequence::FullPickSequence(ArmSubsystem* arm, ExtenderSubsystem* extender, GripperJointSubsystem* gripperJoint) {
    // Build the sequence of commands
    AddCommands(
        // Start message
        frc2::PrintCommand("Starting Full Pick Sequence..."),

        // 1. Open the gripperJoint to prepare for pickup
        MoveGripperToPosition(gripperJoint, JOINT_UP_ANGLE), // Assuming UP is "open"

        // 2. Move the arm to the pick-up position
        MoveArmToPosition(arm, PICK_APPLE_ANGLE),

        // 3. Extend the arm for 1.2 seconds
        //ExtendForDuration(extender, 1.2, true), // true = clockwise (extend)

        // 4. tilt the gripper to grab the object
        MoveGripperToPosition(gripperJoint, JOINT_DOWN_ANGLE), // Assuming DOWN is "closed"

        // 5. Wait for half a second to ensure a firm grip
        frc2::WaitCommand(0.5_s),

        // 6. Retract the arm for 1.0 second
        //ExtendForDuration(extender, 1.0, false), // false = counter-clockwise (retract)

        // 7. Move the arm back to a safe "home" position
        MoveArmToPosition(arm, HOME_ANGLE),

        // End message
        frc2::PrintCommand("Full Pick Sequence Complete!")
    );
}