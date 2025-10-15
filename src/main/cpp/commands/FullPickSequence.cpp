#include "commands/FullPickSequence.h"
#include "commands/MoveArmToPosition.h"
#include "commands/MoveGripperJointToPosition.h"
//#include "commands/ExtendForDuration.h"
#include "commands/GripperOperate.h"
#include "commands/MoveElevatorToPosition.h"
#include "commands/CalibrateElevator.h"

#include "Constants.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/ParallelCommandGroup.h>

FullPickSequence::FullPickSequence(ArmSubsystem* arm, GripperSubsystem* gripper, GripperJointSubsystem* gripperJoint, ElevatorSubsystem* elevator) {
    // Build the sequence of commands
    AddCommands(
        // Start message
        frc2::PrintCommand("Starting Full Pick Sequence..."),

        // 1. Open the gripperJoint to prepare for pickup
       // MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE), // Assuming UP is "open"

        // 2. Move the arm to the pick-up position
        MoveArmToPosition(arm, PICK_APPLE_ANGLE),

        frc2::WaitCommand(1.5_s),

        frc2::ParallelCommandGroup(
            // Move the gripperJoint to MID position while calibrating the elevator
            
            CalibrateElevator(elevator, 10.0),
            MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE)
        ),
        // 3. Extend the arm for 1.2 seconds
        //ExtendForDuration(extender, 1.2, true), // true = clockwise (extend)

        // 4. tilt the gripper to grab the object
        

        // 5. Wait for half a second to ensure a firm grip
        frc2::WaitCommand(0.5_s),

        MoveGripperJointToPosition(gripperJoint, JOINT_DOWN_ANGLE),

        MoveElevatorToPosition(elevator, 60.0f, 2.0f),
        
        frc2::WaitCommand(0.5_s),

        frc2::InstantCommand([gripper] { 
            gripper->SetOpenGripper(); 
            std::cout << "GripperSequence: Opening gripper at DOWN position" << std::endl;
        }, {gripper}),

        frc2::WaitCommand(2.0_s),

        frc2::InstantCommand([gripper] { 
            gripper->SetClosedGripper(); 
            std::cout << "GripperSequence: Opening gripper at DOWN position" << std::endl;
        }, {gripper}),

        frc2::WaitCommand(2.0_s),

        MoveElevatorToPosition(elevator, 175.0f, 1.0f),

        // 6. Retract the arm for 1.0 second
        //ExtendForDuration(extender, 1.0, false), // false = counter-clockwise (retract)

        // 7. Move the arm back to a safe "home" position
        MoveArmToPosition(arm, HOME_ANGLE),

        // End message
        frc2::PrintCommand("Full Pick Sequence Complete!")
    );
}