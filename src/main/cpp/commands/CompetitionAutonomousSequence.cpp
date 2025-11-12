#include "commands/CompetitionAutonomousSequence.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include "commands/DriveDistanceCommand.h"

CompetitionAutonomousSequence::CompetitionAutonomousSequence(AMCU* amcu,
                                                           ArmSubsystem* arm,
                                                           GripperSubsystem* gripper,
                                                           GripperJointSubsystem* gripperJoint,
                                                           CameraSubsystem* camera,
                                                           ElevatorSubsystem* elevator,
                                                           ExtenderSubsystem* extender)
{
    // Compose a competition-friendly autonomous routine using existing commands
    AddCommands(
        // Locate and pick an apple from the field
        // DriveSmartPickupGround(arm, gripper, gripperJoint, camera, elevator, amcu),
        std::move(CalibrateExtender(extender)),

        ExtendForDuration(extender, ExtendForDuration::Direction::RETRACT, 0.1, 2.0),
        
        frc2::WaitCommand(2.0_s),

        SmartPickSequence(arm, gripper, gripperJoint, camera, elevator),

        //frc2::WaitCommand(2.0_s),

        //storage::StoreSingleApple(extender, elevator, arm, gripperJoint, gripper, 0),
        // Store the picked apple into the requested storage slot
        //storage::StoreAppleCommand(extender, elevator, arm, gripperJoint, gripper, storeSlot),
        GripperOperate(gripperJoint, gripper, GripperOperate::Position::DOWN, false, 2.0),
        frc2::WaitCommand(2.0_s),
        //ExtendForDuration(extender, ExtendForDuration::Direction::RETRACT, 0.3, 2.0),

        GripperOperate(gripperJoint, gripper, GripperOperate::Position::DOWN, true, 2.0)
        

        //ExtendForDuration(extender, ExtendForDuration::Direction::EXTEND, 0.1, 2.0)
        // Drive forward to scoring area (distance-based for reliability)
        // Parameters: AMCU*, xMeters, yMeters, omega_degrees, timeoutSeconds
        // DriveDistanceCommand(amcu, 1 /*1m forward*/, 0, 0, 3.0),
        //SpeedDriveCommand(amcu, 4, 15, 0, 0)
        // Ensure gripper is opened to release object (move joint up + open gripper)
        
        // DriveDistanceCommand(amcu, 1 /*1m back - AMCU driveDistance likely interprets sign via API*/, 0, 0, 2.5)
    );
}
