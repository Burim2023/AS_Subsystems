#include "commands/GripperPickupSequence.h"
#include "Constants.h"
#include <iostream>

GripperPickupSequence::GripperPickupSequence(GripperJointSubsystem* gripperJoint, GripperSubsystem* gripper) {
    // Set requirements for both subsystems
    AddRequirements({gripperJoint, gripper});
    
    // Build the command sequence
    AddCommands(
        // === PHASE 1: DOWN POSITION ===
        // Move gripper joint to down position
        MoveGripperJointToPosition(gripperJoint, JOINT_DOWN_ANGLE),
        
        // Open gripper
        frc2::InstantCommand([gripper] { 
            gripper->SetOpenGripper(); 
            std::cout << "GripperSequence: Opening gripper at DOWN position" << std::endl;
        }, {gripper}),
        
        // Wait 1 second
        frc2::WaitCommand(1.0_s),
        
        // Close gripper
        frc2::InstantCommand([gripper] { 
            gripper->SetClosedGripper(); 
            std::cout << "GripperSequence: Closing gripper at DOWN position" << std::endl;
        }, {gripper}),
        
        // === PHASE 2: MID POSITION ===
        // Move gripper joint to mid position
        MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
        
        // Open gripper
        frc2::InstantCommand([gripper] { 
            gripper->SetOpenGripper(); 
            std::cout << "GripperSequence: Opening gripper at MID position" << std::endl;
        }, {gripper}),
        
        // Wait 1 second
        frc2::WaitCommand(1.0_s),
        
        // Close gripper
        frc2::InstantCommand([gripper] { 
            gripper->SetClosedGripper(); 
            std::cout << "GripperSequence: Closing gripper at MID position" << std::endl;
        }, {gripper}),
        
        // === PHASE 3: UP POSITION ===
        // Move gripper joint to up position
        MoveGripperJointToPosition(gripperJoint, JOINT_UP_ANGLE),
        
        // Open gripper
        frc2::InstantCommand([gripper] { 
            gripper->SetOpenGripper(); 
            std::cout << "GripperSequence: Opening gripper at UP position" << std::endl;
        }, {gripper}),
        
        // Wait 1 second
        frc2::WaitCommand(1.0_s),
        
        // Close gripper
        frc2::InstantCommand([gripper] { 
            gripper->SetClosedGripper(); 
            std::cout << "GripperSequence: Closing gripper at UP position" << std::endl;
        }, {gripper}),
        
        // Final message
        frc2::InstantCommand([] { 
            std::cout << "GripperSequence: Complete!" << std::endl;
        })
    );
}