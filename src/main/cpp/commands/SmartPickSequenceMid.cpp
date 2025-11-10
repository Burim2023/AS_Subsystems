#include "commands/SmartPickSequenceMid.h"
#include "commands/MoveArmToPosition.h"
#include "commands/MoveGripperJointToPosition.h"
#include "commands/MoveElevatorToPosition.h"
#include "commands/CalibrateElevator.h"
#include "commands/ElevatorPresets.h"
#include "commands/AppleGripperCheckCommand.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

SmartPickSequenceMid::SmartPickSequenceMid(ArmSubsystem* arm, 
                                          GripperSubsystem* gripper, 
                                          GripperJointSubsystem* gripperJoint,
                                          CameraSubsystem* camera,
                                          ElevatorSubsystem* elevator) {
    
    SetName("SmartPickSequenceMid");

    AddCommands(
        frc2::PrintCommand("🍎 Starting Smart Pick Sequence (MID LEVEL)..."),
        
        frc2::PrintCommand("🔄 Resetting gripper to mid position..."),
        MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
        frc2::WaitCommand(2.5_s),

        MoveArmToPosition(arm, PICK_APPLE_ANGLE),
        frc2::WaitCommand(4.0_s),

        frc2::PrintCommand("🔧 Starting elevator calibration..."),
        CalibrateElevator(elevator, 10.0),
        frc2::PrintCommand("✅ Calibration complete - positioning for MID detection"),

        // === MID LEVEL POSITIONING ===
        frc2::PrintCommand("Moving to MID apple pickup position..."),
        MoveElevatorToPosition(elevator, 80.0f, 5.0f),  // MID: Between ground and elevated
        frc2::WaitCommand(3.0_s),

        MoveGripperJointToPosition(gripperJoint, JOINT_DOWN_ANGLE),
        frc2::WaitCommand(2.0_s),

        frc2::InstantCommand([gripper] { 
            gripper->SetOpenGripper(); 
            std::cout << "SmartPick MID: Opening gripper for apple detection" << std::endl;
        }, {gripper}),
        frc2::WaitCommand(2.0_s),

        // FINAL MID POSITION
        MoveElevatorToPosition(elevator, 70.0f, 5.0f),  // MID: Final pickup height
        frc2::WaitCommand(3.0_s),
        
        frc2::PrintCommand("🎯 Robot positioned at MID level - starting apple detection"),
        
        AppleGripperCheckCommand(camera, AppleGripperCheckCommand::CheckMode::QUICK_CHECK, 5.0),
        
        frc2::ConditionalCommand(
            // SUCCESS PATH
            frc2::SequentialCommandGroup(
                frc2::PrintCommand("✅ MID apple detected! Executing grip sequence..."),
                
                frc2::InstantCommand([gripper] { 
                    gripper->SetClosedGripper(); 
                    std::cout << "SmartPick MID: Closing gripper to grab apple" << std::endl;
                }, {gripper}),
                frc2::WaitCommand(2.0_s),

                MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
                frc2::WaitCommand(2.0_s),

                MoveElevatorToPosition(elevator, 170.0f, 5.0f),
                frc2::WaitCommand(3.0_s),
                
                MoveArmToPosition(arm, HOME_ANGLE),
                frc2::WaitCommand(5.0_s),
                
                frc2::PrintCommand("✅ Smart Pick MID Complete - Apple Secured!")
            ),
            
            // FAILURE PATH
            frc2::SequentialCommandGroup(
                frc2::PrintCommand("❌ No MID apple detected - Aborting sequence"),
                
                MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
                frc2::WaitCommand(2.0_s),

                MoveElevatorToPosition(elevator, 170.0f, 5.0f),
                frc2::WaitCommand(3.0_s),
                
                MoveArmToPosition(arm, HOME_ANGLE),
                frc2::WaitCommand(7.0_s),
                
                frc2::PrintCommand("🏠 MID sequence aborted - returned to safe position")
            ),
            
            []() { 
                bool detected = frc::SmartDashboard::GetBoolean("Camera/Apple/Found", false);
                double distance = frc::SmartDashboard::GetNumber("Camera/Apple/Distance_MM", -1);
                bool validDetection = detected && (distance >= 120.0 && distance <= 1000.0); // MID: Medium range
                
                std::cout << "SmartPick MID: Apple detection result = " << (validDetection ? "TRUE" : "FALSE") << std::endl;
                std::cout << "SmartPick MID: Distance = " << distance << "mm" << std::endl;
                
                return validDetection;
            }
        ),
        
        frc2::PrintCommand("🏁 Smart Pick MID Sequence Finished")
    );
}