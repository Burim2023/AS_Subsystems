#include "commands/SmartPickSequenceHigh.h"
#include "commands/MoveArmToPosition.h"
#include "commands/MoveGripperJointToPosition.h"
#include "commands/MoveElevatorToPosition.h"
#include "commands/CalibrateElevator.h"
#include "commands/ElevatorPresets.h"
#include "commands/AppleGripperCheckCommand.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

SmartPickSequenceHigh::SmartPickSequenceHigh(ArmSubsystem* arm, 
                                            GripperSubsystem* gripper, 
                                            GripperJointSubsystem* gripperJoint,
                                            CameraSubsystem* camera,
                                            ElevatorSubsystem* elevator) {
    
    SetName("SmartPickSequenceHigh");

    AddCommands(
        // === PHASE 1: PREPARATION ===
        frc2::PrintCommand("🍎 Starting Smart Pick Sequence (HIGH LEVEL)..."),
        
        // ENSURE GRIPPER STARTS IN KNOWN POSITION
        frc2::PrintCommand("🔄 Resetting gripper to mid position..."),
        MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
        
        // Wait to ensure gripper reaches mid position before continuing
        frc2::WaitCommand(2.5_s),

        // Move arm to pick position
        MoveArmToPosition(arm, PICK_APPLE_ANGLE),
        
        // Wait for arm to reach position
        frc2::WaitCommand(4.0_s),

        frc2::PrintCommand("🔧 Starting elevator calibration..."),

        // === PHASE 2: CALIBRATE ELEVATOR ===
        CalibrateElevator(elevator, 10.0),

        frc2::PrintCommand("✅ Calibration complete - positioning for HIGH level detection"),

        // === PHASE 3: POSITION FOR HIGH DETECTION ===
        frc2::PrintCommand("Moving elevator to HIGH apple pickup position..."),
        MoveElevatorToPosition(elevator, 140.0f, 5.0f),  // HIGH: Much higher initial position
        frc2::WaitCommand(3.0_s),

        MoveGripperJointToPosition(gripperJoint, JOINT_DOWN_ANGLE),
        frc2::WaitCommand(2.0_s),

        frc2::InstantCommand([gripper] { 
            gripper->SetOpenGripper(); 
            std::cout << "SmartPick HIGH: Opening gripper for apple detection" << std::endl;
        }, {gripper}),
        frc2::WaitCommand(2.0_s),

        // FINAL HIGH POSITION - highest pickup level
        MoveElevatorToPosition(elevator, 125.0f, 5.0f),  // HIGH: Final pickup height
        frc2::WaitCommand(3.0_s),
        
        frc2::PrintCommand("🎯 Robot positioned at HIGH level - starting apple detection"),
        
        // Apple detection for high level
        AppleGripperCheckCommand(camera, AppleGripperCheckCommand::CheckMode::QUICK_CHECK, 5.0),
        
        frc2::PrintCommand("🐛 DEBUG: HIGH apple detection completed - checking results..."),
        
        // Conditional logic for high level
        frc2::ConditionalCommand(
            // IF HIGH APPLE DETECTED: Execute grip sequence
            frc2::SequentialCommandGroup(
                frc2::PrintCommand("✅ HIGH apple detected! Executing grip sequence..."),
                
                frc2::InstantCommand([gripper] { 
                    gripper->SetClosedGripper(); 
                    std::cout << "SmartPick HIGH: Closing gripper to grab apple" << std::endl;
                }, {gripper}),
                frc2::WaitCommand(2.0_s),

                MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
                frc2::WaitCommand(2.0_s),

                MoveElevatorToPosition(elevator, 170.0f, 5.0f),
                frc2::WaitCommand(3.0_s),
                
                MoveArmToPosition(arm, HOME_ANGLE),
                frc2::WaitCommand(5.0_s),
                
                frc2::PrintCommand("✅ Smart Pick HIGH Complete - Apple Secured!")
            ),
            
            // IF NO HIGH APPLE DETECTED: Abort sequence
            frc2::SequentialCommandGroup(
                frc2::PrintCommand("❌ No HIGH apple detected - Aborting sequence"),
                
                MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
                frc2::WaitCommand(2.0_s),

                MoveElevatorToPosition(elevator, 170.0f, 5.0f),
                frc2::WaitCommand(3.0_s),
                
                MoveArmToPosition(arm, HOME_ANGLE),
                frc2::WaitCommand(7.0_s),
                
                frc2::PrintCommand("🏠 HIGH sequence aborted - returned to safe position")
            ),
            
            // Detection condition for high level - wider range for distant apples
            []() { 
                bool detected = frc::SmartDashboard::GetBoolean("Camera/Apple/Found", false);
                double distance = frc::SmartDashboard::GetNumber("Camera/Apple/Distance_MM", -1);
                bool validDetection = detected && (distance >= 150.0 && distance <= 1500.0); // HIGH: Longer range
                
                std::cout << "SmartPick HIGH: Apple detection result = " << (validDetection ? "TRUE" : "FALSE") << std::endl;
                std::cout << "SmartPick HIGH: SmartDashboard Apple/Found = " << detected << std::endl;
                std::cout << "SmartPick HIGH: Distance = " << distance << "mm" << std::endl;
                
                return validDetection;
            }
        ),
        
        frc2::PrintCommand("🏁 Smart Pick HIGH Sequence Finished")
    );
}