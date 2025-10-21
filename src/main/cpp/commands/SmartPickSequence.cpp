#include "commands/SmartPickSequence.h"
#include "commands/MoveArmToPosition.h"
#include "commands/MoveGripperJointToPosition.h"
#include "commands/MoveElevatorToPosition.h"
#include "commands/CalibrateElevator.h"
#include "commands/ElevatorPresets.h"
#include "Constants.h"

SmartPickSequence::SmartPickSequence(ArmSubsystem* arm, 
                                   GripperSubsystem* gripper, 
                                   GripperJointSubsystem* gripperJoint,
                                   CameraSubsystem* camera,
                                   ElevatorSubsystem* elevator) {
    
    SetName("SmartPickSequence");
    
    // THIS WAS THE ORIGINAL WORKING VERSION - using WasAppleDetected()
    auto* appleCheck = new AppleGripperCheckCommand(camera, 
                                                   AppleGripperCheckCommand::CheckMode::CONTINUOUS_MONITOR, 
                                                   5.0);

    AddCommands(
        // === PHASE 1: PREPARATION ===
        frc2::PrintCommand("🍎 Starting Smart Pick Sequence..."),
        
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

        // === PHASE 2: CALIBRATE ELEVATOR WHILE HOLDING GRIPPER ===
        frc2::ParallelDeadlineGroup(
            CalibrateElevator(elevator, 10.0),                               // DEADLINE
            MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE, true)  // HOLD at mid during calibration
        ),

        frc2::PrintCommand("✅ Calibration complete - positioning for detection"),

        // === PHASE 3: POSITION FOR DETECTION ===
        MoveElevatorToPosition(elevator, 60.0f, 1.0f),
        
        frc2::WaitCommand(2.0_s),

        MoveGripperJointToPosition(gripperJoint, JOINT_DOWN_ANGLE),
        
        frc2::WaitCommand(2.0_s),

        frc2::InstantCommand([gripper] { 
            gripper->SetOpenGripper(); 
            std::cout << "SmartPick: Opening gripper for apple detection" << std::endl;
        }, {gripper}),

        frc2::WaitCommand(2.0_s),
        
        frc2::PrintCommand("🎯 Robot positioned - starting apple detection"),
        frc2::PrintCommand("🔍 Checking for apple presence..."),
        *appleCheck,
        frc2::PrintCommand("🐛 DEBUG: Apple detection completed - checking results..."),
        
        frc2::ConditionalCommand(
            // IF APPLE DETECTED: Execute grip sequence
            frc2::SequentialCommandGroup(
                frc2::PrintCommand("✅ Apple detected! Executing grip sequence..."),
                
                MoveElevatorToPosition(elevator, 45.0f, 1.0f),
                
                frc2::WaitCommand(3.0_s),
                
                frc2::InstantCommand([gripper] { 
                    gripper->SetClosedGripper(); 
                    std::cout << "SmartPick: Closing gripper to grab apple" << std::endl;
                }, {gripper}),

                frc2::WaitCommand(2.0_s),

                MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
                
                frc2::WaitCommand(2.0_s),

                MoveElevatorToPosition(elevator, 170.0f, 1.0f),
                
                frc2::WaitCommand(2.0_s),

                MoveArmToPosition(arm, HOME_ANGLE),
                
                frc2::WaitCommand(1.0_s),
                MoveGripperJointToPosition(gripperJoint, JOINT_DOWN_ANGLE),
                
                frc2::PrintCommand("✅ Smart Pick Complete - Apple Secured!")
            ),
            
            // IF NO APPLE DETECTED: Abort sequence
            frc2::SequentialCommandGroup(
                frc2::PrintCommand("❌ No apple detected - Aborting sequence"),
                
                MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
                frc2::WaitCommand(1.5_s),

                MoveElevatorToPosition(elevator, 170.0f, 1.0f),
                frc2::WaitCommand(1.0_s),
                
                MoveArmToPosition(arm, HOME_ANGLE),
                frc2::WaitCommand(1.0_s),
                
                frc2::PrintCommand("🔄 Final gripper position reset..."),
                MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
                
                frc2::PrintCommand("🏠 Sequence aborted - returned to safe position")
            ),
            
            // THIS WAS THE ORIGINAL CONDITIONAL LOGIC THAT WORKED ONCE
            [appleCheck]() { 
                bool detected = frc::SmartDashboard::GetBoolean("Camera/Apple/Found", false);
                double distance = frc::SmartDashboard::GetNumber("Camera/Apple/Distance_MM", -1);
                bool validDetection = detected && (distance >= 200.0 && distance <= 2500.0);
                
                std::cout << "SmartPick: Apple detection result = " << (validDetection ? "TRUE" : "FALSE") << std::endl;
                std::cout << "SmartPick: SmartDashboard Apple/Found = " << detected << std::endl;
                std::cout << "SmartPick: Distance = " << distance << "mm" << std::endl;
                
                return validDetection;
            }
        ),
        
        frc2::PrintCommand("🏁 Smart Pick Sequence Finished")
    );
}