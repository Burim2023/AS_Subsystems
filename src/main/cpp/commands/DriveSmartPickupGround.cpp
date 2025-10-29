#include "commands/DriveSmartPickupGround.h"
#include "commands/MoveArmToPosition.h"
#include "commands/MoveGripperJointToPosition.h"
#include "commands/MoveElevatorToPosition.h"
#include "commands/CalibrateElevator.h"
#include "commands/ElevatorPresets.h"
#include "AMCU.h"
#include "Constants.h"
#include <frc2/command/PrintCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

DriveSmartPickupGround::DriveSmartPickupGround(ArmSubsystem* arm, 
                                   GripperSubsystem* gripper, 
                                   GripperJointSubsystem* gripperJoint,
                                   CameraSubsystem* camera,
                                   ElevatorSubsystem* elevator, AMCU* amcu)
                                   :m_amcu(amcu),
                                    m_arm(arm),
                                    m_gripper(gripper),
                                    m_gripperJoint(gripperJoint),
                                    m_camera(camera),
                                    m_elevator(elevator) {
    
    SetName("DriveSmartPickupGround");
    
    if (!m_amcu) {
        std::cout << "ERROR: AMCU is null in DriveSmartPickupGround!" << std::endl;
        return;
    }

    AddCommands(
        frc2::InstantCommand([this] {
            std::cout << "DriveSmartPickupGround: Scheduled/Initialized" << std::endl;
        }),
        // === PHASE 1: PREPARATION ===
        frc2::PrintCommand("🍎 Starting DRIVE Smart Pick Sequence..."),
        
        frc2::PrintCommand("🔄 Initial gripper positioning..."),
        MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
        frc2::WaitCommand(2.0_s),

        MoveGripperJointToPosition(gripperJoint, JOINT_CAM_ANGLE),
        frc2::WaitCommand(2.0_s),
        
        // === PHASE 2: DRIVE TO FIND APPLE ===
        frc2::PrintCommand("🚗 Starting apple search drive..."),
        frc2::ParallelRaceGroup(
            // Drive forward slowly
            SpeedDriveCommand(m_amcu, 20, 15, 0, 0),
            //DriveDistanceCommand(m_amcu, 0.5, 0, 0, 2.0),
            // Stop when apple is detected
            frc2::WaitUntilCommand([this] {
                bool appleFound = frc::SmartDashboard::GetBoolean("Camera/Apple/Found", false);
                double distance = frc::SmartDashboard::GetNumber("Camera/Apple/Distance_MM", -1);
                bool detected = appleFound && distance < 500.0;
                if (detected) {
                    std::cout << "DEBUG: Apple detected at distance " << distance << "mm" << std::endl;
                }
                return detected;
            })
        ),
        
        // CRITICAL: Ensure robot stops after the race
        frc2::InstantCommand([this] {
            if (m_amcu) {
                std::cout << "DEBUG: Stopping robot after apple detection" << std::endl;
                m_amcu->stop();
            }
        }, {}),
        
        // Add settling time after stopping
        frc2::WaitCommand(1.0_s),

        // === PHASE 3: 180° TURN SEQUENCE ===
        frc2::PrintCommand("🔄 Apple detected! Executing 180° turn..."),
        frc2::SequentialCommandGroup(
            // Start the turn (turn right)
            SpeedDriveCommand(m_amcu, 6, 0, 0, 30),
            
            // Turn for calculated time (180° ÷ 30°/s = 6 seconds)
            frc2::WaitCommand(6.0_s),
            
            // Stop the turn
            frc2::InstantCommand([this] {
                if (m_amcu) {
                    std::cout << "DEBUG: Stopping 180° turn" << std::endl;
                    m_amcu->stop();
                }
            }, {}),
            
            // Longer pause to let robot settle after turn
            frc2::WaitCommand(1.5_s),

            SpeedDriveCommand(m_amcu, 2, -5, 0, 0),

            frc2::WaitCommand(2.0_s)

        ),
        frc2::PrintCommand("✅ 180° turn completed - now facing away from apple"),
        
        // === PHASE 4: RESET TO KNOWN STATE ===
        frc2::PrintCommand("🔄 Resetting to known gripper position..."),
        MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
        frc2::WaitCommand(2.5_s),

        // Move arm to pick position
        frc2::PrintCommand("🦾 Moving arm to pick position..."),
        MoveArmToPosition(arm, PICK_APPLE_ANGLE),
        frc2::WaitCommand(4.0_s),

        // === PHASE 5: ISOLATED ELEVATOR CALIBRATION (NO PARALLEL COMMANDS) ===
        frc2::PrintCommand("🔧 Starting ISOLATED elevator calibration..."),
        
        // CRITICAL FIX: Remove ParallelDeadlineGroup - run calibration alone
        CalibrateElevator(elevator, 10.0),  // Run calibration by itself
        
        frc2::PrintCommand("✅ Calibration complete - robot is safe"),
        
        // Add extra wait after calibration for system to stabilize
        frc2::WaitCommand(2.0_s),

        // === PHASE 6: POSITION FOR DETECTION ===
        frc2::PrintCommand("📐 Positioning elevator for detection..."),
        MoveElevatorToPosition(elevator, 60.0f, 5.0f),
        frc2::WaitCommand(2.0_s),

        frc2::PrintCommand("🔽 Moving gripper joint to down position..."),
        MoveGripperJointToPosition(gripperJoint, JOINT_DOWN_ANGLE),
        frc2::WaitCommand(2.0_s),

        frc2::InstantCommand([gripper] { 
            gripper->SetOpenGripper(); 
            std::cout << "SmartPick: Opening gripper for apple detection" << std::endl;
        }, {gripper}),
        frc2::WaitCommand(2.0_s),
        
        // === PHASE 7: APPLE DETECTION (FIX MEMORY LEAK) ===
        frc2::PrintCommand("🎯 Robot positioned - starting apple detection"),
        frc2::PrintCommand("🔍 Checking for apple presence..."),
        
        // FIX: Use direct construction instead of 'new' to avoid memory leak
        AppleGripperCheckCommand(camera, 
                                AppleGripperCheckCommand::CheckMode::CONTINUOUS_MONITOR, 
                                5.0),
        
        frc2::PrintCommand("🐛 DEBUG: Apple detection completed - checking results..."),
        
        // === PHASE 8: CONDITIONAL PICKUP SEQUENCE ===
        frc2::ConditionalCommand(
            // IF APPLE DETECTED: Execute grip sequence
            frc2::SequentialCommandGroup(
                frc2::PrintCommand("✅ Apple detected! Executing grip sequence..."),
                
                frc2::PrintCommand("🔽 Lowering elevator to grab position..."),
                MoveElevatorToPosition(elevator, 45.0f, 5.0f),
                frc2::WaitCommand(3.0_s),
                
                frc2::InstantCommand([gripper] { 
                    gripper->SetClosedGripper(); 
                    std::cout << "SmartPick: Closing gripper to grab apple" << std::endl;
                }, {gripper}),
                frc2::WaitCommand(2.0_s),

                frc2::PrintCommand("🔼 Retracting gripper joint..."),
                MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
                frc2::WaitCommand(2.0_s),

                frc2::PrintCommand("⬆️ Lifting elevator to safe position..."),
                MoveElevatorToPosition(elevator, 170.0f, 5.0f),
                frc2::WaitCommand(2.0_s),

                frc2::PrintCommand("🏠 Returning arm to home position..."),
                MoveArmToPosition(arm, HOME_ANGLE),
                frc2::WaitCommand(1.0_s),
                
                MoveGripperJointToPosition(gripperJoint, JOINT_DOWN_ANGLE),
                frc2::WaitCommand(1.0_s),
                
                frc2::PrintCommand("✅ Smart Pick Complete - Apple Secured!")
            ),
            
            // IF NO APPLE DETECTED: Abort sequence
            frc2::SequentialCommandGroup(
                frc2::PrintCommand("❌ No apple detected - Aborting sequence"),
                
                frc2::PrintCommand("🔄 Retracting gripper joint..."),
                MoveGripperJointToPosition(gripperJoint, JOINT_MID_ANGLE),
                frc2::WaitCommand(1.5_s),

                frc2::PrintCommand("⬆️ Raising elevator to safe position..."),
                MoveElevatorToPosition(elevator, 170.0f, 5.0f),
                frc2::WaitCommand(1.0_s),
                
                frc2::PrintCommand("🏠 Returning arm to home..."),
                MoveArmToPosition(arm, HOME_ANGLE),
                frc2::WaitCommand(1.0_s),
                
                frc2::PrintCommand("🔽 Final gripper position reset..."),
                MoveGripperJointToPosition(gripperJoint, JOINT_DOWN_ANGLE),
                frc2::WaitCommand(1.0_s),
                
                frc2::PrintCommand("🏠 Sequence aborted - returned to safe position")
            ),
            
            // Detection condition
            []() { 
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