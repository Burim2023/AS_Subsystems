#include "commands/ElevatorTestSequence.h"
#include <iostream>

ElevatorTestSequence::ElevatorTestSequence(ElevatorSubsystem* elevator) {
    // Set requirements for the elevator subsystem
    AddRequirements({elevator});
    
    // Build the command sequence
    AddCommands(
        // === PHASE 1: CALIBRATION ===
        // Start message
        frc2::PrintCommand("🎯 Starting Elevator Test Sequence..."),
        
        // Calibrate elevator to find zero position
        CalibrateElevator(elevator, 15.0), // 10 second timeout
        
        // Wait 2 seconds after calibration
        frc2::WaitCommand(5.0_s),
        frc2::PrintCommand("✅ Calibration complete - Starting position tests"),
        
        // === PHASE 2: LOW POSITION ===
        // Move to LOW position (25mm)
        ElevatorPresets(elevator, ElevatorPresets::Position::LOW),
        
        // Wait 2 seconds at LOW position
        frc2::WaitCommand(5.0_s),
        frc2::PrintCommand("📍 Elevator at LOW position (25mm)"),
        
        // === PHASE 3: MEDIUM POSITION ===
        // Move to MEDIUM position (50mm)
        ElevatorPresets(elevator, ElevatorPresets::Position::MEDIUM),
        
        // Wait 2 seconds at MEDIUM position
        frc2::WaitCommand(5.0_s),
        frc2::PrintCommand("📍 Elevator at MEDIUM position (50mm)"),
        
        // === PHASE 4: HIGH POSITION ===
        // Move to HIGH position (75mm)
        ElevatorPresets(elevator, ElevatorPresets::Position::HIGH),
        
        // Wait 2 seconds at HIGH position
        frc2::WaitCommand(5.0_s),
        frc2::PrintCommand("📍 Elevator at HIGH position (75mm)")
        
        // === PHASE 5: RETURN TO GROUND ===
        // Return to ground position
        //ElevatorPresets(elevator, ElevatorPresets::Position::GROUND),
        
        // Final message
        //frc2::PrintCommand("🏁 Elevator Test Sequence Complete! All positions tested.")
    );
}