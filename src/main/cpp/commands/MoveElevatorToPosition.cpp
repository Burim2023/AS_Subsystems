#include "commands/MoveElevatorToPosition.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

MoveElevatorToPosition::MoveElevatorToPosition(ElevatorSubsystem* elevator, float targetPosition, float tolerance)
    : m_elevator(elevator), m_targetPosition(targetPosition), m_tolerance(tolerance) {
    AddRequirements({m_elevator});
    SetName("MoveElevatorToPosition");
}

void MoveElevatorToPosition::Initialize() {
    std::cout << "MoveElevatorToPosition: Moving to " << m_targetPosition << "mm" << std::endl;
    frc::SmartDashboard::PutString("Active Command", "Moving Elevator to " + std::to_string(m_targetPosition) + "mm");
    
    if (m_elevator->IsInitialized()) {
        m_elevator->MoveTo(m_targetPosition);
    } else {
        std::cout << "MoveElevatorToPosition: ERROR - Elevator not initialized!" << std::endl;
    }
}

void MoveElevatorToPosition::Execute() {
    // The elevator subsystem handles the movement automatically via its thread
    // We just monitor progress here
    float currentPos = m_elevator->GetCurrentPosition();
    float error = std::abs(currentPos - m_targetPosition);
    frc::SmartDashboard::PutNumber("Elevator Command Error", error);
}

bool MoveElevatorToPosition::IsFinished() {
    if (!m_elevator->IsInitialized()) {
        return true; // End immediately if not initialized
    }
    return m_elevator->IsAtTarget(m_tolerance);
}

void MoveElevatorToPosition::End(bool interrupted) {
    if (interrupted) {
        std::cout << "MoveElevatorToPosition: Command interrupted at position " 
                  << m_elevator->GetCurrentPosition() << "mm" << std::endl;
        m_elevator->Stop();
        frc::SmartDashboard::PutString("Active Command", "Elevator Move Interrupted");
    } else {
        std::cout << "MoveElevatorToPosition: Reached target position " << m_targetPosition 
                  << "mm (actual: " << m_elevator->GetCurrentPosition() << "mm)" << std::endl;
        frc::SmartDashboard::PutString("Active Command", "Elevator Move Complete");
    }
}