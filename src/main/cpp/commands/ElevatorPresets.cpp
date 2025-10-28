#include "commands/ElevatorPresets.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

ElevatorPresets::ElevatorPresets(ElevatorSubsystem* elevator, Position position)
    : m_elevator(elevator), m_targetPosition(position) {
    AddRequirements({m_elevator});
    SetName("ElevatorPresets");
}

void ElevatorPresets::Initialize() {
    float targetValue = GetPositionValue(m_targetPosition);
    std::string posName = GetPositionName(m_targetPosition);
    
    std::cout << "ElevatorPresets: Moving to " << posName << " position (" << targetValue << "mm)" << std::endl;
    frc::SmartDashboard::PutString("Active Command", "Elevator to " + posName);
    
    if (m_elevator->IsInitialized()) {
        m_elevator->MoveTo(targetValue);
    } else {
        std::cout << "ElevatorPresets: ERROR - Elevator not initialized!" << std::endl;
    }
}

void ElevatorPresets::Execute() {
    // Movement handled by subsystem
}

bool ElevatorPresets::IsFinished() {
    if (!m_elevator->IsInitialized()) {
        return true; // End immediately if not initialized
    }
    return m_elevator->IsAtTarget(5.0f); // 2mm tolerance for presets
}

void ElevatorPresets::End(bool interrupted) {
    if (interrupted) {
        std::cout << "ElevatorPresets: Movement interrupted" << std::endl;
        m_elevator->Stop();
        frc::SmartDashboard::PutString("Active Command", "Elevator Preset Interrupted");
    } else {
        std::cout << "ElevatorPresets: Reached " << GetPositionName(m_targetPosition) 
                  << " position (" << m_elevator->GetCurrentPosition() << "mm)" << std::endl;
        frc::SmartDashboard::PutString("Active Command", "Elevator Preset Complete");
    }
}

float ElevatorPresets::GetPositionValue(Position pos) {
    return static_cast<float>(pos);
}

std::string ElevatorPresets::GetPositionName(Position pos) {
    switch(pos) {
        case Position::GROUND: return "GROUND";
        case Position::LOW: return "LOW";
        case Position::MEDIUM: return "MEDIUM";
        case Position::HIGH: return "HIGH";
        case Position::MAX: return "MAX";
        default: return "UNKNOWN";
    }
}