#include "commands/MoveElevatorToPosition.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

MoveElevatorToPosition::MoveElevatorToPosition(ElevatorSubsystem* elevator, float targetPosition, float tolerance)
    : m_elevator(elevator), 
      m_targetPosition(targetPosition), 
      m_tolerance(tolerance),
      m_startTime(0),
      m_timeout(5.0)  // 5 second timeout
{
    AddRequirements({m_elevator});
    SetName("MoveElevatorToPosition");
}

void MoveElevatorToPosition::Initialize() {
    m_startTime = frc::Timer::GetFPGATimestamp();  // ✅ Remove .value()
    
    std::cout << "MoveElevatorToPosition: Moving to " << m_targetPosition << "mm (timeout: " 
              << m_timeout << "s)" << std::endl;
    frc::SmartDashboard::PutString("Active Command", "Moving Elevator to " + std::to_string(m_targetPosition) + "mm");
    
    if (m_elevator->IsInitialized()) {
        m_elevator->MoveTo(m_targetPosition);
    } else {
        std::cout << "MoveElevatorToPosition: ERROR - Elevator not initialized!" << std::endl;
    }
}

void MoveElevatorToPosition::Execute() {
    float currentPos = m_elevator->GetCurrentPosition();
    float error = std::abs(currentPos - m_targetPosition);
    
    frc::SmartDashboard::PutNumber("Elevator Command Error", error);
    
    static double lastLogTime = 0;
    double now = frc::Timer::GetFPGATimestamp();  // ✅ Remove .value()
    if (now - lastLogTime > 0.5) {
        std::cout << "MoveElevatorToPosition: current=" << currentPos << "mm, target=" 
                  << m_targetPosition << "mm, error=" << error << "mm" << std::endl;
        lastLogTime = now;
    }
}

bool MoveElevatorToPosition::IsFinished() {
    if (!m_elevator->IsInitialized()) {
        std::cout << "MoveElevatorToPosition: Finished (not initialized)" << std::endl;
        return true;
    }
    
    double elapsed = frc::Timer::GetFPGATimestamp() - m_startTime;  // ✅ Remove .value()
    if (elapsed >= m_timeout) {
        std::cout << "MoveElevatorToPosition: TIMEOUT after " << elapsed << "s (target=" 
                  << m_targetPosition << "mm, current=" << m_elevator->GetCurrentPosition() 
                  << "mm)" << std::endl;
        return true;
    }
    
    bool atTarget = m_elevator->IsAtTarget(m_tolerance);
    if (atTarget) {
        std::cout << "MoveElevatorToPosition: TARGET REACHED (current=" 
                  << m_elevator->GetCurrentPosition() << "mm)" << std::endl;
    }
    
    return atTarget;
}

void MoveElevatorToPosition::End(bool interrupted) {
    double elapsed = frc::Timer::GetFPGATimestamp() - m_startTime;  // ✅ Remove .value()
    
    if (interrupted) {
        std::cout << "MoveElevatorToPosition: INTERRUPTED at position " 
                  << m_elevator->GetCurrentPosition() << "mm after " << elapsed << "s" << std::endl;
        frc::SmartDashboard::PutString("Active Command", "Elevator Move Interrupted");
    } else {
        std::cout << "MoveElevatorToPosition: COMPLETED (target=" << m_targetPosition 
                  << "mm, actual=" << m_elevator->GetCurrentPosition() << "mm, time=" 
                  << elapsed << "s)" << std::endl;
        frc::SmartDashboard::PutString("Active Command", "Elevator Move Complete");
    }
}