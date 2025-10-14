#include "commands/CalibrateElevator.h"
#include <iostream>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>

CalibrateElevator::CalibrateElevator(ElevatorSubsystem* elevator, double timeoutSeconds)
    : m_elevator(elevator), m_timeoutSeconds(timeoutSeconds), m_calibrationStarted(false) {
    AddRequirements({m_elevator});
    SetName("CalibrateElevator");
}

void CalibrateElevator::Initialize() {
    std::cout << "CalibrateElevator: Starting calibration sequence" << std::endl;
    frc::SmartDashboard::PutString("Active Command", "Calibrating Elevator");
    
    m_startPosition = m_elevator->GetCurrentPosition();
    m_timer.Reset();
    m_timer.Start();
    m_calibrationStarted = false;
    
    if (!m_elevator->IsInitialized()) {
        std::cout << "CalibrateElevator: ERROR - Elevator not initialized!" << std::endl;
    }
}

void CalibrateElevator::Execute() {
    if (!m_calibrationStarted && m_elevator->IsInitialized()) {
        m_elevator->Calibrate();
        m_calibrationStarted = true;
        std::cout << "CalibrateElevator: Calibration command sent" << std::endl;
    }
    
    // Update SmartDashboard with calibration progress
    frc::SmartDashboard::PutNumber("Calibration Time", m_timer.Get());
    frc::SmartDashboard::PutNumber("Calibration Start Pos", m_startPosition);
}

bool CalibrateElevator::IsFinished() {
    if (!m_elevator->IsInitialized()) {
        return true; // End immediately if not initialized
    }
    
    // Check if calibration is complete by seeing if elevator is near zero position
    float currentPos = m_elevator->GetCurrentPosition();
    bool nearZero = std::abs(currentPos) < 3.0f; // Within 3mm of zero
    
    // Check for timeout
    bool timedOut = m_timer.Get() > m_timeoutSeconds;
    
    if (nearZero && m_calibrationStarted) {
        std::cout << "CalibrateElevator: Calibration complete, position: " << currentPos << "mm" << std::endl;
        return true;
    }
    
    if (timedOut) {
        std::cout << "CalibrateElevator: Calibration timed out after " << m_timeoutSeconds << " seconds" << std::endl;
        return true;
    }
    
    return false;
}

void CalibrateElevator::End(bool interrupted) {
    m_timer.Stop();
    if (interrupted) {
        std::cout << "CalibrateElevator: Calibration interrupted" << std::endl;
        m_elevator->Stop();
        frc::SmartDashboard::PutString("Active Command", "Calibration Interrupted");
    } else {
        std::cout << "CalibrateElevator: Calibration finished at position " 
                  << m_elevator->GetCurrentPosition() << "mm" << std::endl;
        frc::SmartDashboard::PutString("Active Command", "Calibration Complete");
    }
}