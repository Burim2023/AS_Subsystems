#include "subsystems/ElevatorSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

ElevatorSubsystem::ElevatorSubsystem() 
    : m_isInitialized(false), m_lastTargetPosition(-1.0f), m_amcu(nullptr) {
    SetName("ElevatorSubsystem");
}

ElevatorSubsystem::~ElevatorSubsystem() {
    if (m_isInitialized) {
        elevator::destroy();
    }
}

void ElevatorSubsystem::Init(AMCU* amcu) {
    if (!m_isInitialized && amcu != nullptr) {
        m_amcu = amcu;
        elevator::init(amcu);
        m_isInitialized = true;
        frc::SmartDashboard::PutString("Elevator Status", "Initialized");
    }
}

void ElevatorSubsystem::MoveTo(float position) {
    if (m_isInitialized) {
        elevator::moveTo(position);
        m_lastTargetPosition = position;
        frc::SmartDashboard::PutNumber("Elevator Target", position);
    }
}

void ElevatorSubsystem::Calibrate() {
    if (m_isInitialized) {
        elevator::calibrate();
        m_lastTargetPosition = 0.0f; // Calibration moves to zero
        frc::SmartDashboard::PutString("Elevator Status", "Calibrating");
    }
}

float ElevatorSubsystem::GetCurrentPosition() {
    return elevator::currentPos.load();
}

bool ElevatorSubsystem::IsAtTarget(float tolerance) {
    if (m_lastTargetPosition < 0) return true; // No target set
    
    float currentPos = GetCurrentPosition();
    return std::abs(currentPos - m_lastTargetPosition) <= tolerance;
}

void ElevatorSubsystem::Stop() {
    if (m_isInitialized) {
        // Stop by setting target to current position
        MoveTo(GetCurrentPosition());
    }
}

void ElevatorSubsystem::Periodic() {
    if (m_isInitialized) {
        // Update SmartDashboard with current status
        float currentPos = GetCurrentPosition();
        frc::SmartDashboard::PutNumber("Elevator Position", currentPos);
        frc::SmartDashboard::PutNumber("Elevator Target", m_lastTargetPosition);
        frc::SmartDashboard::PutBoolean("Elevator At Target", IsAtTarget());
        frc::SmartDashboard::PutNumber("Elevator Error", std::abs(currentPos - m_lastTargetPosition));
    }
}