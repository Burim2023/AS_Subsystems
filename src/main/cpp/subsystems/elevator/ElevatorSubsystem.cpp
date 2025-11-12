#include "subsystems/elevator/ElevatorSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include "Constants.h"

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
        //std::cout << "ElevatorSubsystem: Initialized successfully" << std::endl;
    }
}

void ElevatorSubsystem::MoveTo(float position) {
    if (m_isInitialized) {
        // Clamp position to safe range
        position = std::max(0.0f, std::min(position, ELEVATOR_HEIGHT));
        
        elevator::moveTo(position);
        m_lastTargetPosition = position;
        frc::SmartDashboard::PutNumber("Elevator Target", position);
        
        //std::cout << "ElevatorSubsystem: Moving to " << position << "mm" << std::endl;
    }
}

void ElevatorSubsystem::Calibrate() {
    if (m_isInitialized) {
        //std::cout << "ElevatorSubsystem: Starting calibration..." << std::endl;
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
    float error = std::abs(currentPos - m_lastTargetPosition);
    
    return error <= tolerance;
}

void ElevatorSubsystem::Stop() {
    if (m_isInitialized && m_amcu) {
        //std::cout << "ElevatorSubsystem: STOP requested" << std::endl;
        
        // CRITICAL FIX: Use AMCU to directly stop the motor instead of elevator::moveTo()
        try {
            m_amcu->setSpeed(Constants::kMotorElevator, 0);  // Force stop the motor
            m_amcu->setRPM(Constants::kMotorElevator, 0);    // Also stop RPM command
            
            // Update target to current position to prevent restart
            m_lastTargetPosition = GetCurrentPosition();
            
            //std::cout << "ElevatorSubsystem: Motor stopped via AMCU" << std::endl;
        } catch (...) {
            //std::cout << "ERROR: Failed to stop elevator motor" << std::endl;
        }
        
        frc::SmartDashboard::PutString("Elevator Status", "Stopped");
    }
}

void ElevatorSubsystem::Periodic() {
    if (m_isInitialized) {
        float currentPos = GetCurrentPosition();
        float error = (m_lastTargetPosition >= 0) ? std::abs(currentPos - m_lastTargetPosition) : 0.0f;
        
        frc::SmartDashboard::PutNumber("Elevator Position", currentPos);
        frc::SmartDashboard::PutNumber("Elevator Target", m_lastTargetPosition);
        frc::SmartDashboard::PutBoolean("Elevator At Target", IsAtTarget(5.0f));
        frc::SmartDashboard::PutNumber("Elevator Error", error);
        
        static float lastPosition = currentPos;
        static int oscillationCount = 0;
        static int stableCount = 0;
        static double lastWarningTime = 0;
        
        if (m_lastTargetPosition >= 0 && error < 10.0f) {
            if (std::abs(currentPos - lastPosition) > 2.0f) {
                oscillationCount++;
                stableCount = 0;
            } else {
                stableCount++;
                if (stableCount > 10) {
                    oscillationCount = 0;
                }
            }
            
            // Only WARN, don't auto-stop
            if (oscillationCount > 15) {
                double now = frc::Timer::GetFPGATimestamp();  // ✅ Remove .value()
                if (now - lastWarningTime > 1.0) {
                    std::cout << "WARNING: Elevator oscillation detected (target=" 
                              << m_lastTargetPosition << "mm, current=" << currentPos 
                              << "mm, error=" << error << "mm)" << std::endl;
                    lastWarningTime = now;
                }
            }
        }
        
        lastPosition = currentPos;
    }
}