#include "commands/ExtendForDuration.h"
#include <iostream>

// Constructor for time-based positioning - DON'T calculate time here
ExtendForDuration::ExtendForDuration(ExtenderSubsystem* subsystem, Direction direction, double timeRatio, double safetyTimeoutMultiplier)
    : m_extender(subsystem), m_direction(direction), m_targetTime(0.0), m_safetyTimeout(0.0), m_useTimeBased(true), m_timeRatio(timeRatio), m_timeoutMultiplier(safetyTimeoutMultiplier) {
    
    SetName("ExtendForDuration");
    AddRequirements(m_extender);
    
    // Store the ratio and multiplier, but DON'T calculate target time yet
    std::cout << "ExtendForDuration: Time-based constructor" << std::endl;
    std::cout << "  TimeRatio: " << m_timeRatio << " (will calculate target time in Initialize)" << std::endl;
    std::cout << "  TimeoutMultiplier: " << m_timeoutMultiplier << std::endl;
}

// Fix the simple constructor - match header order
ExtendForDuration::ExtendForDuration(ExtenderSubsystem* subsystem, Direction direction)
    : m_extender(subsystem), m_direction(direction), m_targetTime(0.0), m_safetyTimeout(10.0), m_useTimeBased(false), m_timeRatio(1.0), m_timeoutMultiplier(2.0) {
    
    SetName("ExtendForDuration");
    AddRequirements(m_extender);
    
    std::cout << "ExtendForDuration: Simple constructor (limit-switch mode)" << std::endl;
    std::cout << "  Direction: " << (m_direction == Direction::EXTEND ? "EXTEND" : "RETRACT") << std::endl;
    std::cout << "  Mode: Limit switch based (no time calculation)" << std::endl;
}

void ExtendForDuration::Initialize() {
    // Reset and start timer
    m_timer.Reset();
    m_timer.Start();
    
    // NOW calculate target time (after calibration is complete)
    double maxTime = m_extender->GetMaxTimeFrontToBack();
    
    if (m_useTimeBased && maxTime > 0.0 && m_extender->IsCalibrated()) {
        // CALIBRATED: Calculate time-based positioning NOW
        m_targetTime = maxTime * m_timeRatio;  // Calculate target time HERE!
        m_safetyTimeout = m_targetTime * m_timeoutMultiplier;
        
        std::cout << "ExtendForDuration: Calculating time-based positioning" << std::endl;
        std::cout << "  MaxTime: " << maxTime << " seconds" << std::endl;
        std::cout << "  TimeRatio: " << m_timeRatio << std::endl;
        std::cout << "  Calculated TargetTime: " << m_targetTime << " seconds" << std::endl;
        std::cout << "  SafetyTimeout: " << m_safetyTimeout << " seconds" << std::endl;
    } else {
        // NOT CALIBRATED: Fall back to limit switch mode
        m_useTimeBased = false;
        m_targetTime = 0.0;
        m_safetyTimeout = 20.0;  // Default timeout
        
        std::cout << "ExtendForDuration: NOT CALIBRATED - falling back to limit switch mode" << std::endl;
        std::cout << "  MaxTime: " << maxTime << " (invalid)" << std::endl;
    }
    
    // Debug output
    std::cout << "=== ExtendForDuration::Initialize() ===" << std::endl;
    std::cout << "Direction: " << (m_direction == Direction::EXTEND ? "EXTEND" : "RETRACT") << std::endl;
    std::cout << "Use Time Based: " << (m_useTimeBased ? "YES" : "NO") << std::endl;
    std::cout << "Max Time: " << maxTime << " seconds" << std::endl;
    std::cout << "Target Time: " << m_targetTime << " seconds" << std::endl;
    std::cout << "Safety Timeout: " << m_safetyTimeout << " seconds" << std::endl;
    
    // Start movement based on direction
    if (m_direction == Direction::EXTEND) {
        m_extender->MoveToFront();
        std::cout << "Setting EXTEND state" << std::endl;
        if (m_useTimeBased) {
            std::cout << "ExtendForDuration: Starting EXTEND for " << m_targetTime << " seconds (time-based)" << std::endl;
        } else {
            std::cout << "ExtendForDuration: Starting EXTEND to front limit" << std::endl;
        }
    } else {
        m_extender->MoveToBack();
        std::cout << "Setting RETRACT state" << std::endl;
        if (m_useTimeBased) {
            std::cout << "ExtendForDuration: Starting RETRACT for " << m_targetTime << " seconds (time-based)" << std::endl;
        } else {
            std::cout << "ExtendForDuration: Starting RETRACT to back limit" << std::endl;
        }
    }
}

void ExtendForDuration::Execute() {
    // Let the subsystem state machine handle the movement
    m_extender->ExtenderSubsystemCurrentState();
}

bool ExtendForDuration::IsFinished() {
    // SAFETY: Always check limit switches first (for normally closed switches)
    bool frontPressed = m_extender->IsFrontLimitPressed();
    bool backPressed = m_extender->IsBackLimitPressed();
    
    // Stop immediately if appropriate limit switch is hit
    if (m_direction == Direction::EXTEND && frontPressed) {
        std::cout << "ExtendForDuration: Front limit reached - STOPPING" << std::endl;
        return true;
    }
    
    if (m_direction == Direction::RETRACT && backPressed) {
        std::cout << "ExtendForDuration: Back limit reached - STOPPING" << std::endl;
        return true;
    }
    
    double currentTime = m_timer.Get();
    
    // Safety timeout check
    if (currentTime >= m_safetyTimeout) {
        std::cout << "ExtendForDuration: SAFETY TIMEOUT after " << m_safetyTimeout << " seconds" << std::endl;
        return true;
    }
    
    if (m_useTimeBased) {
        // Time-based mode: finish when target time reached
        bool targetTimeReached = currentTime >= m_targetTime;
        
        if (targetTimeReached) {
            std::cout << "ExtendForDuration: Target time " << m_targetTime << "s reached" << std::endl;
        }
        
        return targetTimeReached;
        
    } else {
        // Limit switch mode: finish when limit switch hit (already checked above)
        return false;
    }
}

void ExtendForDuration::End(bool interrupted) {
    // Stop movement when command ends
    m_extender->Stop();
    
    double actualTime = m_timer.Get();
    
    if (interrupted) {
        std::cout << "ExtendForDuration: Command was interrupted after " << actualTime << " seconds" << std::endl;
    } else {
        if (m_useTimeBased) {
            std::cout << "ExtendForDuration: " << GetDirectionName() << " completed - Target: " 
                      << m_targetTime << "s, Actual: " << actualTime << "s" << std::endl;
        } else {
            std::cout << "ExtendForDuration: " << GetDirectionName() << " completed in " << actualTime << " seconds" << std::endl;
        }
    }
}

// Keep the GetDirectionName implementation at the end:
std::string ExtendForDuration::GetDirectionName() const {
    return (m_direction == Direction::EXTEND) ? "EXTEND" : "RETRACT";
}