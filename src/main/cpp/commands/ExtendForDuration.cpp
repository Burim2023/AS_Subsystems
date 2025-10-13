#include "commands/ExtendForDuration.h"
#include <iostream>

ExtendForDuration::ExtendForDuration(ExtenderSubsystem* subsystem, double duration, bool clockwise)
    : m_extender(subsystem), m_duration(duration), m_isClockwise(clockwise) {
    // Declare subsystem dependency - prevents conflicts with other commands
    AddRequirements({m_extender});
}

void ExtendForDuration::Initialize() {
    // Reset and start the timer
    m_timer.Reset();
    m_timer.Start();
    
    // Start the servo rotation
    if (m_isClockwise) {
        m_extender->RotateClockwise();
        std::cout << "ExtendForDuration: Starting clockwise rotation for " << m_duration << " seconds" << std::endl;
    } else {
        m_extender->RotateCounterClockwise();
        std::cout << "ExtendForDuration: Starting counter-clockwise rotation for " << m_duration << " seconds" << std::endl;
    }
}

void ExtendForDuration::Execute() {
    // Nothing to do here - the servo keeps rotating until the timer expires
}

bool ExtendForDuration::IsFinished() {
    // Command finishes when the timer exceeds the specified duration (2020 WPILib syntax)
    return m_timer.Get() >= m_duration;
}

void ExtendForDuration::End(bool interrupted) {
    // CRITICAL: Stop the servo when the command ends
    m_extender->StopMovement();
    
    if (interrupted) {
        std::cout << "ExtendForDuration: Command was interrupted after " << m_timer.Get() << " seconds" << std::endl;
    } else {
        std::cout << "ExtendForDuration: Completed " << m_duration << " second rotation" << std::endl;
    }
}