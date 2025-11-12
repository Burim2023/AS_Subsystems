#include "commands/AppleGripperCheckCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

AppleGripperCheckCommand::AppleGripperCheckCommand(CameraSubsystem* cameraSubsystem, 
                                                 CheckMode mode, 
                                                 double timeoutSeconds)
    : m_camera(cameraSubsystem)
    , m_mode(mode)
    , m_timeoutSeconds(timeoutSeconds)
    , m_appleDetected(false)
    , m_appleDistance(-1.0) {
    
    AddRequirements({m_camera});
    SetName("AppleDetection");
}

void AppleGripperCheckCommand::Initialize() {
    std::cout << "=== APPLE DETECTION START ===" << std::endl;
    std::cout << "Mode: " << GetModeString() << std::endl;
    std::cout << "Camera: Orbbec Gemini E (RGB + Depth)" << std::endl;
    std::cout << "Depth Range: " << MIN_DEPTH_MM << "mm - " << MAX_DEPTH_MM << "mm" << std::endl;
    
    if (m_timeoutSeconds > 0) {
        std::cout << "Timeout: " << m_timeoutSeconds << " seconds" << std::endl;
    } else {
        std::cout << "Timeout: None (continuous)" << std::endl;
    }
    
    m_timer.Reset();
    m_timer.Start();
    
    // Reset detection results
    m_appleDetected = false;
    m_appleDistance = -1.0;
    
    frc::SmartDashboard::PutString("Apple Detection Status", "Searching...");
    frc::SmartDashboard::PutBoolean("Apple Found", false);
    frc::SmartDashboard::PutNumber("Apple Distance (mm)", -1);
}

void AppleGripperCheckCommand::Execute() {
    // Use the correct namespace "Camera/" instead of GetNamespace()
    if (!m_camera) {
        std::cout << "AppleGripperCheckCommand: camera subsystem is null, aborting detection" << std::endl;
        m_appleDetected = false;
        m_appleDistance = -1.0;
        return;
    }

    m_appleDetected = frc::SmartDashboard::GetBoolean("Camera/Apple/Found", false);
    
    if (m_appleDetected) {
        // Get apple center coordinates for depth lookup
        double appleCx = frc::SmartDashboard::GetNumber("Camera/Apple/Cx", -1);
        double appleCy = frc::SmartDashboard::GetNumber("Camera/Apple/Cy", -1);
        
        if (appleCx >= 0 && appleCy >= 0) {
            // Get depth value using the camera subsystem method
            try {
                m_appleDistance = m_camera->GetAppleDistance();
            } catch (const std::exception &e) {
                std::cout << "AppleGripperCheckCommand: Exception calling GetAppleDistance(): " << e.what() << std::endl;
                m_appleDistance = -1.0;
            }
            
            // Convert to millimeters if the method returns centimeters
            if (m_appleDistance > 0 && m_appleDistance < 100) {
                m_appleDistance *= 10.0; // Convert cm to mm
            }
            
            // Validate depth range for Orbbec Gemini E
            if (m_appleDistance > 0) {
                if (m_appleDistance < MIN_DEPTH_MM || m_appleDistance > MAX_DEPTH_MM) {
                    std::cout << "WARNING: Apple distance " << m_appleDistance 
                              << "mm outside valid range (" << MIN_DEPTH_MM 
                              << "-" << MAX_DEPTH_MM << "mm)" << std::endl;
                }
            }
        } else {
            m_appleDistance = -1.0;
        }
    } else {
        m_appleDistance = -1.0;
    }
    
    // Update SmartDashboard
    frc::SmartDashboard::PutBoolean("Apple Found", m_appleDetected);
    frc::SmartDashboard::PutNumber("Apple Distance (mm)", m_appleDistance);
    
    if (m_appleDetected) {
        if (m_appleDistance > 0) {
            frc::SmartDashboard::PutString("Apple Detection Status", 
                "Apple detected at " + std::to_string((int)m_appleDistance) + "mm");
        } else {
            frc::SmartDashboard::PutString("Apple Detection Status", "Apple detected - no distance data");
        }
    } else {
        frc::SmartDashboard::PutString("Apple Detection Status", "No apple detected");
    }
    
    // Log to console every 0.5 seconds
    static int logCounter = 0;
    if (++logCounter % 25 == 0) { // Every ~0.5 seconds at 50Hz
        LogAppleStatus();
    }
}

bool AppleGripperCheckCommand::IsFinished() {
    // Check timeout
    if (m_timeoutSeconds > 0 && m_timer.Get() > m_timeoutSeconds) {
        std::cout << "Apple Detection: TIMEOUT after " << m_timeoutSeconds << " seconds" << std::endl;
        return true;
    }
    
    switch (m_mode) {
        case CheckMode::QUICK_CHECK:
            // Finish after 0.1 seconds to get stable reading
            return m_timer.Get() > 0.1;
            
        case CheckMode::CONTINUOUS_MONITOR:
            // Run until timeout or interrupted
            return false;
    }
    
    return false;
}

void AppleGripperCheckCommand::End(bool interrupted) {
    m_timer.Stop();
    
    std::cout << "=== APPLE DETECTION END ===" << std::endl;
    
    if (interrupted) {
        std::cout << "Status: INTERRUPTED" << std::endl;
        frc::SmartDashboard::PutString("Apple Detection Status", "INTERRUPTED");
    } else {
        std::cout << "Status: COMPLETED" << std::endl;
        frc::SmartDashboard::PutString("Apple Detection Status", "COMPLETED");
    }
    
    // Final log
    LogAppleStatus();
    
    std::cout << "Duration: " << m_timer.Get() << " seconds" << std::endl;
    std::cout << "=============================" << std::endl;
}

void AppleGripperCheckCommand::LogAppleStatus() const {
    std::cout << "APPLE_DETECT: ";
    
    if (m_appleDetected) {
        if (m_appleDistance > 0) {
            std::cout << "FOUND at " << (int)m_appleDistance << "mm";
            
            // Add range status for Orbbec depth camera
            if (m_appleDistance < MIN_DEPTH_MM) {
                std::cout << " (TOO_CLOSE)";
            } else if (m_appleDistance > MAX_DEPTH_MM) {
                std::cout << " (TOO_FAR)";
            } else {
                std::cout << " (IN_RANGE)";
            }
        } else {
            std::cout << "FOUND but NO_DEPTH_DATA";
        }
    } else {
        std::cout << "NOT_FOUND";
    }
    
    std::cout << " | Timer: " << (int)(m_timer.Get() * 1000) << "ms" << std::endl;
}

std::string AppleGripperCheckCommand::GetModeString() const {
    switch (m_mode) {
        case CheckMode::QUICK_CHECK: 
            return "Quick Check";
        case CheckMode::CONTINUOUS_MONITOR: 
            return "Continuous Monitor";
        default: 
            return "Unknown";
    }
}