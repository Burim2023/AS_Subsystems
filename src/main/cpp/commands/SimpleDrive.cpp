#include "commands/SimpleDrive.h"
#include "AMCU.h"
#include <iostream>

SimpleDrive::SimpleDrive(AMCU* amcu, double forward, double strafe, double rotate)
    : m_amcu(amcu), m_forward(forward), m_strafe(strafe), m_rotate(rotate) {
    // This command doesn't require any subsystems since AMCU isn't a subsystem
    // AddRequirements() would be used here if we had subsystem requirements
}

void SimpleDrive::Initialize() {
    std::cout << "🚗 SimpleDrive: Starting drive with forward=" << m_forward 
              << ", strafe=" << m_strafe << ", rotate=" << m_rotate << std::endl;
}

void SimpleDrive::Execute() {
    // Call AMCU drive methods directly
    if (m_amcu != nullptr) {
        // Convert normalized values (-1.0 to 1.0) to AMCU's expected units
        // AMCU expects speeds in cm/s for x,y and degrees/s for rotation
        // Since AMCU uses uint8_t (unsigned), we'll use absolute values
        // and handle direction through the robot's coordinate system
        
        // Scale the normalized values to reasonable speeds (all positive)
        uint8_t xSpeed_cms = (uint8_t)(abs(m_forward) * 50);  // Max 50 cm/s forward/back
        uint8_t ySpeed_cms = (uint8_t)(abs(m_strafe) * 50);   // Max 50 cm/s strafe
        uint8_t wSpeed_degs = (uint8_t)(abs(m_rotate) * 90);  // Max 90 degrees/s rotation
        
        // For now, use positive values only - direction handling may need to be
        // implemented at the AMCU firmware level or through motor configuration
        // You may need to adjust this based on your robot's coordinate system
        
        // Use AMCU's speedDrive method for continuous driving
        m_amcu->speedDrive(xSpeed_cms, ySpeed_cms, wSpeed_degs);
        
        // Debug output to help with testing
        std::cout << "SimpleDrive: x=" << (int)xSpeed_cms 
                  << " y=" << (int)ySpeed_cms 
                  << " w=" << (int)wSpeed_degs << std::endl;
    }
}

void SimpleDrive::End(bool interrupted) {
    std::cout << "🛑 SimpleDrive: Stopping drive" << std::endl;
    
    // Stop the drivetrain using AMCU's stop method
    if (m_amcu != nullptr) {
        m_amcu->stop(); // Use AMCU's built-in stop method
    }
}

bool SimpleDrive::IsFinished() {
    // This command never finishes on its own - it must be interrupted
    // This makes it suitable for default commands or manual control
    return false;
}