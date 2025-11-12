#include "commands/DriveDistanceCommand.h"
#include <iostream>

DriveDistanceCommand::DriveDistanceCommand(AMCU* amcu, 
                                          double xMeter, 
                                          double yMeter, 
                                          double omega_degree, 
                                          double timeoutSeconds)
    : m_amcu(amcu), 
      m_xMeter(xMeter), 
      m_yMeter(yMeter), 
      m_omega_degree(omega_degree), 
      m_timeout(timeoutSeconds) 
{
    SetName("DriveDistanceCommand");
}

void DriveDistanceCommand::Initialize() {
    m_timer.Reset();
    m_timer.Start();

    if (!m_amcu) {
        std::cout << "DriveDistanceCommand: AMCU null - aborting" << std::endl;
        return;
    }

    std::cout << "DriveDistanceCommand: Starting driveDistance(" 
              << m_xMeter << "m, " 
              << m_yMeter << "m, " 
              << m_omega_degree << "°) for " 
              << m_timeout << "s" << std::endl;
    
    // ✅ Option B: Direct meters - no scaling
    // Cast double to uint8_t (will round down)
    uint8_t xConverted = static_cast<uint8_t>(m_xMeter);
    uint8_t yConverted = static_cast<uint8_t>(m_yMeter);
    uint16_t omegaConverted = static_cast<uint16_t>(m_omega_degree);
    
    std::cout << "DriveDistanceCommand: Sending to AMCU: x=" 
              << static_cast<int>(xConverted) << "m, y=" 
              << static_cast<int>(yConverted) << "m, omega=" 
              << omegaConverted << "°" << std::endl;
    
    m_amcu->driveDistance(xConverted, yConverted, omegaConverted);
}

void DriveDistanceCommand::Execute() {
    // AMCU handles the movement
}

void DriveDistanceCommand::End(bool interrupted) {
    if (m_amcu) {
        std::cout << "DriveDistanceCommand: Stopping (interrupted=" << interrupted << ")" << std::endl;
        m_amcu->stop();
    }
    m_timer.Stop();
}

bool DriveDistanceCommand::IsFinished() {
    return m_timer.Get() >= m_timeout;
}