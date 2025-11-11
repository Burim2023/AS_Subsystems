#include "commands/DriveDistanceCommand.h"
#include <iostream>

// Static member initialization
// DriveDistanceCommand* DriveDistanceCommand::s_currentCommand = nullptr;

DriveDistanceCommand::DriveDistanceCommand(AMCU *amcu, uint8_t xMeter, uint8_t yMeter, uint16_t omega_degree, double timeoutSeconds)
    : m_amcu(amcu), m_xMeter(xMeter), m_yMeter(yMeter), m_omega_degree(omega_degree), m_timeout(timeoutSeconds)
{
    SetName("DriveDistanceCommand");
    // AMCU is now a proper subsystem - declare requirement to prevent conflicts
    AddRequirements({m_amcu});
}

void DriveDistanceCommand::Initialize()
{
    m_timer.Reset();
    m_timer.Start();

    if (!m_amcu)
    {
        std::cout << "DriveDistanceCommand: AMCU null - aborting" << std::endl;
        return;
    }

    std::cout << "DriveDistanceCommand: Starting driveDistance(" << (int)m_xMeter << ", " << (int)m_yMeter
              << ", " << m_omega_degree << ") for " << m_timeout << "s" << std::endl;
    m_amcu->driveDistance(m_xMeter, m_yMeter, m_omega_degree);
}

void DriveDistanceCommand::Execute()
{
    // Optional: resend if needed, but driveDistance is typically one-shot
}

void DriveDistanceCommand::End(bool interrupted)
{
    if (m_amcu)
    {
        std::cout << "DriveDistanceCommand: Stopping (interrupted=" << interrupted << ")" << std::endl;
        m_amcu->stop();
    }
    m_timer.Stop();
}

bool DriveDistanceCommand::IsFinished()
{
    return m_timer.Get() >= m_timeout;
}

// // Static callback function
// void DriveDistanceCommand::DriveCompleteCallback() {
//     if (s_currentCommand) {
//         s_currentCommand->m_driveComplete = true;
//     }
// }