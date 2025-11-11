#include "commands/SpeedDriveCommand.h"
#include <iostream>

SpeedDriveCommand::SpeedDriveCommand(AMCU *amcu, double timeoutSeconds, uint8_t forward, uint8_t strafe, uint8_t rot)
    : m_amcu(amcu), m_timeout(timeoutSeconds), m_fwd(forward), m_strafe(strafe), m_rot(rot)
{
    // Declare subsystem requirement
    AddRequirements({amcu});
}

void SpeedDriveCommand::Initialize()
{
    m_timer.Reset();
    m_timer.Start();
    m_keepalive.Reset();
    m_keepalive.Start();

    if (!m_amcu)
    {
        std::cout << "SpeedDriveCommand: AMCU null - aborting drive" << std::endl;
        return;
    }
    std::cout << "SpeedDriveCommand: start speedDrive(" << static_cast<int>(m_fwd) << "," << static_cast<int>(m_strafe) << "," << static_cast<int>(m_rot) << ")" << std::endl;
    m_amcu->speedDrive(m_fwd, m_strafe, m_rot);
}

void SpeedDriveCommand::Execute()
{
    if (!m_amcu)
        return;
    if (m_keepalive.Get() >= kKeepaliveInterval)
    {
        m_amcu->speedDrive(m_fwd, m_strafe, m_rot);
        m_keepalive.Reset();
    }
}

void SpeedDriveCommand::End(bool interrupted)
{
    if (m_amcu)
    {
        std::cout << "SpeedDriveCommand: stop (interrupted=" << interrupted << ")" << std::endl;
        m_amcu->stop();
    }
    m_timer.Stop();
    m_keepalive.Stop();
}

bool SpeedDriveCommand::IsFinished()
{
    if (m_timeout <= 0.0)
        return false;
    return m_timer.Get() >= m_timeout;
}