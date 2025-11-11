#include "commands/DriveForDuration.h"
#include "subsystems/amcu/AMCU.h"
#include <iostream>

DriveForDuration::DriveForDuration(AMCU *amcu, double x, double y, double rotation, double duration)
    : m_amcu(amcu), m_x(x), m_y(y), m_rotation(rotation), m_duration(duration)
{
    // AMCU is now a proper subsystem - declare requirement to prevent conflicts
    AddRequirements({m_amcu});
}

void DriveForDuration::Initialize()
{
    // Reset and start the timer
    m_timer.Reset();
    m_timer.Start();

    // Start driving
    m_amcu->timeDrive(m_x, m_y, m_rotation, 100); // 100ms update rate

    std::cout << "DriveForDuration: Starting drive (x=" << m_x
              << ", y=" << m_y << ", rot=" << m_rotation
              << ") for " << m_duration << " seconds" << std::endl;
}

void DriveForDuration::Execute()
{
    // Continue driving - AMCU handles the movement internally
    // We could add additional logic here if needed
}

bool DriveForDuration::IsFinished()
{
    // Command finishes when the timer exceeds the specified duration
    return m_timer.Get() >= m_duration;
}

void DriveForDuration::End(bool interrupted)
{
    // CRITICAL: Stop the robot when the command ends
    m_amcu->stop();

    if (interrupted)
    {
        std::cout << "DriveForDuration: Command was interrupted after " << m_timer.Get() << " seconds" << std::endl;
    }
    else
    {
        std::cout << "DriveForDuration: Completed " << m_duration << " second drive" << std::endl;
    }
}