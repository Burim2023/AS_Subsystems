#include "subsystems/gripper/GripperSubsystem.h"
#include <iostream>

GripperSubsystem::GripperSubsystem()
{
    SetName("GripperSubsystem");
}

void GripperSubsystem::Init()
{
    if (!m_servo)
    {
        m_servo = std::make_unique<studica::Servo>(GRIPPER_SERVO_PORT);
        std::cout << "GripperSubsystem: Servo initialized on port " << GRIPPER_SERVO_PORT << std::endl;
    }
    m_servo->SetAngle(GRIPPER_OPEN_ANGLE);
    m_angle = GRIPPER_OPEN_ANGLE;
}

double GripperSubsystem::GetServoAngle()
{
    if (m_servo)
    {
        return m_servo->GetAngle();
    }
    return -1.0; // Return error value if not initialized
}

double GripperSubsystem::GetGripperPosition()
{
    if (m_servo)
    {
        return m_servo->GetAngle();
    }
    return 0.0;
}

void GripperSubsystem::SetOpenGripper()
{
    if (m_servo)
    {
        m_servo->SetAngle(GRIPPER_OPEN_ANGLE);
        m_angle = GRIPPER_OPEN_ANGLE;
    }
}

void GripperSubsystem::SetClosedGripper()
{
    if (m_servo)
    {
        m_servo->SetAngle(GRIPPER_CLOSED_ANGLE);
        m_angle = GRIPPER_CLOSED_ANGLE;
    }
}

void GripperSubsystem::SetServoAngleZero()
{
    if (m_servo)
    {
        m_servo->SetAngle(0);
        m_angle = 0;
    }
}

void GripperSubsystem::UpdateDashboard()
{
    frc::SmartDashboard::PutNumber("Gripper Angle", m_angle);
    if (m_servo)
    {
        frc::SmartDashboard::PutNumber("Gripper Raw Value", m_servo->Get());
    }
    else
    {
        frc::SmartDashboard::PutNumber("Gripper Raw Value", -1);
    }
}

void GripperSubsystem::Periodic()
{
    UpdateDashboard();
}