#pragma once

#include "studica/Servo.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <memory>

#define GRIPPER_SERVO_PORT 21

#define GRIPPER_CLOSED_ANGLE 0
#define GRIPPER_OPEN_ANGLE 45

class GripperSubsystem : public frc2::SubsystemBase
{
public:
    explicit GripperSubsystem();
    void Init();
    void SetOpenGripper();
    void SetClosedGripper();
    void SetServoAngleZero();
    void UpdateDashboard();
    void Periodic() override;

    double GetServoAngle();
    double GetGripperPosition();

private:
    std::unique_ptr<studica::Servo> m_servo; // Member variable (not global)
    double m_angle = GRIPPER_OPEN_ANGLE;     // Member variable (not global)
};