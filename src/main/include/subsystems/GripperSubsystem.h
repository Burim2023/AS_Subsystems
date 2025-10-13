#pragma once

#include "studica/Servo.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

#define GRIPPER_SERVO_PORT 20

#define GRIPPER_CLOSED_ANGLE 0
#define GRIPPER_OPEN_ANGLE 25


class GripperSubsystem : public frc2::SubsystemBase {
public:
    explicit GripperSubsystem();
    void Init();
    void SetOpenGripper();
    void SetClosedGripper();
    void SetServoAngleZero();
    void UpdateDashboard();
    void Periodic() override;


private:
    double servoAngleGripper = 0;
    
};