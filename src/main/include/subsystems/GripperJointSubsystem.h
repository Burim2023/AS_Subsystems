#pragma once

#include "studica/Servo.h"
#include <frc/smartdashboard/SmartDashboard.h>

#define JOINT_SERVO_PORT 21

#define JOINT_DOWN_ANGLE 0
#define JOINT_MID_ANGLE 50
#define JOINT_UP_ANGLE 75


class GripperJointSubsystem {
public:
    explicit GripperJointSubsystem();
    void Init();
    void SetGripperUpAngle();
    void SetGripperMidAngle();
    void SetGripperDownAngle();
    void UpdateDashboard();
    void Periodic();


private:
    double servoAngleGripperJoint = 5;
    
};
