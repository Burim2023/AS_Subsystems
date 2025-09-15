#pragma once

#include "studica/Servo.h"
#include <frc/smartdashboard/SmartDashboard.h>

#define GRIPPER_SERVO_PORT 19

#define GRIPPER_CLOSED_ANGLE 10
#define GRIPPER_OPEN_ANGLE 0


class GripperSubsystem {
public:
    explicit GripperSubsystem();
    void Init();
    void SetOpenGripper();
    void SetClosedGripper();
    void UpdateDashboard();
    void Periodic();


private:
    double servoAngleGripper = 0;
    
};
