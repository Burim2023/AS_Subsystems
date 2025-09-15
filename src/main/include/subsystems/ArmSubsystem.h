#pragma once

#include "studica/Servo.h"
#include <frc/smartdashboard/SmartDashboard.h>

#define ARM_SERVO_PORT 18

#define HOME_ANGLE 0
#define DROP_APPLE_ANGLE 30
#define PICK_APPLE_ANGLE 156

class ArmSubsystem {
public:
    explicit ArmSubsystem();
    void Init();
    void SetHomePosition();
    void SetDropApplePosition();
    void SetPickApplePosition();
    void UpdateDashboard();
    void Periodic();
    void SetServoAngleZero();

private:
    double servoAngleArm = 30;
    
};
