#pragma once

#include "studica/Servo.h"
#include <frc/smartdashboard/SmartDashboard.h>

#define EXTENDER_SERVO_PORT 20


#define EXTENDER_PICK_ANGLE 300
#define EXTENDER_DROP_ANGLE 50


class ExtenderSubsystem {
public:
    explicit ExtenderSubsystem();
    void Init();
    void SetDropPostion();
    void SetPickPostion(); // MAX Position of Extender // This Psotion is for picking from the Storage solution aswell as from the Ground
    void UpdateDashboard();
    void Periodic();


private:
    double servoAngleExtender = 0;
    
};
