#pragma once
#include <frc/Ultrasonic.h>
#include <frc/Timer.h>
//#include "studica/DistanceSensor.h"
#include <frc/smartdashboard/SmartDashboard.h>

#define ULTRASONIC_TRIGGER_PORT 10
#define ULTRASONIC_ECHO_PORT 11

class UltrasonicSubsystem {
public:
    UltrasonicSubsystem();
    void Init();
    double GetDistance();
    void UpdateDashboard();
    void Periodic();
    bool IsWallDetected(double threshold = 5.0); // 5cm threshold

private:
    static constexpr int TRIGGER_PORT = ULTRASONIC_TRIGGER_PORT;
    static constexpr int ECHO_PORT = ULTRASONIC_ECHO_PORT;
};