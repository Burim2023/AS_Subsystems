#include "subsystems/ExtenderSubsystem.h"
#include <iostream>

studica::Servo* ExtenderServo = nullptr;
double servoAngleExtender = EXTENDER_PICK_ANGLE;

ExtenderSubsystem::ExtenderSubsystem() {}

void ExtenderSubsystem::Init() {
    if (ExtenderServo == nullptr) {
        ExtenderServo = new studica::Servo(EXTENDER_SERVO_PORT);
        std::cout << "Servo initialized on port " << EXTENDER_SERVO_PORT << std::endl;
    }
    ExtenderServo->SetAngle(EXTENDER_PICK_ANGLE);
    servoAngleExtender = EXTENDER_PICK_ANGLE;
}


void ExtenderSubsystem::SetPickPostion(){
    ExtenderServo->SetAngle(EXTENDER_PICK_ANGLE);
}

void ExtenderSubsystem::SetDropPostion() {
    ExtenderServo->SetAngle(EXTENDER_DROP_ANGLE);
}

void ExtenderSubsystem::UpdateDashboard() {
    frc::SmartDashboard::PutNumber("Arm Servo Position", servoAngleExtender);
    if (ExtenderServo) {
        frc::SmartDashboard::PutNumber("Arm Servo Raw Value", ExtenderServo->GetAngle());
    } else {
        frc::SmartDashboard::PutNumber("Arm Servo Raw Value", -1);
    }
}

void ExtenderSubsystem::Periodic() {
    UpdateDashboard();
}