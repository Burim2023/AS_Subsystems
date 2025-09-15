#include "subsystems/ArmSubsystem.h"
#include <iostream>

studica::Servo* ArmServo = nullptr;
double servoAngleArm = HOME_ANGLE;

ArmSubsystem::ArmSubsystem() {}

void ArmSubsystem::Init() {
    if (ArmServo == nullptr) {
        ArmServo = new studica::Servo(ARM_SERVO_PORT);
        std::cout << "Servo initialized on port " << ARM_SERVO_PORT << std::endl;
    }
    ArmServo->SetAngle(HOME_ANGLE);
    servoAngleArm = HOME_ANGLE;
}

void ArmSubsystem::SetHomePosition() {
    
    ArmServo->SetAngle(HOME_ANGLE);
}

void ArmSubsystem::SetDropApplePosition() {
    ArmServo->SetAngle(DROP_APPLE_ANGLE);
}

void ArmSubsystem::SetPickApplePosition() {
    ArmServo->SetAngle(PICK_APPLE_ANGLE); 
}

void ArmSubsystem::UpdateDashboard() {
    frc::SmartDashboard::PutNumber("Arm Servo Position", servoAngleArm);
    if (ArmServo) {
        frc::SmartDashboard::PutNumber("Arm Servo Raw Value", ArmServo->Get());
    } else {
        frc::SmartDashboard::PutNumber("Arm Servo Raw Value", -1);
    }
}

void ArmSubsystem::Periodic() {
    UpdateDashboard();
}

void ArmSubsystem::SetServoAngleZero() {
    ArmServo->SetAngle(0);
}