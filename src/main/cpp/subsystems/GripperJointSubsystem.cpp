#include "subsystems/GripperJointSubsystem.h"
#include <iostream>

studica::Servo* GripperJointServo = nullptr;
double servoAngleGripperJoint = JOINT_DOWN_ANGLE;

GripperJointSubsystem::GripperJointSubsystem() {}

void GripperJointSubsystem::Init() {
    if (GripperJointServo == nullptr) {
        GripperJointServo = new studica::Servo(JOINT_SERVO_PORT);
        std::cout << "Servo initialized on port " << JOINT_SERVO_PORT << std::endl;
    }
    GripperJointServo->SetAngle(JOINT_DOWN_ANGLE);
    servoAngleGripperJoint = JOINT_DOWN_ANGLE;
}


void GripperJointSubsystem::SetGripperUpAngle(){
    GripperJointServo->SetAngle(JOINT_UP_ANGLE);
}

void GripperJointSubsystem::SetGripperMidAngle() {
    GripperJointServo->SetAngle(JOINT_MID_ANGLE);
}

void GripperJointSubsystem::SetGripperDownAngle() {
    GripperJointServo->SetAngle(JOINT_DOWN_ANGLE);
}

void GripperJointSubsystem::UpdateDashboard() {
    frc::SmartDashboard::PutNumber("Arm Servo Position", servoAngleGripperJoint);
    if (GripperJointServo) {
        frc::SmartDashboard::PutNumber("Arm Servo Raw Value", GripperJointServo->Get());
    } else {
        frc::SmartDashboard::PutNumber("Arm Servo Raw Value", -1);
    }
}

void GripperJointSubsystem::Periodic() {
    UpdateDashboard();
}