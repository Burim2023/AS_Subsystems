#include "subsystems/gripper/GripperJointSubsystem.h"
#include <iostream>
#include <algorithm>

GripperJointSubsystem::GripperJointSubsystem() {
    GripperJointServo = nullptr;
    servoAngleGripperJoint = JOINT_DOWN_ANGLE;
    targetAngle = JOINT_DOWN_ANGLE;
    servoSpeed = 2.0;
}

void GripperJointSubsystem::Init() {
    if (GripperJointServo == nullptr) {
        GripperJointServo = new studica::Servo(JOINT_SERVO_PORT);
        std::cout << "Servo initialized on port " << JOINT_SERVO_PORT << std::endl;
    }
    GripperJointServo->SetAngle(JOINT_DOWN_ANGLE);
    servoAngleGripperJoint = JOINT_DOWN_ANGLE;
}


void GripperJointSubsystem::SetGripperUpAngle(){
    targetAngle = JOINT_UP_ANGLE;
}

void GripperJointSubsystem::SetGripperMidAngle() {
    targetAngle = JOINT_MID_ANGLE;
}

void GripperJointSubsystem::SetGripperCamAngle() {
    targetAngle = JOINT_CAM_ANGLE;
}

void GripperJointSubsystem::SetGripperDownAngle() {
    targetAngle = JOINT_DOWN_ANGLE;
}

void GripperJointSubsystem::SetServoAngleZero(){
    targetAngle = 0;
}

void GripperJointSubsystem::SetServoSpeed(double speed) {
    servoSpeed = std::max(kMinSpeed, std::min(speed, kMaxSpeed)); // Clamp between limits
}

bool GripperJointSubsystem::IsMoving() const {
    return std::abs(servoAngleGripperJoint - targetAngle) > kMovementTolerance;
}

void GripperJointSubsystem::UpdateDashboard() {
    frc::SmartDashboard::PutNumber("Gripper Joint Current Angle", servoAngleGripperJoint);
    frc::SmartDashboard::PutNumber("Gripper Joint Target Angle", targetAngle);
    frc::SmartDashboard::PutNumber("Gripper Joint Speed", servoSpeed);
    if (GripperJointServo) {
        frc::SmartDashboard::PutNumber("Gripper Joint Raw Value", GripperJointServo->Get());
    } else {
        frc::SmartDashboard::PutNumber("Gripper Joint Raw Value", -1);
    }
    // Status indicator
    bool isMoving = IsMoving();
    frc::SmartDashboard::PutBoolean("Gripper Joint Moving", isMoving);
}

void GripperJointSubsystem::Periodic() {
    // Gradually move servo towards target angle
    if (GripperJointServo && IsMoving()) {
        double direction = (targetAngle > servoAngleGripperJoint) ? 1.0 : -1.0;
        double step = std::min(servoSpeed, std::abs(targetAngle - servoAngleGripperJoint));
        servoAngleGripperJoint += direction * step;
        GripperJointServo->SetAngle(servoAngleGripperJoint);
        // Debug output
        static int debugCounter = 0;
        if (debugCounter++ % 25 == 0) { // Print every 25 cycles (~0.5 seconds)
            std::cout << "Gripper Joint moving: " << servoAngleGripperJoint << " -> " << targetAngle << std::endl;
        }
    }
    UpdateDashboard();
}