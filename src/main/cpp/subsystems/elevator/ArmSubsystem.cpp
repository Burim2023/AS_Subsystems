#include "subsystems/elevator/ArmSubsystem.h"
#include <iostream>
#include <algorithm>

// ...existing code...


ArmSubsystem::ArmSubsystem() {
    ArmServo = nullptr;
    servoAngleArm = HOME_ANGLE;
    targetAngle = HOME_ANGLE;
    servoSpeed = 5.0;
}

void ArmSubsystem::Init() {
    if (ArmServo == nullptr) {
        ArmServo = new studica::Servo(ARM_SERVO_PORT);
        std::cout << "Servo initialized on port " << ARM_SERVO_PORT << std::endl;
    }
    ArmServo->SetAngle(HOME_ANGLE);
    servoAngleArm = HOME_ANGLE;
}

void ArmSubsystem::SetHomePosition() {
    targetAngle = HOME_ANGLE;
}

void ArmSubsystem::SetDropApplePosition() {
    targetAngle = DROP_APPLE_ANGLE;
}

void ArmSubsystem::SetPickApplePosition() {
    targetAngle = PICK_APPLE_ANGLE;
}

void ArmSubsystem::SetServoAngleZero() {
    targetAngle = 0;
}

void ArmSubsystem::SetServoSpeed(double speed) {
    servoSpeed = std::max(kMinSpeed, std::min(speed, kMaxSpeed)); // Clamp between limits
}

bool ArmSubsystem::IsMoving() const {
    return std::abs(servoAngleArm - targetAngle) > kMovementTolerance;
}

void ArmSubsystem::UpdateDashboard() {
    frc::SmartDashboard::PutNumber("Arm Current Angle", servoAngleArm);
    frc::SmartDashboard::PutNumber("Arm Target Angle", targetAngle);
    frc::SmartDashboard::PutNumber("Arm Speed", servoSpeed);
    if (ArmServo) {
        frc::SmartDashboard::PutNumber("Arm Servo Raw Value", ArmServo->GetAngle());
    } else {
        frc::SmartDashboard::PutNumber("Arm Servo Raw Value", -1);
    }
    // Status indicator
    bool isMoving = IsMoving();
    frc::SmartDashboard::PutBoolean("Arm Moving", isMoving);
    // Status string for easier monitoring
    std::string status;
    if (!ArmServo) {
        status = "ERROR: Not initialized";
    } else if (isMoving) {
        status = "Moving: " + std::to_string(servoAngleArm) + " -> " + std::to_string(targetAngle);
    } else {
        status = "At position: " + std::to_string(servoAngleArm) + " degrees";
    }
    frc::SmartDashboard::PutString("Arm Status", status);
}

void ArmSubsystem::Periodic() {
    // Gradually move servo towards target angle
    if (ArmServo && IsMoving()) {
        double direction = (targetAngle > servoAngleArm) ? 1.0 : -1.0;
        double step = std::min(servoSpeed, std::abs(targetAngle - servoAngleArm));
        servoAngleArm += direction * step;
        ArmServo->SetAngle(servoAngleArm);
        // Debug output
        static int debugCounter = 0;
        if (debugCounter++ % 25 == 0) { // Print every 25 cycles (~0.5 seconds)
            std::cout << "Arm moving: " << servoAngleArm << " -> " << targetAngle << std::endl;
        }
    }
    UpdateDashboard();
}