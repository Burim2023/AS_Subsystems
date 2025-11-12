#include "subsystems/gripper/GripperSubsystem.h"
#include <iostream>

studica::Servo* GripperServo = nullptr;
// frc::PWM* myServo;
double servoAngleGripper = GRIPPER_OPEN_ANGLE;

GripperSubsystem::GripperSubsystem() {}

void GripperSubsystem::Init() {
    if (GripperServo == nullptr) {
        GripperServo = new studica::Servo(GRIPPER_SERVO_PORT);
        // myServo = new frc::PWM(19);
        
        std::cout << "Servo initialized on port " << GRIPPER_SERVO_PORT << std::endl;
    }
    GripperServo->SetAngle(GRIPPER_OPEN_ANGLE);
    // servoAngleGripper = GRIPPER_OPEN_ANGLE;
}

double GripperSubsystem::GetServoAngle() {
    if (GripperServo) {
        return GripperServo->GetAngle();
    }
}

double GripperSubsystem::GetGripperPosition() {
    if (GripperServo) {
        return GripperServo->GetAngle();
    }
    return 0.0;
}

void GripperSubsystem::SetOpenGripper(){
    if (GripperServo) {
        GripperServo->SetAngle(GRIPPER_OPEN_ANGLE);
    } else {
        std::cout << "Warning: GripperServo is null in SetOpenGripper()" << std::endl;
    }
}

void GripperSubsystem::SetClosedGripper() {
    if (GripperServo) {
        GripperServo->SetAngle(GRIPPER_CLOSED_ANGLE);
    } else {
        std::cout << "Warning: GripperServo is null in SetClosedGripper()" << std::endl;
    }
}

void GripperSubsystem::SetServoAngleZero() {
    if (GripperServo) {
        GripperServo->SetAngle(0);
    } else {
        std::cout << "Warning: GripperServo is null in SetServoAngleZero()" << std::endl;
    }
}

void GripperSubsystem::UpdateDashboard() {
    frc::SmartDashboard::PutNumber("Arm Servo Position", servoAngleGripper);
    if (GripperServo) {
        frc::SmartDashboard::PutNumber("Arm Servo Raw Value", GripperServo->Get());
    } else {
        frc::SmartDashboard::PutNumber("Arm Servo Raw Value", -1);
    }
}

void GripperSubsystem::Periodic() {
    // myServo->SetSpeed(0.3);
    UpdateDashboard();
}