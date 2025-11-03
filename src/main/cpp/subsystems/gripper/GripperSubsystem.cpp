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
    GripperServo->SetAngle(GRIPPER_OPEN_ANGLE);
}

void GripperSubsystem::SetClosedGripper() {
    GripperServo->SetAngle(GRIPPER_CLOSED_ANGLE);
}

void GripperSubsystem::SetServoAngleZero() {
    GripperServo->SetAngle(0);
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