#include "subsystems/UltrasonicSubsystem.h"
#include <frc/Ultrasonic.h>
#include <iostream>

frc::Ultrasonic* ultrasonicSensor = nullptr;

UltrasonicSubsystem::UltrasonicSubsystem() {}

void UltrasonicSubsystem::Init() {
    if (!ultrasonicSensor) {
        ultrasonicSensor = new frc::Ultrasonic(TRIGGER_PORT, ECHO_PORT, frc::Ultrasonic::kMilliMeters);
        frc::Ultrasonic::SetAutomaticMode(true);
        std::cout << "Ultrasonic sensor initialized - Trigger: " << TRIGGER_PORT << ", Echo: " << ECHO_PORT << std::endl;
        std::cout.flush(); // Force output to be visible immediately
    }
}

double UltrasonicSubsystem::GetDistance() {
    if (ultrasonicSensor) {
        if (ultrasonicSensor->IsRangeValid()) {
            return ultrasonicSensor->GetRangeMM() / 10.0; // convert mm to cm
        } else {
            std::cout << "Ultrasonic: Range not valid yet" << std::endl;
            std::cout.flush();
            return 0.0; // Return 0 if not valid yet
        }
    }
    std::cout << "Ultrasonic: Sensor not initialized!" << std::endl;
    std::cout.flush();
    return -1.0;
}

bool UltrasonicSubsystem::IsWallDetected(double threshold) {
    double distance = GetDistance();
    return (distance > 0 && distance < threshold);
}

void UltrasonicSubsystem::UpdateDashboard() {
    double distance = GetDistance();
    std::cout << "=== Ultrasonic Distance: " << distance << " cm ===" << std::endl;
    std::cout.flush(); // Force immediate output
    
    frc::SmartDashboard::PutNumber("Ultrasonic Distance (cm)", distance);
    frc::SmartDashboard::PutBoolean("Wall Detected", IsWallDetected());
    
    if (IsWallDetected()) {
        std::cout << "*** WALL DETECTED ***" << std::endl;
        std::cout.flush();
    }
}

void UltrasonicSubsystem::Periodic() {
    UpdateDashboard();
}