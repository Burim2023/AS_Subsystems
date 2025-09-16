/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/UltrasonicSubsystem.h"
#include <iostream>

using namespace frc;

Ultrasonic* ultrasonicSensor = nullptr;

UltrasonicSubsystem::UltrasonicSubsystem() {}

void UltrasonicSubsystem::Init() {
    if (!ultrasonicSensor) {
        ultrasonicSensor = new Ultrasonic(TRIGGER_PORT, ECHO_PORT, Ultrasonic::kMilliMeters);
        Ultrasonic::SetAutomaticMode(true);
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
    
    SmartDashboard::PutNumber("Ultrasonic Distance (cm)", distance);
    SmartDashboard::PutBoolean("Wall Detected", IsWallDetected());
    
    if (IsWallDetected()) {
        std::cout << "*** WALL DETECTED ***" << std::endl;
        std::cout.flush();
    }
}

void UltrasonicSubsystem::Periodic() {
    UpdateDashboard();
}

void UltrasonicSubsystem::InitSendable(SendableBuilder& builder) {
    builder.SetSmartDashboardType("Ultrasonic Subsystem");
    builder.AddDoubleProperty("Distance (cm)", [this] { return GetDistance(); }, nullptr);
    builder.AddBooleanProperty("Wall Detected", [this] { return IsWallDetected(); }, nullptr);
}