/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IRRangeSubsystem.h"
#include <iostream>
#include <cmath>

using namespace frc;

AnalogInput* irSensor = nullptr;

IRRangeSubsystem::IRRangeSubsystem(int analogPort) : m_analogPort(analogPort) {}

void IRRangeSubsystem::Init() {
    if (!irSensor) {
        irSensor = new AnalogInput(m_analogPort);
        std::cout << "IR Range sensor initialized on analog port " << m_analogPort << std::endl;
        std::cout.flush();
    }
}

double IRRangeSubsystem::GetVoltage() {
    if (irSensor) {
        return irSensor->GetVoltage();
    }
    std::cout << "IR Range: Sensor not initialized!" << std::endl;
    std::cout.flush();
    return 0.0;
}

double IRRangeSubsystem::VoltageToDistance(double voltage) {
    // Studica IR Range Sensor (10-80cm) voltage-to-distance conversion
    // Based on typical IR sensor characteristics - may need calibration
    
    if (voltage < kMinVoltage || voltage > kMaxVoltage) {
        return -1.0; // Invalid reading
    }
    
    // Linear interpolation between voltage points
    // Higher voltage = closer distance
    // This is an approximation - you may need to calibrate with actual measurements
    double normalizedVoltage = (voltage - kMinVoltage) / (kMaxVoltage - kMinVoltage);
    double distance = kMaxRange - (normalizedVoltage * (kMaxRange - kMinRange));
    
    return distance;
}

double IRRangeSubsystem::GetDistance() {
    if (!irSensor) {
        std::cout << "IR Range: Sensor not initialized!" << std::endl;
        std::cout.flush();
        return -1.0;
    }
    
    double voltage = GetVoltage();
    double distance = VoltageToDistance(voltage);
    
    // Debug output
    std::cout << "IR Range - Voltage: " << voltage << "V, Distance: " << distance << " cm" << std::endl;
    std::cout.flush();
    
    return distance;
}

bool IRRangeSubsystem::IsValidReading() {
    double distance = GetDistance();
    return (distance >= kMinRange && distance <= kMaxRange);
}

bool IRRangeSubsystem::IsObjectDetected(double threshold) {
    double distance = GetDistance();
    return (distance > 0 && distance < threshold && IsValidReading());
}

void IRRangeSubsystem::UpdateDashboard() {
    double distance = GetDistance();
    double voltage = GetVoltage();
    
    std::cout << "=== IR Range Distance: " << distance << " cm (Voltage: " << voltage << "V) ===" << std::endl;
    std::cout.flush();
    
    SmartDashboard::PutNumber("IR Range Distance (cm)", distance);
    SmartDashboard::PutNumber("IR Range Voltage (V)", voltage);
    SmartDashboard::PutBoolean("IR Object Detected", IsObjectDetected());
    SmartDashboard::PutBoolean("IR Valid Reading", IsValidReading());
    
    if (IsObjectDetected()) {
        std::cout << "*** IR OBJECT DETECTED ***" << std::endl;
        std::cout.flush();
    }
}

void IRRangeSubsystem::Periodic() {
    UpdateDashboard();
}

void IRRangeSubsystem::InitSendable(SendableBuilder& builder) {
    builder.SetSmartDashboardType("IR Range Subsystem");
    builder.AddDoubleProperty("Distance (cm)", [this] { return GetDistance(); }, nullptr);
    builder.AddDoubleProperty("Voltage (V)", [this] { return GetVoltage(); }, nullptr);
    builder.AddBooleanProperty("Object Detected", [this] { return IsObjectDetected(); }, nullptr);
    builder.AddBooleanProperty("Valid Reading", [this] { return IsValidReading(); }, nullptr);
}