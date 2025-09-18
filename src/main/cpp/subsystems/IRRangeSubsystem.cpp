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

IRRangeSubsystem::IRRangeSubsystem(int analogPort) : m_analogPort(analogPort) {}

void IRRangeSubsystem::Init() {
    if (!m_analogInput) {
        m_analogInput = std::make_unique<AnalogInput>(m_analogPort);
        std::cout << "IR Range sensor initialized on analog port " << m_analogPort << std::endl;
        std::cout.flush();
    }
}

double IRRangeSubsystem::GetVoltage() {
    if (m_analogInput) {
        return m_analogInput->GetVoltage();
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
    if (m_analogInput) {
        double voltage = m_analogInput->GetVoltage();
        double m_lastVoltage = voltage;
        
        // Debug: Print voltage for troubleshooting
        static int debugCounter = 0;
        if (debugCounter++ % 50 == 0) { // Print every 50 cycles (~1 second)
            std::cout << "IR Sensor Debug - Voltage: " << voltage << "V, Port: " << m_analogInput->GetChannel() << std::endl;
        }
        
        // Check if voltage is too low (likely hardware issue)
        if (voltage < 0.05) { // 50mV threshold
            return -1.0; // Hardware error
        }
        
        // Relaxed voltage range for troubleshooting
        if (voltage >= 0.05 && voltage <= 5.0) {
            // Simple voltage-to-distance conversion for debugging
            // Assuming typical IR sensor: higher voltage = closer object
            if (voltage < 0.4) {
                return 300.0; // Far distance (low voltage)
            } else if (voltage > 2.5) {
                return 10.0;  // Close distance (high voltage)
            } else {
                // Linear interpolation between 10cm and 300cm
                return 300.0 - ((voltage - 0.4) / (2.5 - 0.4)) * 290.0;
            }
        }
        
        return -1.0; // Invalid voltage range
    }
    return -1.0; // No sensor
}

bool IRRangeSubsystem::IsValidReading() {
    if (m_analogInput) {
        double voltage = m_analogInput->GetVoltage();
        // More permissive for debugging
        return voltage >= 0.05 && voltage <= 5.0;
    }
    return false;
}

bool IRRangeSubsystem::IsObjectDetected(double threshold) {
    double distance = GetDistance();
    return (distance > 0 && distance < threshold && IsValidReading());
}

void IRRangeSubsystem::UpdateDashboard() {
    double distance = GetDistance();
    double voltage = GetVoltage();
    bool isValid = IsValidReading();
    bool objectDetected = IsObjectDetected();
    
    frc::SmartDashboard::PutNumber("IR Range Distance (cm)", distance);
    frc::SmartDashboard::PutNumber("IR Range Voltage (V)", voltage);
    frc::SmartDashboard::PutBoolean("IR Object Detected", objectDetected);
    frc::SmartDashboard::PutBoolean("IR Valid Reading", isValid);
    
    // Enhanced debug status
    std::string status;
    if (!m_analogInput) {
        status = "ERROR: Not initialized";
    } else if (voltage < 0.05) {
        status = "HARDWARE ERROR: Voltage too low (" + std::to_string(voltage) + "V) - Check power/wiring";
    } else if (voltage > 5.0) {
        status = "ERROR: Voltage too high (" + std::to_string(voltage) + "V)";
    } else if (!isValid) {
        status = "Invalid voltage range: " + std::to_string(voltage) + "V";
    } else {
        status = "OK - " + std::to_string(distance) + " cm (" + std::to_string(voltage) + "V)";
    }
    
    frc::SmartDashboard::PutString("IR Debug Status", status);
    frc::SmartDashboard::PutString("IR Status", status);
    frc::SmartDashboard::PutNumber("IR Analog Port", m_analogInput ? m_analogInput->GetChannel() : -1);
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