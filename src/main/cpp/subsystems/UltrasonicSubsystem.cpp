/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/UltrasonicSubsystem.h"
#include <iostream>

using namespace frc;

/**
 * Constructor — stores the trigger/echo port numbers for both sensors.
 */
UltrasonicSubsystem::UltrasonicSubsystem(int leftTrigger, int leftEcho, int rightTrigger, int rightEcho)
    : m_leftTriggerPort(leftTrigger),
      m_leftEchoPort(leftEcho),
      m_rightTriggerPort(rightTrigger),
      m_rightEchoPort(rightEcho) {}

/**
 * Initializes both ultrasonic sensors and enables automatic mode.
 */
void UltrasonicSubsystem::Init() {
    // Create the left ultrasonic sensor if not already created
    if (!m_leftSensor) {
        m_leftSensor = std::make_unique<Ultrasonic>(
            m_leftTriggerPort, m_leftEchoPort, Ultrasonic::kMilliMeters);
    }

    // Create the right ultrasonic sensor if not already created
    if (!m_rightSensor) {
        m_rightSensor = std::make_unique<Ultrasonic>(
            m_rightTriggerPort, m_rightEchoPort, Ultrasonic::kMilliMeters);
    }

    // Enable automatic background pinging for all Ultrasonic objects
    Ultrasonic::SetAutomaticMode(true);

    // // Console output for debugging
    // std::cout << "Ultrasonic sensors initialized:" << std::endl;
    // std::cout << " Left  -> Trigger: " << m_leftTriggerPort 
    //           << ", Echo: " << m_leftEchoPort << std::endl;
    // std::cout << " Right -> Trigger: " << m_rightTriggerPort 
    //           << ", Echo: " << m_rightEchoPort << std::endl;
    // std::cout.flush();
}

/**
 * Reads the current distance from the left ultrasonic sensor.
 * Converts millimeters → centimeters.
 */
double UltrasonicSubsystem::GetLeftDistance() {
    if (m_leftSensor && m_leftSensor->IsRangeValid()) {
        return m_leftSensor->GetRangeMM() / 10.0;  // mm → cm
    }
    return 0.0; // 0 means invalid or not ready
}

/**
 * Reads the current distance from the right ultrasonic sensor.
 * Converts millimeters → centimeters.
 */
double UltrasonicSubsystem::GetRightDistance() {
    if (m_rightSensor && m_rightSensor->IsRangeValid()) {
        return m_rightSensor->GetRangeMM() / 10.0;  // mm → cm
    }
    return 0.0; // 0 means invalid or not ready
}

/**
 * Checks if a wall/object is detected on the left side.
 * Returns true if distance < 15 cm and valid.
 */
bool UltrasonicSubsystem::IsLeftWallDetected() {
    double distance = GetLeftDistance();
    return (distance > 0 && distance < kDefaultThreshold);
}

/**
 * Checks if a wall/object is detected on the right side.
 * Returns true if distance < 15 cm and valid.
 */
bool UltrasonicSubsystem::IsRightWallDetected() {
    double distance = GetRightDistance();
    return (distance > 0 && distance < kDefaultThreshold);
}

/**
 * Updates SmartDashboard with distance and wall detection info.
 * Helps during testing and autonomous tuning.
 */
void UltrasonicSubsystem::UpdateDashboard() {
    double leftDist = GetLeftDistance();
    double rightDist = GetRightDistance();

    // Display distance readings
    SmartDashboard::PutNumber("Left Ultrasonic Distance (cm)", leftDist);
    SmartDashboard::PutNumber("Right Ultrasonic Distance (cm)", rightDist);

    // Display wall detection states
    SmartDashboard::PutBoolean("Left Wall Detected (<15cm)", IsLeftWallDetected());
    SmartDashboard::PutBoolean("Right Wall Detected (<15cm)", IsRightWallDetected());
}

/**
 * Called periodically by the FRC framework (every robot loop).
 * Keeps SmartDashboard values up to date.
 */
void UltrasonicSubsystem::Periodic() {
    UpdateDashboard();
}

/**
 * Allows SmartDashboard and Shuffleboard to display live sensor values.
 */
void UltrasonicSubsystem::InitSendable(SendableBuilder& builder) {
    builder.SetSmartDashboardType("UltrasonicSubsystem");

    // Expose left and right distances
    builder.AddDoubleProperty("Left Distance (cm)", 
        [this] { return GetLeftDistance(); }, nullptr);
    builder.AddDoubleProperty("Right Distance (cm)", 
        [this] { return GetRightDistance(); }, nullptr);

    // Expose wall detection booleans
    builder.AddBooleanProperty("Left Wall Detected (<15cm)", 
        [this] { return IsLeftWallDetected(); }, nullptr);
    builder.AddBooleanProperty("Right Wall Detected (<15cm)", 
        [this] { return IsRightWallDetected(); }, nullptr);
}