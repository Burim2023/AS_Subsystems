/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Ultrasonic.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableHelper.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <memory>

namespace frc {

/**
 * UltrasonicSubsystem
 * --------------------
 * This subsystem manages **two ultrasonic sensors** (left and right).
 * Each sensor measures distance to nearby objects independently.
 * 
 * The subsystem:
 * - Initializes both sensors
 * - Continuously updates distances
 * - Publishes readings and wall detection info to SmartDashboard
 */
class UltrasonicSubsystem : public Sendable,
                           public SendableHelper<UltrasonicSubsystem> {
 public:
  /**
   * Constructor for UltrasonicSubsystem.
   * 
   * @param leftTrigger  DIO port number for the left sensor trigger.
   * @param leftEcho     DIO port number for the left sensor echo.
   * @param rightTrigger DIO port number for the right sensor trigger.
   * @param rightEcho    DIO port number for the right sensor echo.
   */
  UltrasonicSubsystem(int leftTrigger, int leftEcho, int rightTrigger, int rightEcho);

  /**
   * Initializes both ultrasonic sensors and enables automatic mode.
   * Must be called before reading distances.
   */
  void Init();

  /**
   * Returns the distance measured by the left ultrasonic sensor.
   * @return Distance in centimeters (returns 0 if invalid).
   */
  double GetLeftDistance();

  /**
   * Returns the distance measured by the right ultrasonic sensor.
   * @return Distance in centimeters (returns 0 if invalid).
   */
  double GetRightDistance();

  /**
   * Updates the SmartDashboard with the latest distance readings and
   * wall detection status for both sensors.
   */
  void UpdateDashboard();

  /**
   * Called automatically by the robot framework each cycle.
   * Simply calls UpdateDashboard().
   */
  void Periodic();

  /**
   * Checks if an object (wall) is detected by the left sensor.
   * @return true if distance < 15 cm, false otherwise.
   */
  bool IsLeftWallDetected();

  /**
   * Checks if an object (wall) is detected by the right sensor.
   * @return true if distance < 15 cm, false otherwise.
   */
  bool IsRightWallDetected();

  /**
   * Adds properties for live SmartDashboard monitoring.
   */
  void InitSendable(SendableBuilder& builder) override;

 private:
  // Smart pointers to the two ultrasonic sensors
  std::unique_ptr<Ultrasonic> m_leftSensor;
  std::unique_ptr<Ultrasonic> m_rightSensor;

  // DIO port numbers for each sensor
  int m_leftTriggerPort;
  int m_leftEchoPort;
  int m_rightTriggerPort;
  int m_rightEchoPort;

  // Default wall detection threshold (in cm)
  static constexpr double kDefaultThreshold = 15.0;
};

}  // namespace frc
