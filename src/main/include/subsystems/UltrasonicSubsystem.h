/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Ultrasonic.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableHelper.h>
#include <frc/smartdashboard/SendableBuilder.h>

namespace frc {

/**
 * Ultrasonic rangefinder subsystem class.
 *
 * This subsystem wraps the WPILib Ultrasonic sensor functionality
 * to provide distance measurement and wall detection capabilities
 * for autonomous robot navigation. The subsystem handles sensor
 * initialization, periodic updates, and dashboard integration.
 */
class UltrasonicSubsystem : public Sendable,
                           public SendableHelper<UltrasonicSubsystem> {
 public:
  /**
   * Create an instance of the UltrasonicSubsystem.
   *
   * Initializes the subsystem but does not create the sensor hardware
   * until Init() is called.
   */
  UltrasonicSubsystem();

  /**
   * Initialize the ultrasonic sensor hardware.
   *
   * Creates and configures the WPILib Ultrasonic sensor with the
   * specified trigger and echo ports. Enables automatic mode for
   * continuous measurements.
   */
  void Init();

  /**
   * Get the current distance measurement from the ultrasonic sensor.
   *
   * @return Distance in centimeters to the nearest object. Returns 0.0
   *         if the range is not valid yet, or -1.0 if the sensor is
   *         not initialized.
   */
  double GetDistance();

  /**
   * Update the SmartDashboard with current sensor readings.
   *
   * Publishes distance measurement and wall detection status to the
   * dashboard for monitoring and debugging purposes.
   */
  void UpdateDashboard();

  /**
   * Periodic function called by the robot framework.
   *
   * Updates the dashboard with current sensor readings on each
   * robot cycle.
   */
  void Periodic();

  /**
   * Check if a wall is detected within the specified threshold distance.
   *
   * @param threshold Maximum distance in centimeters to consider as
   *                  wall detected. Default is 10.0 cm.
   * @return True if a valid distance measurement is below the threshold,
   *         false otherwise.
   */
  bool IsWallDetected(double threshold = 10.0);

  void InitSendable(SendableBuilder& builder) override;

 private:
  // Digital I/O port definitions
  static constexpr int TRIGGER_PORT = 8;
  static constexpr int ECHO_PORT = 9;
  
  // Default threshold for wall detection in centimeters
  static constexpr double kDefaultThreshold = 10.0;
};

}  // namespace frc