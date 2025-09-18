/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/AnalogInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableHelper.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <memory>

namespace frc {

/**
 * Studica IR Range Sensor subsystem class.
 *
 * This subsystem wraps the Studica IR Range Sensor (10cm to 80cm) functionality
 * to provide distance measurement capabilities for autonomous robot navigation.
 * The sensor uses analog voltage output that corresponds to distance measurements.
 * 
 * Sensor specifications:
 * - Range: 10cm to 80cm
 * - Output: Analog voltage (higher voltage = closer distance)
 * - Supply voltage: 5V
 * - Interface: Analog input port
 */
class IRRangeSubsystem : public Sendable,
                        public SendableHelper<IRRangeSubsystem> {
 public:
  /**
   * Create an instance of the IRRangeSubsystem.
   *
   * @param analogPort The analog input port number (0-3 on RoboRIO)
   */
  IRRangeSubsystem(int analogPort = 0);

  /**
   * Initialize the IR range sensor hardware.
   *
   * Sets up the analog input for reading the sensor voltage.
   */
  void Init();

  /**
   * Get the current distance measurement from the IR range sensor.
   *
   * Converts the analog voltage reading to distance in centimeters
   * based on the sensor's calibration curve.
   *
   * @return Distance in centimeters to the nearest object. Returns -1.0
   *         if the reading is outside the valid range (10-80cm).
   */
  double GetDistance();

  /**
   * Get the raw voltage reading from the sensor.
   *
   * @return Raw voltage value from the analog input (0-5V)
   */
  double GetVoltage();

  /**
   * Check if an object is detected within the specified threshold distance.
   *
   * @param threshold Maximum distance in centimeters to consider as
   *                  object detected. Default is 20.0 cm.
   * @return True if a valid distance measurement is below the threshold,
   *         false otherwise.
   */
  bool IsObjectDetected(double threshold = 15.0);

  /**
   * Check if the sensor reading is within the valid range.
   *
   * @return True if the distance reading is between 10cm and 80cm,
   *         false otherwise.
   */
  bool IsValidReading();

  /**
   * Update the SmartDashboard with current sensor readings.
   *
   * Publishes distance measurement, voltage, and object detection status
   * to the dashboard for monitoring and debugging purposes.
   */
  void UpdateDashboard();

  /**
   * Periodic function called by the robot framework.
   *
   * Updates the dashboard with current sensor readings on each
   * robot cycle.
   */
  void Periodic();

  void InitSendable(SendableBuilder& builder) override;

 private:
  // Analog input for the IR sensor - proper member variable
  std::unique_ptr<AnalogInput> m_analogInput;
  
  // Analog input port for the IR sensor
  int m_analogPort;
  
  // Sensor range constants
  static constexpr double kMinRange = 10.0;  // Minimum range in cm
  static constexpr double kMaxRange = 80.0;  // Maximum range in cm
  
  // Voltage to distance conversion constants (may need calibration)
  static constexpr double kMaxVoltage = 4.8;  // Voltage at minimum distance
  static constexpr double kMinVoltage = 0.4;  // Voltage at maximum distance
  
  // Default threshold for object detection in centimeters
  static constexpr double kDefaultThreshold = 15.0;
  
  /**
   * Convert voltage reading to distance in centimeters.
   *
   * Uses the sensor's characteristic curve to convert analog voltage
   * to distance measurement.
   *
   * @param voltage Raw voltage reading from sensor
   * @return Distance in centimeters, or -1.0 if invalid
   */
  double VoltageToDistance(double voltage);
};

}  // namespace frc