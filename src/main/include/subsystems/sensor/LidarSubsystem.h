/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <studica/Lidar.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableHelper.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <vector>
#include <cmath>

namespace frc {

/**
 * Studica 360-degree LiDAR sensor subsystem class.
 *
 * This subsystem wraps the Studica LiDAR library to provide
 * comprehensive distance measurements in all directions.
 * The LiDAR provides 360-degree scanning with distance measurements
 * for autonomous navigation and obstacle detection.
 * 
 * Sensor specifications:
 * - Range: 0.15m to 12m
 * - Angular resolution: 1 degree
 * - Scan frequency: 5-15 Hz
 * - Interface: USB connection
 * - Protocol: Studica LiDAR protocol
 */
class LidarSubsystem : public Sendable,
                      public SendableHelper<LidarSubsystem> {
 public:
  /**
   * Structure to hold a single LiDAR measurement point
   */
  struct LidarPoint {
    double angle;     // Angle in degrees (0-359)
    double distance;  // Distance in centimeters
    bool valid;       // Whether this measurement is valid
    
    LidarPoint() : angle(0), distance(0), valid(false) {}
    LidarPoint(double a, double d, bool v) : angle(a), distance(d), valid(v) {}
  };

  /**
   * Create an instance of the LidarSubsystem.
   *
   * @param port The USB port to use (kUSB1 or kUSB2)
   */
  LidarSubsystem(studica::Lidar::Port port = studica::Lidar::kUSB1);

  /**
   * Destructor - stops scanning and cleans up resources
   */
  ~LidarSubsystem();

  /**
   * Initialize the LiDAR sensor hardware.
   *
   * Creates the LiDAR object and starts scanning.
   */
  void Init();

  /**
   * Start LiDAR scanning.
   *
   * Begins continuous 360-degree scanning and data collection.
   */
  void StartScan();

  /**
   * Restart LiDAR scanning.
   *
   * Stops and restarts the LiDAR if it has connection issues.
   */
  void RestartScan();

  /**
   * Stop LiDAR scanning.
   *
   * Stops the scanning process and data collection.
   */
  void StopScan();

  /**
   * Get the current complete scan data.
   *
   * @return Vector of LidarPoint structures containing all measurement points
   */
  std::vector<LidarPoint> GetScanData();

  /**
   * Get distance measurement at a specific angle.
   *
   * @param angle Target angle in degrees (0-359)
   * @param tolerance Angle tolerance in degrees (default ±5°)
   * @return Distance in centimeters, or -1.0 if no valid measurement
   */
  double GetDistanceAtAngle(double angle, double tolerance = 5.0);

  /**
   * Get the closest obstacle distance in a specified angular range.
   *
   * @param startAngle Starting angle in degrees
   * @param endAngle Ending angle in degrees
   * @return Closest distance in centimeters, or -1.0 if no obstacles
   */
  double GetClosestDistance(double startAngle, double endAngle);

  /**
   * Get the closest obstacle distance in front of the robot.
   *
   * @param fieldOfView Angular field of view in degrees (default ±30°)
   * @return Closest distance in centimeters in front direction
   */
  double GetFrontDistance(double fieldOfView = 60.0);

  /**
   * Check if there are obstacles in a specified direction.
   *
   * @param angle Target angle in degrees
   * @param threshold Maximum distance to consider as obstacle (cm)
   * @param fieldOfView Angular field of view to check (degrees)
   * @return True if obstacle detected within threshold
   */
  bool IsObstacleDetected(double angle, double threshold = 50.0, double fieldOfView = 30.0);

  /**
   * Check if the path ahead is clear for navigation.
   *
   * @param threshold Minimum clear distance required (cm)
   * @param fieldOfView Angular field of view to check (degrees)
   * @return True if path is clear
   */
  bool IsPathClear(double threshold = 100.0, double fieldOfView = 60.0);

  /**
   * Get the number of valid measurement points in the current scan.
   *
   * @return Number of valid points
   */
  int GetValidPointCount();

  /**
   * Check if the LiDAR is currently scanning.
   *
   * @return True if actively scanning
   */
  bool IsScanning();

  /**
   * Enable a filter on the LiDAR data.
   *
   * @param filter The filter type to enable
   * @param enable True to enable, false to disable
   */
  void EnableFilter(studica::Lidar::Filter filter, bool enable);

  /**
   * Configure the Kalman filter for noise reduction.
   *
   * @param q Process noise (default 1e-5)
   * @param r Measurement noise (default 1e-1)
   * @param p Initial estimate error (default 1.0)
   */
  void ConfigureKalmanFilter(float q = 1e-5, float r = 1e-1, float p = 1.0);

  /**
   * Update the SmartDashboard with current LiDAR data.
   *
   * Publishes key measurements and status to the dashboard.
   */
  void UpdateDashboard();

  /**
   * Periodic function called by the robot framework.
   *
   * Updates the dashboard and processes LiDAR data.
   */
  void Periodic();

  void InitSendable(SendableBuilder& builder) override;

 private:
  // LiDAR hardware
  std::unique_ptr<studica::Lidar> m_lidar; 
  std::mutex m_lidarMutex; 
  studica::Lidar::Port m_port;
  
  // Current scan data
  studica::Lidar::ScanData m_currentScan;
  bool m_hasValidData;
  
  // Error tracking for automatic recovery
  int m_consecutiveErrors = 0;
  
  // Non-blocking restart state machine
  enum class RestartState {
    None,
    Stopping,
    Waiting,
    Starting
  };
  
  RestartState m_restartState = RestartState::None;
  int m_restartTimer = 0;
  int m_periodicCounter = 0;  // Track total periodic calls for timing
  int m_lastRestartTime = 0;  // Track when last restart was initiated
  static constexpr int kRestartStopDelay = 10;   // 10 cycles (~200ms at 50Hz)
  static constexpr int kRestartStartDelay = 25;  // 25 cycles (~500ms at 50Hz)
  static constexpr int kRestartCooldown = 250;   // 250 cycles (~5 seconds at 50Hz)
  
  // Constants
  static constexpr double kMinRange = 15.0;   // Minimum range in cm
  static constexpr double kMaxRange = 1200.0; // Maximum range in cm
  static constexpr int kMaxPoints = 360;      // Maximum points per scan
  
  /**
   * Update the current scan data from the LiDAR.
   */
  void UpdateScanData();
  
  /**
   * Process non-blocking restart state machine.
   */
  void ProcessRestartStateMachine();
  
  /**
   * Convert millimeters to centimeters.
   *
   * @param mm Distance in millimeters
   * @return Distance in centimeters
   */
  double MMtoCM(double mm);
  
  /**
   * Validate if a distance measurement is within valid range.
   *
   * @param distance Distance measurement in centimeters
   * @return True if valid, false otherwise
   */
  bool IsValidDistance(double distance);
  
  /**
   * Normalize angle to 0-359 degree range.
   *
   * @param angle Input angle in degrees
   * @return Normalized angle (0-359)
   */
  double NormalizeAngle(double angle);
};

}  // namespace frc