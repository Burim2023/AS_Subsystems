// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #pragma once

// #include <studica/Lidar.h>
// #include <memory>
// #include <mutex>

// namespace frc
// {
//   class LidarSubsystem
//   {
//   public:
//     LidarSubsystem(studica::Lidar::Port port = studica::Lidar::kUSB1);
//     ~LidarSubsystem();
//     void Init();
//     void UpdateLidar();
//     double GetDistanceAtAngle(double angle, double tolerance = 5.0);

//   private:
//     std::unique_ptr<studica::Lidar> m_lidar;
//     mutable std::mutex m_lidarMutex;
//     studica::Lidar::Port m_port;

//     studica::Lidar::ScanData m_currentScan;
//     bool m_hasValidData;
//     bool m_disabled; // Flag to disable LiDAR after repeated failures

//     static constexpr double kMinRange = 15.0;   // Minimum range in cm
//     static constexpr double kMaxRange = 1200.0; // Maximum range in cm

//     static double MMtoCM(double mm) { return mm / 10.0; }
//     static bool IsValidDistance(double distance) { return (distance >= kMinRange && distance <= kMaxRange && distance > 0); }
//     static double NormalizeAngle(double angle)
//     {
//       while (angle < 0)
//         angle += 360;
//       while (angle >= 360)
//         angle -= 360;
//       return angle;
//     }
//   };

// } // namespace frc