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
#include <mutex>

namespace frc
{
  class IRRangeSubsystem
  {
  public:
    IRRangeSubsystem(int leftAnalogPort, int rightAnalogPort);
    ~IRRangeSubsystem();
    void Init();
    double GetIRLeftDistance();
    double GetIRRightDistance();
    bool IRDistanceSimilar();
    void UpdateInfraRed();
    double GetIRRightVoltage();
    double GetIRRLeftVoltage();
    bool IsObjectDetected(double threshold = 15.0);

  private:
    static constexpr double kMinRange = 10.0;  // Minimum range in cm
    static constexpr double kMaxRange = 80.0;  // Maximum range in cm
    static constexpr double kMaxVoltage = 4.8; // Voltage at minimum distance
    static constexpr double kMinVoltage = 0.4; // Voltage at maximum distance
    static constexpr double kDefaultThreshold = 15.0;

    int m_leftAnalogPort;
    int m_rightAnalogPort;
    double getMedian(std::vector<double> &values);
    double irLeftValue;
    double irRightValue;

    std::unique_ptr<AnalogInput> m_irSideRight;
    std::unique_ptr<AnalogInput> m_irSideLeft;
    std::vector<double> irSideLeftValueList;
    std::vector<double> irSideRightValueList;

    mutable std::mutex m_mutex;
  };

}