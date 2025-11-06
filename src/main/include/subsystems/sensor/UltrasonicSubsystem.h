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
#include <mutex>

namespace frc
{

  class UltrasonicSubsystem
  {
  public:
    UltrasonicSubsystem(int leftTrigger, int leftEcho, int rightTrigger, int rightEcho);
    ~UltrasonicSubsystem();

    void Init();
    double GetLeftDistance();
    double GetRightDistance();
    void UpdateUltraSonic();
    bool IsLeftWallDetected();
    bool IsRightWallDetected();

  private:
    int m_leftTriggerPort;
    int m_leftEchoPort;
    int m_rightTriggerPort;
    int m_rightEchoPort;

    static constexpr double kDefaultThreshold = 15.0;

    double getMedian(std::vector<double> &values);
    double leftValue;
    double rightValue;

    std::unique_ptr<Ultrasonic> m_leftSensor;
    std::unique_ptr<Ultrasonic> m_rightSensor;
    std::vector<double> sideLeftValueList;
    std::vector<double> sideRightValueList;

    mutable std::mutex m_mutex;
  };

}
