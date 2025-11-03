/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/sensor/UltrasonicSubsystem.h"
#include <iostream>

using namespace frc;

/**
 * Constructor — stores the trigger/echo port numbers for both sensors.
 */
UltrasonicSubsystem::UltrasonicSubsystem(int leftTrigger, int leftEcho, int rightTrigger, int rightEcho)
    : m_leftTriggerPort(leftTrigger),
      m_leftEchoPort(leftEcho),
      m_rightTriggerPort(rightTrigger),
      m_rightEchoPort(rightEcho) {

            leftValue = 0.0;
            rightValue = 0.0;
      }


UltrasonicSubsystem::~UltrasonicSubsystem(){}


void UltrasonicSubsystem::Init() {
    if (m_leftSensor == nullptr && m_rightSensor == nullptr){
        try
        {
            m_leftSensor = std::make_unique<frc::Ultrasonic>(m_leftTriggerPort, m_leftEchoPort);
            m_rightSensor = std::make_unique<frc::Ultrasonic>(m_rightTriggerPort, m_rightEchoPort);
            m_rightSensor->SetAutomaticMode(true);
            m_leftSensor->SetAutomaticMode(true);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}


void UltrasonicSubsystem::UpdateUltraSonic()
{
    try
    {
        if (sideLeftValueList.size() >= 9)
        {
            leftValue = getMedian(sideLeftValueList);
            sideLeftValueList.erase(sideLeftValueList.begin());
        }
        if (sideRightValueList.size() >= 9)
        {
            rightValue = getMedian(sideRightValueList);
            sideRightValueList.erase(sideRightValueList.begin());
        }
        sideLeftValueList.push_back(m_leftSensor->GetRangeMM());
        sideRightValueList.push_back(m_rightSensor->GetRangeMM());
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

double UltrasonicSubsystem::getMedian(std::vector<double> &values)
{
    if (values.empty())
        return 0.0;
    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    size_t mid = sorted.size() / 2;
    return sorted[mid];
}

double UltrasonicSubsystem::GetRightDistance()
{
    return rightValue / 10.0;
}

double UltrasonicSubsystem::GetLeftDistance()
{
    return leftValue / 10.0;
}

bool UltrasonicSubsystem::IsLeftWallDetected() {
    double distance = GetLeftDistance();
    return (distance > 0 && distance < kDefaultThreshold);
}

bool UltrasonicSubsystem::IsRightWallDetected() {
    double distance = GetRightDistance();
    return (distance > 0 && distance < kDefaultThreshold);
}