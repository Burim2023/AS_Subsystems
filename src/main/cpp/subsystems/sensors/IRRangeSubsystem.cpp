/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/sensor/IRRangeSubsystem.h"
#include <iostream>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

IRRangeSubsystem::IRRangeSubsystem(int leftAnalogPort, int rightAnalogPort) : m_leftAnalogPort(leftAnalogPort), m_rightAnalogPort(rightAnalogPort) {}

IRRangeSubsystem::~IRRangeSubsystem() {}

void IRRangeSubsystem::Init()
{
    if (m_irSideLeft == nullptr && m_irSideRight == nullptr)
    {
        try
        {
            m_irSideLeft = std::make_unique<frc::AnalogInput>(m_leftAnalogPort);
            m_irSideRight = std::make_unique<frc::AnalogInput>(m_rightAnalogPort);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}

void IRRangeSubsystem::UpdateInfraRed()
{

    if (m_irSideLeft != nullptr && m_irSideRight != nullptr)
    {
        try
        {
            const size_t maxSamples = 9;
            double vLeft = m_irSideLeft->GetAverageVoltage();
            double vRight = m_irSideRight->GetAverageVoltage();

            // guard against zero or tiny voltages
            if (vLeft <= 0.0001)
            {
                vLeft = 0.0001;
            }
            if (vRight <= 0.0001)
            {
                vRight = 0.0001;
            }

            double rawLeft = std::pow(vLeft, -1.2045) * 27.726;
            double rawRight = std::pow(vRight, -1.2045) * 27.726 - 1.0;

            irSideLeftValueList.push_back(rawLeft);
            irSideRightValueList.push_back(rawRight);

            if (irSideLeftValueList.size() > maxSamples)
                irSideLeftValueList.erase(irSideLeftValueList.begin());
            if (irSideRightValueList.size() > maxSamples)
                irSideRightValueList.erase(irSideRightValueList.begin());

            if (!irSideLeftValueList.empty())
                irLeftValue = getMedian(irSideLeftValueList);
            if (!irSideRightValueList.empty())
                irRightValue = getMedian(irSideRightValueList);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}

// bool IRRangeSubsystem::LeftIsObjectDetected(double threshold) {
//     double distance = GetDistance();
//     return (distance > 0 && distance < threshold && IsValidReading());
// }

double IRRangeSubsystem::getMedian(std::vector<double> &values)
{
    if (values.empty())
        return 0.0;
    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    size_t mid = sorted.size() / 2;
    return sorted[mid];
}

double IRRangeSubsystem::GetIRLeftDistance()
{
    return irLeftValue;
}

double IRRangeSubsystem::GetIRRightDistance()
{
    return irRightValue;
}

bool IRRangeSubsystem::IRDistanceSimilar()
{
    double IRDistance = GetIRLeftDistance() - GetIRRightDistance();
    if (IRDistance > -1 && IRDistance < 1)
    {
        return true;
    }
    return false;
}

double IRRangeSubsystem::GetIRRLeftVoltage()
{
    if (m_irSideLeft)
    {
        return m_irSideLeft->GetVoltage();
    }
    return 0.0;
}

double IRRangeSubsystem::GetIRRightVoltage()
{
    if (m_irSideRight)
    {
        return m_irSideRight->GetVoltage();
    }
    return 0.0;
}
