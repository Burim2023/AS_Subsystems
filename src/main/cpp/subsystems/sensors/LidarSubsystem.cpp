/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/sensor/LidarSubsystem.h"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace frc;

LidarSubsystem::LidarSubsystem(studica::Lidar::Port port)
    : m_lidar(nullptr), m_port(port), m_hasValidData(false), m_disabled(false) {}

LidarSubsystem::~LidarSubsystem()
{
    std::lock_guard<std::mutex> lock(m_lidarMutex);
    if (m_lidar)
    {
        try
        {
            m_lidar->Stop();
        }
        catch (...)
        {
        } // Ignore errors during destruction
    }
}

void LidarSubsystem::Init()
{
    std::lock_guard<std::mutex> lock(m_lidarMutex);

    if (m_disabled)
    {
        // LiDAR has been disabled due to repeated failures
        return;
    }

    if (!m_lidar)
    {
        try
        {
            m_lidar = std::make_unique<studica::Lidar>(m_port);
            m_lidar->KalmanConfig(1e-5, 1e-1, 1.0);
            m_lidar->EnableFilter(studica::Lidar::kKALMAN, true);
            m_lidar->Start();
        }
        catch (const std::exception &e)
        {
            std::cerr << "LiDAR init failed: " << e.what() << std::endl;
            m_disabled = true; // Disable LiDAR to prevent further crashes
            m_lidar.reset();   // Clean up any partial initialization
            // Don't throw - allow robot to continue without LiDAR
        }
        catch (...)
        {
            std::cerr << "LiDAR init failed with unknown exception" << std::endl;
            m_disabled = true;
            m_lidar.reset();
        }
    }
}

void LidarSubsystem::UpdateLidar()
{
    std::lock_guard<std::mutex> lock(m_lidarMutex);

    if (m_disabled)
    {
        // LiDAR has been disabled - skip update
        m_hasValidData = false;
        return;
    }

    if (m_lidar)
    {
        try
        {
            // WARNING: GetData() can block for up to 200+ seconds if USB device is slow/failing
            // There's no timeout mechanism in Studica library, so we rely on exception handling
            m_currentScan = m_lidar->GetData();
            m_hasValidData = true;
        }
        catch (const std::exception &e)
        {
            // Log error but don't spam console
            static int errorCount = 0;
            if (++errorCount % 50 == 0) // Log every 50th error
            {
                std::cerr << "LiDAR GetData() error: " << e.what()
                          << " (count=" << errorCount << ")" << std::endl;

                // After 500 errors (10 minutes at 50Hz), disable LiDAR permanently
                if (errorCount >= 500)
                {
                    std::cerr << "LiDAR: Too many errors, disabling permanently" << std::endl;
                    m_disabled = true;
                    m_lidar->Stop();
                    m_lidar.reset();
                }
            }
            m_hasValidData = false;
        }
        catch (...)
        {
            static int unknownErrorCount = 0;
            if (++unknownErrorCount % 50 == 0)
            {
                std::cerr << "LiDAR GetData() unknown error (count="
                          << unknownErrorCount << ")" << std::endl;

                // After 500 errors, disable LiDAR permanently
                if (unknownErrorCount >= 500)
                {
                    std::cerr << "LiDAR: Too many unknown errors, disabling permanently" << std::endl;
                    m_disabled = true;
                    try
                    {
                        m_lidar->Stop();
                    }
                    catch (...)
                    {
                    }
                    m_lidar.reset();
                }
            }
            m_hasValidData = false;
        }
    }
    else
    {
        m_hasValidData = false;
    }
}

double LidarSubsystem::GetDistanceAtAngle(double angle, double tolerance)
{
    std::lock_guard<std::mutex> lock(m_lidarMutex);

    angle = NormalizeAngle(angle);

    if (!m_hasValidData)
    {
        return -1.0;
    }

    // Find the closest angle within tolerance
    double minAngleDiff = tolerance + 1;
    double bestDistance = -1.0;

    for (int i = 0; i < 360; ++i)
    {
        double sensorAngle = m_currentScan.angle[i];
        double angleDiff = std::abs(sensorAngle - angle);

        // Handle wrap-around
        if (angleDiff > 180)
        {
            angleDiff = 360 - angleDiff;
        }

        if (angleDiff <= tolerance && angleDiff < minAngleDiff)
        {
            double distance = MMtoCM(m_currentScan.distance[i]);
            if (IsValidDistance(distance))
            {
                bestDistance = distance;
                minAngleDiff = angleDiff;
            }
        }
    }

    return bestDistance;
}
