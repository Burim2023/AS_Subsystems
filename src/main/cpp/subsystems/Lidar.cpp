/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2025 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Lidar.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>

using namespace frc;

LidarSubsystem::LidarSubsystem(studica::Lidar::Port port) 
    : m_lidar(nullptr), m_port(port), m_hasValidData(false) {
}

LidarSubsystem::~LidarSubsystem() {
    StopScan();
    if (m_lidar) {
        delete m_lidar;
    }
}

void LidarSubsystem::Init() {
    try {
        if (!m_lidar) {
            m_lidar = new studica::Lidar(m_port);
            std::cout << "Studica LiDAR sensor initialized on USB port " << static_cast<int>(m_port) << std::endl;
            std::cout.flush();
            
            // Give LiDAR time to initialize
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            // Enable Kalman filter by default for noise reduction
            ConfigureKalmanFilter();
            EnableFilter(studica::Lidar::kKALMAN, true);
            
            std::cout << "LiDAR initialization complete" << std::endl;
            std::cout.flush();
        }
    } catch (const std::exception& e) {
        std::cout << "LiDAR initialization failed: " << e.what() << std::endl;
        std::cout.flush();
    }
}

void LidarSubsystem::StartScan() {
    if (m_lidar) {
        try {
            m_lidar->Start();
            std::cout << "LiDAR scanning started" << std::endl;
            std::cout.flush();
            
            // Give LiDAR time to start scanning
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } catch (const std::exception& e) {
            std::cout << "LiDAR start scan failed: " << e.what() << std::endl;
            std::cout.flush();
        }
    } else {
        std::cout << "Cannot start LiDAR scan - sensor not initialized" << std::endl;
        std::cout.flush();
    }
}

void LidarSubsystem::StopScan() {
    if (m_lidar) {
        m_lidar->Stop();
        std::cout << "LiDAR scanning stopped" << std::endl;
        std::cout.flush();
    }
}

void LidarSubsystem::RestartScan() {
    if (m_lidar && m_restartState == RestartState::None) {
        // Check if enough time has passed since last restart attempt
        if (m_periodicCounter - m_lastRestartTime >= kRestartCooldown) {
            std::cout << "Initiating non-blocking LiDAR restart..." << std::endl;
            std::cout.flush();
            m_restartState = RestartState::Stopping;
            m_restartTimer = 0;
            m_lastRestartTime = m_periodicCounter;
        } else {
            std::cout << "LiDAR restart skipped - cooldown period active" << std::endl;
            std::cout.flush();
        }
    }
}

void LidarSubsystem::ProcessRestartStateMachine() {
    if (m_restartState == RestartState::None) {
        return;
    }
    
    m_restartTimer++;
    
    switch (m_restartState) {
        case RestartState::Stopping:
            if (m_restartTimer >= 2) {  // Small delay before stopping
                try {
                    m_lidar->Stop();
                    std::cout << "LiDAR stopped for restart" << std::endl;
                    std::cout.flush();
                    m_restartState = RestartState::Waiting;
                    m_restartTimer = 0;
                } catch (const std::exception& e) {
                    std::cout << "LiDAR stop failed during restart: " << e.what() << std::endl;
                    std::cout.flush();
                    m_restartState = RestartState::None;  // Abort restart
                }
            }
            break;
            
        case RestartState::Waiting:
            if (m_restartTimer >= kRestartStopDelay) {
                m_restartState = RestartState::Starting;
                m_restartTimer = 0;
            }
            break;
            
        case RestartState::Starting:
            if (m_restartTimer >= 2) {  // Small delay before starting
                try {
                    m_lidar->Start();
                    std::cout << "LiDAR restarted successfully" << std::endl;
                    std::cout.flush();
                    m_consecutiveErrors = 0;  // Reset error count
                    m_restartState = RestartState::None;
                    m_restartTimer = 0;
                } catch (const std::exception& e) {
                    std::cout << "LiDAR start failed during restart: " << e.what() << std::endl;
                    std::cout.flush();
                    m_restartState = RestartState::None;  // Abort restart
                }
            }
            break;
            
        default:
            m_restartState = RestartState::None;
            break;
    }
}

void LidarSubsystem::UpdateScanData() {
    if (m_lidar) {
        try {
            m_currentScan = m_lidar->GetData();
            m_hasValidData = true;
            m_consecutiveErrors = 0; // Reset error counter on success
        } catch (const std::exception& e) {
            m_consecutiveErrors++;
            
            // Log error but don't spam console
            if (m_consecutiveErrors == 1 || m_consecutiveErrors % 50 == 0) {
                std::cout << "LiDAR scan update error (attempt " << m_consecutiveErrors << "): " << e.what() << std::endl;
                std::cout.flush();
            }
            
            m_hasValidData = false;
            
            // Try to restart scanning after multiple consecutive errors (non-blocking)
            if (m_consecutiveErrors >= 10 && m_consecutiveErrors % 20 == 0 && m_restartState == RestartState::None) {
                std::cout << "Initiating automatic LiDAR restart..." << std::endl;
                std::cout.flush();
                RestartScan();  // This will use the non-blocking restart
            }
        }
    } else {
        m_hasValidData = false;
        if (m_consecutiveErrors == 0) {
            std::cout << "LiDAR not initialized!" << std::endl;
            std::cout.flush();
        }
        m_consecutiveErrors++;
    }
}

std::vector<LidarSubsystem::LidarPoint> LidarSubsystem::GetScanData() {
    std::vector<LidarPoint> points;
    
    if (!m_hasValidData) {
        UpdateScanData();
    }
    
    if (m_hasValidData) {
        points.reserve(360);
        for (int i = 0; i < 360; ++i) {
            double distance = MMtoCM(m_currentScan.distance[i]);
            bool valid = IsValidDistance(distance);
            points.emplace_back(m_currentScan.angle[i], distance, valid);
        }
    }
    
    return points;
}

double LidarSubsystem::GetDistanceAtAngle(double angle, double tolerance) {
    angle = NormalizeAngle(angle);
    
    if (!m_hasValidData) {
        UpdateScanData();
    }
    
    if (!m_hasValidData) {
        return -1.0;
    }
    
    // Find the closest angle within tolerance
    double minAngleDiff = tolerance + 1;
    double bestDistance = -1.0;
    
    for (int i = 0; i < 360; ++i) {
        double sensorAngle = m_currentScan.angle[i];
        double angleDiff = std::abs(sensorAngle - angle);
        
        // Handle wrap-around
        if (angleDiff > 180) {
            angleDiff = 360 - angleDiff;
        }
        
        if (angleDiff <= tolerance && angleDiff < minAngleDiff) {
            double distance = MMtoCM(m_currentScan.distance[i]);
            if (IsValidDistance(distance)) {
                bestDistance = distance;
                minAngleDiff = angleDiff;
            }
        }
    }
    
    return bestDistance;
}

double LidarSubsystem::GetClosestDistance(double startAngle, double endAngle) {
    startAngle = NormalizeAngle(startAngle);
    endAngle = NormalizeAngle(endAngle);
    
    if (!m_hasValidData) {
        UpdateScanData();
    }
    
    if (!m_hasValidData) {
        return -1.0;
    }
    
    double minDistance = kMaxRange;
    bool foundValid = false;
    
    for (int i = 0; i < 360; ++i) {
        double angle = m_currentScan.angle[i];
        
        // Handle angle wrap-around
        bool inRange = false;
        if (startAngle <= endAngle) {
            inRange = (angle >= startAngle && angle <= endAngle);
        } else {
            inRange = (angle >= startAngle || angle <= endAngle);
        }
        
        if (inRange) {
            double distance = MMtoCM(m_currentScan.distance[i]);
            if (IsValidDistance(distance) && distance < minDistance) {
                minDistance = distance;
                foundValid = true;
            }
        }
    }
    
    return foundValid ? minDistance : -1.0;
}

double LidarSubsystem::GetFrontDistance(double fieldOfView) {
    double halfFOV = fieldOfView / 2.0;
    return GetClosestDistance(360 - halfFOV, halfFOV);
}

bool LidarSubsystem::IsObstacleDetected(double angle, double threshold, double fieldOfView) {
    double halfFOV = fieldOfView / 2.0;
    double startAngle = NormalizeAngle(angle - halfFOV);
    double endAngle = NormalizeAngle(angle + halfFOV);
    
    double closestDistance = GetClosestDistance(startAngle, endAngle);
    return (closestDistance > 0 && closestDistance < threshold);
}

bool LidarSubsystem::IsPathClear(double threshold, double fieldOfView) {
    return !IsObstacleDetected(0, threshold, fieldOfView);
}

int LidarSubsystem::GetValidPointCount() {
    if (!m_hasValidData) {
        UpdateScanData();
    }
    
    if (!m_hasValidData) {
        return 0;
    }
    
    int count = 0;
    for (int i = 0; i < 360; ++i) {
        double distance = MMtoCM(m_currentScan.distance[i]);
        if (IsValidDistance(distance)) {
            count++;
        }
    }
    return count;
}

bool LidarSubsystem::IsScanning() {
    return (m_lidar != nullptr && m_hasValidData);
}

void LidarSubsystem::EnableFilter(studica::Lidar::Filter filter, bool enable) {
    if (m_lidar) {
        m_lidar->EnableFilter(filter, enable);
        std::cout << "LiDAR filter " << static_cast<int>(filter) << (enable ? " enabled" : " disabled") << std::endl;
        std::cout.flush();
    }
}

void LidarSubsystem::ConfigureKalmanFilter(float q, float r, float p) {
    if (m_lidar) {
        m_lidar->KalmanConfig(q, r, p);
        std::cout << "LiDAR Kalman filter configured: Q=" << q << ", R=" << r << ", P=" << p << std::endl;
        std::cout.flush();
    }
}

double LidarSubsystem::MMtoCM(double mm) {
    return mm / 10.0;
}

bool LidarSubsystem::IsValidDistance(double distance) {
    return (distance >= kMinRange && distance <= kMaxRange && distance > 0);
}

double LidarSubsystem::NormalizeAngle(double angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

void LidarSubsystem::UpdateDashboard() {
    // Update scan data
    UpdateScanData();
    
    double frontDistance = GetFrontDistance();
    int validPoints = GetValidPointCount();
    bool pathClear = IsPathClear();
    
    SmartDashboard::PutNumber("LiDAR Front Distance (cm)", frontDistance);
    SmartDashboard::PutNumber("LiDAR Valid Points", validPoints);
    SmartDashboard::PutBoolean("LiDAR Path Clear", pathClear);
    SmartDashboard::PutBoolean("LiDAR Scanning", IsScanning());
    SmartDashboard::PutBoolean("LiDAR Has Valid Data", m_hasValidData);
    SmartDashboard::PutBoolean("LiDAR Initialized", m_lidar != nullptr);
    SmartDashboard::PutNumber("LiDAR Error Count", m_consecutiveErrors);
    
    // Additional directional measurements
    SmartDashboard::PutNumber("LiDAR Left Distance (cm)", GetDistanceAtAngle(90));
    SmartDashboard::PutNumber("LiDAR Right Distance (cm)", GetDistanceAtAngle(270));
    SmartDashboard::PutNumber("LiDAR Rear Distance (cm)", GetDistanceAtAngle(180));
}

void LidarSubsystem::Periodic() {
    m_periodicCounter++;  // Track timing for restart cooldown
    ProcessRestartStateMachine();  // Handle non-blocking restart
    UpdateDashboard();
}

void LidarSubsystem::InitSendable(SendableBuilder& builder) {
    builder.SetSmartDashboardType("Studica LiDAR Subsystem");
    builder.AddDoubleProperty("Front Distance (cm)", [this] { return GetFrontDistance(); }, nullptr);
    builder.AddDoubleProperty("Left Distance (cm)", [this] { return GetDistanceAtAngle(90); }, nullptr);
    builder.AddDoubleProperty("Right Distance (cm)", [this] { return GetDistanceAtAngle(270); }, nullptr);
    builder.AddDoubleProperty("Rear Distance (cm)", [this] { return GetDistanceAtAngle(180); }, nullptr);
    builder.AddBooleanProperty("Path Clear", [this] { return IsPathClear(); }, nullptr);
    builder.AddBooleanProperty("Is Scanning", [this] { return IsScanning(); }, nullptr);
    builder.AddDoubleProperty("Valid Points", [this] { return GetValidPointCount(); }, nullptr);
}