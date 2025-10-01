#pragma once

#include "studica/Servo.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <memory>
#include <cmath>

#define ARM_SERVO_PORT 18

#define HOME_ANGLE 5
#define DROP_APPLE_ANGLE 35
#define PICK_APPLE_ANGLE 195

// Preset speed constants for testing
#define ARM_SPEED_VERY_SLOW 0.5
#define ARM_SPEED_SLOW 1.0
#define ARM_SPEED_NORMAL 2.0
#define ARM_SPEED_FAST 5.0
#define ARM_SPEED_VERY_FAST 8.0

/**
 * Arm Subsystem
 * 
 * Controls the main arm servo for apple picking operations.
 * Supports variable speed control for smooth movement between positions.
 */
class ArmSubsystem {
public:
    explicit ArmSubsystem();
    
    /**
     * Initialize the arm servo
     */
    void Init();
    
    /**
     * Move arm to home position (5 degrees)
     */
    void SetHomePosition();
    
    /**
     * Move arm to drop apple position (35 degrees)
     */
    void SetDropApplePosition();
    
    /**
     * Move arm to pick apple position (195 degrees)
     */
    void SetPickApplePosition();
    
    /**
     * Move servo to zero position
     */
    void SetServoAngleZero();
    
    /**
     * Set the speed of servo movement
     * @param speed Degrees per robot cycle (0.1 to 10.0)
     *              Lower values = slower, smoother movement
     *              Higher values = faster movement
     *              Examples: 0.5 = very slow, 2.0 = normal, 8.0 = fast
     */
    void SetServoSpeed(double speed);
    
    /**
     * Convenience methods for preset speeds
     */
    void SetSpeedVerySlow() { SetServoSpeed(ARM_SPEED_VERY_SLOW); }
    void SetSpeedSlow() { SetServoSpeed(ARM_SPEED_SLOW); }
    void SetSpeedNormal() { SetServoSpeed(ARM_SPEED_NORMAL); }
    void SetSpeedFast() { SetServoSpeed(ARM_SPEED_FAST); }
    void SetSpeedVeryFast() { SetServoSpeed(ARM_SPEED_VERY_FAST); }
    
    /**
     * Get current servo speed setting
     * @return Current speed in degrees per cycle
     */
    double GetServoSpeed() const { return servoSpeed; }
    
    /**
     * Get current servo angle
     * @return Current servo angle in degrees
     */
    double GetCurrentAngle() const { return servoAngleArm; }
    
    /**
     * Get target servo angle
     * @return Target servo angle in degrees
     */
    double GetTargetAngle() const { return targetAngle; }
    
    /**
     * Check if servo is currently moving
     * @return True if servo is moving towards target, false if at target
     */
    bool IsMoving() const;
    
    /**
     * Get the distance remaining to target
     * @return Degrees remaining to reach target position
     */
    double GetDistanceToTarget() const { return std::abs(targetAngle - servoAngleArm); }
    
    /**
     * Update dashboard with current servo status
     */
    void UpdateDashboard();
    
    /**
     * Periodic function - handles gradual servo movement
     */
    void Periodic();

private:
    studica::Servo* ArmServo = nullptr;
    double servoAngleArm = HOME_ANGLE;
    double targetAngle = HOME_ANGLE;
    double servoSpeed = 2.0;
    static constexpr double kMovementTolerance = 0.5;
    static constexpr double kMinSpeed = 0.1;
    static constexpr double kMaxSpeed = 10.0;
};
