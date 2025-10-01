#pragma once

#include "studica/Servo.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <memory>
#include <cmath>

#define JOINT_SERVO_PORT 21

#define JOINT_DOWN_ANGLE 0
#define JOINT_MID_ANGLE 95
#define JOINT_UP_ANGLE 120

// Preset speed constants for testing
#define SPEED_VERY_SLOW 0.5
#define SPEED_SLOW 1.0
#define SPEED_NORMAL 2.0
#define SPEED_FAST 5.0
#define SPEED_VERY_FAST 8.0

/**
 * Gripper Joint Subsystem
 * 
 * Controls the servo that moves the gripper joint up, down, and to mid position.
 * Supports variable speed control for smooth movement.
 */
class GripperJointSubsystem {
public:
    explicit GripperJointSubsystem();
    
    /**
     * Initialize the gripper joint servo
     */
    void Init();
    
    /**
     * Move gripper to up position (120 degrees)
     */
    void SetGripperUpAngle();
    
    /**
     * Move gripper to middle position (95 degrees)
     */
    void SetGripperMidAngle();
    
    /**
     * Move gripper to down position (0 degrees)
     */
    void SetGripperDownAngle();
    
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
    void SetSpeedVerySlow() { SetServoSpeed(SPEED_VERY_SLOW); }
    void SetSpeedSlow() { SetServoSpeed(SPEED_SLOW); }
    void SetSpeedNormal() { SetServoSpeed(SPEED_NORMAL); }
    void SetSpeedFast() { SetServoSpeed(SPEED_FAST); }
    void SetSpeedVeryFast() { SetServoSpeed(SPEED_VERY_FAST); }
    
    /**
     * Get current servo speed setting
     * @return Current speed in degrees per cycle
     */
    double GetServoSpeed() const { return servoSpeed; }
    
    /**
     * Get current servo angle
     * @return Current servo angle in degrees
     */
    double GetCurrentAngle() const { return servoAngleGripperJoint; }
    
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
    double GetDistanceToTarget() const { return std::abs(targetAngle - servoAngleGripperJoint); }
    
    /**
     * Update dashboard with current servo status
     */
    void UpdateDashboard();
    
    /**
     * Periodic function - handles gradual servo movement
     */
    void Periodic();

private:
    studica::Servo* GripperJointServo = nullptr;
    double servoAngleGripperJoint = JOINT_DOWN_ANGLE;
    double targetAngle = JOINT_DOWN_ANGLE;
    double servoSpeed = 2.0;
    static constexpr double kMovementTolerance = 0.5;
    static constexpr double kMinSpeed = 0.1;
    static constexpr double kMaxSpeed = 10.0;
};
