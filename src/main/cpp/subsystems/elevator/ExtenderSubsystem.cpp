#include "subsystems/elevator/ExtenderSubsystem.h"
#include "Constants.h"
#include <iostream>

#include "studica/Servo.h"
#include "studica/ServoContinuous.h"

#include <frc/DigitalInput.h>
#include <frc/Timer.h>


studica::Servo ExtenderServo{Constants::EXTENDER_SERVO_PORT};
frc::DigitalInput LimitSwitchStopBack(Constants::LIMIT_SWITCH_STOP_BACK_PORT); // Lower
frc::DigitalInput LimitSwitchStopFront(Constants::LIMIT_SWITCH_STOP_FRONT_PORT); // Upper
frc::Timer ExtenderTimer;

bool ExtenderInitialized = false;
bool TimerDone = false;

ExtenderSubsystem::ExtenderSubsystem() {}

void ExtenderSubsystem::Init() {
    // Reset state to initial values
    currentState = ExtenderState::Idle;
    MaxTimeFrontToBack = 0.0;
    ExtenderInitialized = false;
    TimerDone = false;
    
    // Initialize servo to stopped position
    ExtenderServo.Set(Constants::EXTENDER_DRIVE_STOP);
    
    // Reset and prepare timer
    ExtenderTimer.Reset();
    
    std::cout << "ExtenderSubsystem initialized - State: Idle" << std::endl;
}

void ExtenderSubsystem::ExtenderSubsystemCurrentState() {

    switch (ExtenderSubsystem::currentState) {
        case ExtenderState::Idle:
            // CALIBRATION: First go to back limit, then start timing to front limit
            if(!ExtenderInitialized) {
                std::cout << "Extender is Idle - Starting calibration" << std::endl;
                // First, go to back limit to establish starting position
                ExtenderServo.Set(Constants::EXTENDER_DRIVE_BACKWARD);
                currentState = ExtenderState::MovingBackward;
            }
            if(TimerDone) {
                std::cout << "Calibration complete! MaxTimeFrontToBack: " << MaxTimeFrontToBack << " seconds" << std::endl;
                TimerDone = false;
                // DO NOT RESET TIMER - Keep calibration value!
            }
            break;

        case ExtenderState::AtFront:
            ExtenderServo.Set(Constants::EXTENDER_DRIVE_BACKWARD);
            currentState = ExtenderState::MovingBackward;
            break;

        case ExtenderState::AtBack:
            currentState = ExtenderState::Idle;
            break;

        case ExtenderState::MovingForward:
            if(IsFrontLimitPressed()) {
                std::cout << "LimitSwitch Front pressed." << std::endl;
                ExtenderServo.Set(Constants::EXTENDER_DRIVE_STOP);
                
                // CALIBRATION: If this was a calibration run, save the time
                if(!ExtenderInitialized) {
                    ExtenderTimer.Stop();
                    double calibratedTime = ExtenderTimer.Get();
                    // Only update MaxTimeFrontToBack if we don't have a calibration yet OR if this is better
                    if (MaxTimeFrontToBack == 0.0 || calibratedTime > 0.5) {
                        MaxTimeFrontToBack = calibratedTime;
                        std::cout << "CALIBRATION COMPLETE: MaxTimeFrontToBack = " << MaxTimeFrontToBack << " seconds" << std::endl;
                    }
                    ExtenderInitialized = true;
                    TimerDone = true;
                }
                
                currentState = ExtenderState::AtFront;
            }
            break;

        case ExtenderState::MovingBackward:
            if(IsBackLimitPressed()) {
                std::cout << "LimitSwitch Back pressed." << std::endl;
                ExtenderServo.Set(Constants::EXTENDER_DRIVE_STOP);
                
                // If this is calibration, prepare to start timing
                if(!ExtenderInitialized) {
                    std::cout << "Calibration: At back limit - preparing to time travel to front" << std::endl;
                    currentState = ExtenderState::CalibratingAtBack;
                } else {
                    currentState = ExtenderState::AtBack;
                }
            }
            break;
        
        case ExtenderState::CalibratingAtBack:
            // Start the calibration timing from back to front
            std::cout << "Starting calibration timing: Back -> Front" << std::endl;
            ExtenderTimer.Reset();
            ExtenderTimer.Start();
            ExtenderServo.Set(Constants::EXTENDER_DRIVE_FORWARD);
            currentState = ExtenderState::MovingForward;
            break;
        default:
            // currentState = ExtenderState::Error
            std::cout << "ERROR: Extender State Failed" << std::endl;
            break;
    }
}

void ExtenderSubsystem::Periodic(){
    //ExtenderSubsystemCurrentState();
    // CRITICAL SAFETY: Check limit switches every periodic cycle
    bool frontPressed = IsFrontLimitPressed();
    bool backPressed = IsBackLimitPressed();
    
    // Debug output every 50 cycles (~1 second) to avoid spam
    static int debugCounter = 0;
    if (debugCounter++ % 200 == 0) {
        std::cout << "Periodic: Front=" << frontPressed << " Back=" << backPressed 
                  << " State=" << GetCurrentStateString() << std::endl;
    }
    
    // SAFETY: Stop immediately if limit switch is hit
    if (frontPressed && IsExtending()) {
        std::cout << "SAFETY: Front limit hit - STOPPING EXTEND" << std::endl;
        ExtenderServo.Set(Constants::EXTENDER_DRIVE_STOP);
        currentState = ExtenderState::AtFront;
    }
    
    if (backPressed && IsRetracting()) {
        std::cout << "SAFETY: Back limit hit - STOPPING RETRACT" << std::endl; 
        ExtenderServo.Set(Constants::EXTENDER_DRIVE_STOP);
        currentState = ExtenderState::AtBack;
    }
    
    // Additional safety - stop if both switches pressed (shouldn't happen)
    if (frontPressed && backPressed) {
        std::cout << "ERROR: Both limit switches pressed - EMERGENCY STOP" << std::endl;
        ExtenderServo.Set(Constants::EXTENDER_DRIVE_STOP);
        currentState = ExtenderState::Error;
    }
}

double ExtenderSubsystem::GetMaxTimeFrontToBack() const {
    return MaxTimeFrontToBack;
}

std::string ExtenderSubsystem::GetCurrentStateString() {
    switch (currentState) {
        case ExtenderState::Error:
            return "Error";
        case ExtenderState::Idle:
            return "Idle";
        case ExtenderState::AtFront:
            return "AtFront";
        case ExtenderState::AtBack:
            return "AtBack";
        case ExtenderState::MovingForward:
            return "MovingForward";
        case ExtenderState::MovingBackward:
            return "MovingBackward";
        case ExtenderState::CalibratingAtBack:
            return "CalibratingAtBack";
        default:
            return "Unknown";
    }
}

void ExtenderSubsystem::calibrate() {
    // Only recalibrate if not already calibrated or if explicitly requested
    if (IsCalibrated()) {
        std::cout << "ExtenderSubsystem: Already calibrated (MaxTime=" << MaxTimeFrontToBack 
                  << "s). Use ResetCalibration() to force recalibration." << std::endl;
        return;
    }
    
    // Reset calibration values and start calibration process
    MaxTimeFrontToBack = 0.0;
    ExtenderInitialized = false;
    TimerDone = false;
    currentState = ExtenderState::Idle;
    
    std::cout << "ExtenderSubsystem: Starting calibration process" << std::endl;
}

// New methods for command integration
void ExtenderSubsystem::MoveToFront() {
    if (currentState == ExtenderState::Idle || currentState == ExtenderState::AtBack) {
        ExtenderServo.Set(Constants::EXTENDER_DRIVE_FORWARD);
        currentState = ExtenderState::MovingForward;
        std::cout << "ExtenderSubsystem: Starting move to front" << std::endl;
    }
}

void ExtenderSubsystem::MoveToBack() {
    if (currentState == ExtenderState::Idle || currentState == ExtenderState::AtFront) {
        ExtenderServo.Set(Constants::EXTENDER_DRIVE_BACKWARD);
        currentState = ExtenderState::MovingBackward;
        std::cout << "ExtenderSubsystem: Starting move to back" << std::endl;
    }
}

void ExtenderSubsystem::Stop() {
    ExtenderServo.Set(Constants::EXTENDER_DRIVE_STOP);
    if (currentState == ExtenderState::MovingForward || currentState == ExtenderState::MovingBackward) {
        currentState = ExtenderState::Idle;
        std::cout << "ExtenderSubsystem: Movement stopped" << std::endl;
    }
}

ExtenderState ExtenderSubsystem::GetCurrentState() const {
    return currentState;
}

bool ExtenderSubsystem::IsAtFront() const {
    return currentState == ExtenderState::AtFront;
}

bool ExtenderSubsystem::IsAtBack() const {
    return currentState == ExtenderState::AtBack;
}

bool ExtenderSubsystem::IsMoving() const {
    return currentState == ExtenderState::MovingForward || currentState == ExtenderState::MovingBackward;
}

// Safety limit switch methods for NORMALLY CLOSED switches
bool ExtenderSubsystem::IsFrontLimitPressed() const {
    // For NORMALLY CLOSED switches: pressed = true when circuit opens
    bool pressed = LimitSwitchStopFront.Get();  // NO inversion needed for NC switches
    return pressed;
}

bool ExtenderSubsystem::IsBackLimitPressed() const {
    // For NORMALLY CLOSED switches: pressed = true when circuit opens  
    bool pressed = LimitSwitchStopBack.Get();   // NO inversion needed for NC switches
    return pressed;
}

bool ExtenderSubsystem::IsExtending() const {
    return currentState == ExtenderState::MovingForward;
}

bool ExtenderSubsystem::IsRetracting() const {
    return currentState == ExtenderState::MovingBackward;
}

// Calibration methods
bool ExtenderSubsystem::IsCalibrated() const {
    return MaxTimeFrontToBack > 0.0 && ExtenderInitialized;
}

void ExtenderSubsystem::ResetCalibration() {
    MaxTimeFrontToBack = 0.0;
    ExtenderInitialized = false;
    TimerDone = false;
    std::cout << "ExtenderSubsystem: Calibration reset - will recalibrate on next use" << std::endl;
}

void ExtenderSubsystem::SetMaxTimeFrontToBack(double time) {
    if (time > 0.0) {
        MaxTimeFrontToBack = time;
        ExtenderInitialized = true;  // Mark as calibrated
        std::cout << "ExtenderSubsystem: MaxTimeFrontToBack set to " << time << " seconds" << std::endl;
        std::cout << "ExtenderSubsystem: Calibration complete - extender ready for precise positioning" << std::endl;
    } else {
        std::cout << "ExtenderSubsystem: ERROR - Invalid time value " << time << " - not setting MaxTime" << std::endl;
    }
}

// State control methods for CalibrateExtender command
void ExtenderSubsystem::SetExtendState() {
    ExtenderServo.Set(Constants::EXTENDER_DRIVE_FORWARD);
    currentState = ExtenderState::MovingForward;
    std::cout << "ExtenderSubsystem: SetExtendState() - Moving forward to front limit" << std::endl;
}

void ExtenderSubsystem::SetRetractState() {
    ExtenderServo.Set(Constants::EXTENDER_DRIVE_BACKWARD);
    currentState = ExtenderState::MovingBackward;
    std::cout << "ExtenderSubsystem: SetRetractState() - Moving backward to back limit" << std::endl;
}

void ExtenderSubsystem::SetStopState() {
    ExtenderServo.Set(Constants::EXTENDER_DRIVE_STOP);
    if (currentState == ExtenderState::MovingForward || currentState == ExtenderState::MovingBackward) {
        currentState = ExtenderState::Idle;
    }
    std::cout << "ExtenderSubsystem: SetStopState() - Movement stopped" << std::endl;
}