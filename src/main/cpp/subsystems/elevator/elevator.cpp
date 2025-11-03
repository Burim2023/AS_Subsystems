#include "subsystems/elevator/elevator.h"

#include <chrono>
#include "subsystems/amcu/AMCU.h"
#include <thread>
#include <iostream>
#include <future>
#include <algorithm>
#include <frc/smartdashboard/SmartDashboard.h>

#define ELEVATOR_SPEED_RPM 50.f  // REDUCED from 70 to prevent overload
#define ELEVATOR_PINION_CIRCUMFERENCE 80.1f
#define ENCODER_TICKS_PER_REVOLUTION 1464.f

// ⚙️ Gear settings
#define MOTOR_GEAR_TEETH 48.f
#define DRIVEN_GEAR_TEETH 64.f
#define GEAR_RATIO (MOTOR_GEAR_TEETH / DRIVEN_GEAR_TEETH)

// 🔁 Direction correction
#define GEAR_DIRECTION -1.f

// ⚙️ Elevator movement limits
#define CALIBRATION_OFFSET_MM 15.f

AMCU* acmu;

std::thread* motorHandlerThread;
std::atomic<float> elevator::currentPos = std::atomic<float>(0.f);
std::atomic<float> targetPos = std::atomic<float>(-1.f);
std::atomic<bool> shouldStop = std::atomic<bool>(false);
std::atomic<bool> driveFromLimitSwitchToZero = std::atomic<bool>(false);
std::atomic<bool> calibrating{false};

int8_t prevRPM;
int8_t targetRPM;
int prevEncoderSteps = 0;

void motorHandler() {
    int consecutiveErrors = 0;
    
    while(!shouldStop.load()) {
        try {
            float currentPosition = elevator::currentPos.load();
            float target = targetPos.load();
            
            if(target >= 0) { // Valid target
                float error = target - currentPosition;
                float dist = fabs(error);
                
                // CRITICAL FIX: Larger deadband to prevent oscillation
                const float DEADBAND = 8.0f; // Increased from 5mm to 8mm
                
                if(dist > DEADBAND) {
                    // CRITICAL FIX: More conservative speed control
                    if(error > 0) {
                        // Moving up - REDUCED speeds
                        if(dist > 30.0f) {
                            targetRPM = -40; // Reduced from -70 to -40
                        } else if(dist > 15.0f) {
                            targetRPM = -25; // Reduced from -30 to -25
                        } else {
                            targetRPM = -12; // Reduced from -15 to -12
                        }
                    } else {
                        // Moving down - REDUCED speeds
                        if(dist > 30.0f) {
                            targetRPM = 40; // Reduced from 70 to 40
                        } else if(dist > 15.0f) {
                            targetRPM = 25; // Reduced from 30 to 25
                        } else {
                            targetRPM = 12; // Reduced from 15 to 12
                        }
                    }
                } else {
                    // Within deadband - STOP
                    targetRPM = 0;
                    
                    // CRITICAL FIX: Clear target to prevent hunting
                    if(dist <= DEADBAND && target != -1) {
                        std::cout << "Elevator reached target - clearing to prevent hunting" << std::endl;
                        targetPos.store(-1.0f);
                    }
                }
            } else {
                targetRPM = 0;
            }
            
            // CRITICAL FIX: Add error handling for AMCU communication
            if(!calibrating && !driveFromLimitSwitchToZero && targetRPM != prevRPM) {
                try {
                    // ⚙️ Apply direction correction to motor RPM
                    acmu->setRPM(MOTOR_0, targetRPM * GEAR_DIRECTION);
                    prevRPM = targetRPM;
                    consecutiveErrors = 0; // Reset error count on success
                } catch(...) {
                    consecutiveErrors++;
                    std::cout << "ERROR: Failed to set RPM, consecutive errors: " << consecutiveErrors << std::endl;
                    
                    // If too many errors, stop the elevator
                    if(consecutiveErrors > 5) {
                        std::cout << "CRITICAL: Too many AMCU errors - stopping elevator" << std::endl;
                        targetPos.store(-1.0f);
                        targetRPM = 0;
                        consecutiveErrors = 0;
                    }
                }
            }
            
            // CRITICAL FIX: Add error handling for encoder reading
            try {
                int currentEncoderValue = acmu->getEncoder(MOTOR_0);
                int newEncoderSteps = currentEncoderValue - prevEncoderSteps;
                
                // Only update if encoder reading is reasonable
                if(abs(newEncoderSteps) < 1000) { // Prevent huge jumps
                    elevator::currentPos.store(currentPosition + 
                        ((newEncoderSteps * ELEVATOR_PINION_CIRCUMFERENCE * GEAR_RATIO) / ENCODER_TICKS_PER_REVOLUTION));
                    prevEncoderSteps = currentEncoderValue;
                } else {
                    std::cout << "WARNING: Ignoring large encoder jump: " << newEncoderSteps << std::endl;
                }
            } catch(...) {
                std::cout << "ERROR: Failed to read encoder" << std::endl;
            }

            // CRITICAL FIX: Add error handling for calibration sequence
            if(driveFromLimitSwitchToZero) {
                try {
                    if(elevator::currentPos >= CALIBRATION_OFFSET_MM) {
                        acmu->setRPM(MOTOR_0, 0);
                        driveFromLimitSwitchToZero.store(false);
                        std::cout << "Calibration complete" << std::endl;
                    } else {
                        acmu->setRPM(MOTOR_0, -15 * GEAR_DIRECTION);
                    }
                } catch(...) {
                    std::cout << "ERROR: Calibration sequence failed" << std::endl;
                    driveFromLimitSwitchToZero.store(false);
                    calibrating.store(false);
                }
            }

            // CRITICAL FIX: Update SmartDashboard less frequently to reduce load
            static int dashboardCounter = 0;
            if(dashboardCounter++ % 10 == 0) { // Update every 10 cycles
                try {
                    frc::SmartDashboard::PutNumber("Elevator RPM", targetRPM);
                    frc::SmartDashboard::PutNumber("Elevator Pos", currentPosition);
                } catch(...) {
                    // Ignore SmartDashboard errors
                }
            }
            
        } catch(...) {
            std::cout << "CRITICAL ERROR: Exception in elevator motorHandler" << std::endl;
            // Emergency stop
            try {
                acmu->setRPM(MOTOR_0, 0);
            } catch(...) {}
            targetRPM = 0;
            targetPos.store(-1.0f);
        }
        
        // CRITICAL FIX: Increased sleep time to reduce CPU/communication load
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Increased from 100ms to 50ms for balance
    }
}

void elevator::destroy() {
    shouldStop.store(true);
    
    // CRITICAL FIX: Add timeout for thread joining
    if(motorHandlerThread) {
        if(motorHandlerThread->joinable()) {
            // Try to join with timeout
            auto future = std::async(std::launch::async, [&]{ motorHandlerThread->join(); });
            if(future.wait_for(std::chrono::seconds(2)) == std::future_status::timeout) {
                std::cout << "WARNING: Motor handler thread did not stop gracefully" << std::endl;
            }
        }
        delete motorHandlerThread;
        motorHandlerThread = nullptr;
    }
}

void elevator::init(AMCU* p_acmu) {
    acmu = p_acmu;
    
    // CRITICAL FIX: Add error handling for initialization
    try {
        acmu->setLimitSwitches(MOTOR_0, 0, 1, 0, 0);
        acmu->registerLimitSwitchCallback(&limitswitchcallback);
        
        // Reset encoder and position
        acmu->resetEncoder(MOTOR_0);
        prevEncoderSteps = 0;
        currentPos.store(0.0f);
        
        motorHandlerThread = new std::thread(motorHandler);
        
        std::cout << "Elevator initialized successfully" << std::endl;
    } catch(...) {
        std::cout << "CRITICAL ERROR: Failed to initialize elevator" << std::endl;
    }
}

void elevator::moveTo(const float p_targetPos) {
    // CRITICAL FIX: Clamp target to safe range
    float clampedTarget = std::max(0.0f, std::min(p_targetPos, ELEVATOR_HEIGHT));
    
    std::cout << "Elevator moving to: " << clampedTarget << "mm" << std::endl;
    targetPos.store(clampedTarget);
}

void elevator::calibrate() {
    std::cout << "Starting elevator calibration" << std::endl;
    
    try {
        frc::SmartDashboard::PutString("Elevator Status", "Calibrating");
        
        // Clear any existing target
        targetPos.store(-1.0f);
        
        // Start calibration sequence
        acmu->setRPM(MOTOR_0, 15 * GEAR_DIRECTION);
        calibrating.store(true);
    } catch(...) {
        std::cout << "ERROR: Failed to start calibration" << std::endl;
        calibrating.store(false);
    }
}

void elevator::limitswitchcallback(uint8_t motorNr, uint8_t high) {
    if(calibrating.load() && motorNr == MOTOR_0) {
        std::cout << "Limit switch triggered during calibration" << std::endl;
        
        try {
            currentPos.store(0.0f);
            acmu->resetEncoder(MOTOR_0);
            prevEncoderSteps = 0;
            
            acmu->setRPM(MOTOR_0, -15 * GEAR_DIRECTION);
            calibrating.store(false);
            driveFromLimitSwitchToZero.store(true);
        } catch(...) {
            std::cout << "ERROR: Limit switch callback failed" << std::endl;
            calibrating.store(false);
            driveFromLimitSwitchToZero.store(false);
        }
    }
}
