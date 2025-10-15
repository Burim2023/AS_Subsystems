#include "subsystems/elevator.h"

#include <chrono>
#include <AMCU.h>
#include <thread>
#include <frc/smartdashboard/SmartDashboard.h>

#define ELEVATOR_SPEED_RPM 70.f
#define ELEVATOR_PINION_CIRCUMFERENCE 80.1f  // Updated to mm (was 8.00854799253f cm)
#define ENCODER_TICKS_PER_REVOLUTION 1464.f

// ⚙️ Gear settings (keeping your gear system)
#define MOTOR_GEAR_TEETH 48.f
#define DRIVEN_GEAR_TEETH 64.f
#define GEAR_RATIO (MOTOR_GEAR_TEETH / DRIVEN_GEAR_TEETH)

// 🔁 Direction correction (set to -1 if gears reverse direction)
#define GEAR_DIRECTION -1.f

// ⚙️ Elevator movement limits
#define CALIBRATION_OFFSET_MM 15.f  // Offset below zero after calibration

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
    while(!shouldStop.load()) {
        float dist = fabs(targetPos - elevator::currentPos);
        if(targetPos != -1 && dist > 0.1) {
            if(targetPos > elevator::currentPos) {
                targetRPM = -ELEVATOR_SPEED_RPM * (1 - exp(-dist / 10.f));
                
                if(targetRPM < -ELEVATOR_SPEED_RPM)
                    targetRPM = -ELEVATOR_SPEED_RPM;
            }
            else {
                targetRPM = ELEVATOR_SPEED_RPM * (1 - exp(-dist / 10.f));

                if(targetRPM > ELEVATOR_SPEED_RPM)
                    targetRPM = ELEVATOR_SPEED_RPM;

                if(targetRPM < 5)
                    targetRPM = 5;
            }
        }else if(targetRPM != 0) {
            targetRPM = 0;
        }
        frc::SmartDashboard::PutString("targetRPM", std::to_string(targetRPM));

        if(!calibrating && !driveFromLimitSwitchToZero && targetRPM != prevRPM) {
            // ⚙️ Apply direction correction to motor RPM
            acmu->setRPM(MOTOR_0, targetRPM * GEAR_DIRECTION);
            prevRPM = targetRPM;
        }
        
        float currentPos = elevator::currentPos.load();
        int newEncoderSteps = acmu->getEncoder(MOTOR_0) - prevEncoderSteps;
        // 🧮 Apply gear ratio and direction correction in position calculation
        elevator::currentPos.store(currentPos + 
            ((newEncoderSteps * ELEVATOR_PINION_CIRCUMFERENCE * GEAR_RATIO) / ENCODER_TICKS_PER_REVOLUTION));
        prevEncoderSteps += newEncoderSteps;

        if(driveFromLimitSwitchToZero) {
            if(elevator::currentPos >= CALIBRATION_OFFSET_MM) {
                acmu->setRPM(MOTOR_0, 0);
                driveFromLimitSwitchToZero.store(false);
            }else if(newEncoderSteps < 5) {
                // ⚙️ Apply direction correction for recovery movement
                acmu->setRPM(MOTOR_0, -15 * GEAR_DIRECTION);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        frc::SmartDashboard::PutString("Current elevator pos", std::to_string(currentPos));
        std::this_thread::sleep_for(std::chrono::nanoseconds(30));
    }
}

void elevator::destroy() {
    shouldStop.store(true);
    if(motorHandlerThread->joinable())
        motorHandlerThread->join();

    delete motorHandlerThread;
}

void elevator::init(AMCU* p_acmu) {
    acmu = p_acmu;
    acmu->setLimitSwitches(MOTOR_0, 0, 1, 0, 0);
    acmu->registerLimitSwitchCallback(&limitswitchcallback);

    motorHandlerThread = new std::thread(motorHandler);
}

void elevator::moveTo(const float p_targetPos) {
    targetPos.store(p_targetPos);
}

void elevator::calibrate() {
    frc::SmartDashboard::PutString("calibrating", "elevator");
    // ⚙️ Move downward for calibration (positive RPM with GEAR_DIRECTION = -1 goes down)
    acmu->setRPM(MOTOR_0, 15 * GEAR_DIRECTION);
    calibrating.store(true);
}

void elevator::limitswitchcallback(uint8_t motorNr, uint8_t high) {
    if(calibrating.load()) {
        // ⚙️ Apply direction correction for recovery movement
        
        currentPos.store(0.0f);
        acmu->setRPM(MOTOR_0, -15 * GEAR_DIRECTION);
        calibrating.store(false);
        driveFromLimitSwitchToZero.store(true);
    }
}
