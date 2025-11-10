#include "commands/StoreAppleCommand.h"
#include "commands/MoveArmToPosition.h"
#include "commands/MoveElevatorToPosition.h"
#include "commands/MoveGripperJointToPosition.h"
#include "commands/ExtendForDuration.h"

#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <iostream>
#include <cmath>

namespace storage {

StoreAppleCommand::StoreAppleCommand(ExtenderSubsystem& ext, ElevatorSubsystem& elev, ArmSubsystem& arm,
                                   GripperJointSubsystem& joint, GripperSubsystem& gripper,
                                   const std::string& color, double pickMaxTime)
    : m_extender(ext), m_elevator(elev), m_arm(arm), m_joint(joint), m_gripper(gripper),
      m_color(color), m_maxTime(pickMaxTime), m_autoDetect(false),
      m_state(State::INITIALIZING), m_targetSlot(-1), m_targetTravelTime(0),
      m_maxTimeFrontToBack(0.0), m_storageHeightMM(0.0), m_dropDwellMs(500) {
    
    SetName("StoreAppleCommand");
    AddRequirements({&m_extender, &m_elevator, &m_arm, &m_joint, &m_gripper});
}

StoreAppleCommand::StoreAppleCommand(ExtenderSubsystem& ext, ElevatorSubsystem& elev, ArmSubsystem& arm,
                                   GripperJointSubsystem& joint, GripperSubsystem& gripper)
    : m_extender(ext), m_elevator(elev), m_arm(arm), m_joint(joint), m_gripper(gripper),
      m_color(""), m_maxTime(0.0), m_autoDetect(true),
      m_state(State::INITIALIZING), m_targetSlot(-1), m_targetTravelTime(0),
      m_maxTimeFrontToBack(0.0), m_storageHeightMM(0.0), m_dropDwellMs(500) {
    
    SetName("StoreAppleCommandAuto");
    AddRequirements({&m_extender, &m_elevator, &m_arm, &m_joint, &m_gripper});
}

int StoreAppleCommand::SlotFromMaxTime(double maxtime, double t1, double t2) {
    if (!std::isfinite(maxtime)) {
        throw storage::InvalidTimeError();
    }
    
    if (maxtime < t1) {
        return 0;
    } else if (maxtime < t2) {
        return 1;
    } else {
        return 2;
    }
}

std::chrono::milliseconds StoreAppleCommand::TimeForSlot(int slot, double maxTimeFrontToBack) {
    storage::StorageConfig cfg;
    
    // Load config from NetworkTables
    auto nt = nt::NetworkTableInstance::GetDefault().GetTable("Storage");
    cfg.slot0Frac = nt->GetNumber("Slot0Frac", 0.17);
    cfg.slot1Frac = nt->GetNumber("Slot1Frac", 0.50);
    cfg.slot2Frac = nt->GetNumber("Slot2Frac", 0.83);
    
    return storage::AppleStorage::slot_center_time_ms(slot, maxTimeFrontToBack, cfg);
}

void StoreAppleCommand::SaveStateToNT(const StorageState& state) {
    auto nt = nt::NetworkTableInstance::GetDefault().GetTable("Storage");
    
    // Individual slot colors
    for (int i = 0; i < 3; ++i) {
        std::string key = "Slots/" + std::to_string(i) + "/Color";
        if (state.slots[i]) {
            nt->PutString(key, *state.slots[i]);
        } else {
            nt->PutString(key, "");
        }
    }
    
    // Aggregate status
    int occupiedCount = 0;
    for (const auto& slot : state.slots) {
        if (slot) occupiedCount++;
    }
    
    nt->PutBoolean("IsFull", occupiedCount == 3);
    nt->PutBoolean("IsEmpty", occupiedCount == 0);
    nt->PutNumber("OccupiedSlots", occupiedCount);
    
    // JSON representation
    storage::AppleStorage storage;
    for (int i = 0; i < 3; ++i) {
        if (state.slots[i]) {
            try {
                storage.store(*state.slots[i], (i == 0) ? 0.5 : (i == 1) ? 1.5 : 3.0, 1.0, storage::StorageConfig());
            } catch (...) {
                // Ignore errors during reconstruction
            }
        }
    }
    nt->PutString("StateJson", storage.to_json());
}

void StoreAppleCommand::LoadStateFromNT(StorageState& state) {
    auto nt = nt::NetworkTableInstance::GetDefault().GetTable("Storage");
    
    for (int i = 0; i < 3; ++i) {
        std::string key = "Slots/" + std::to_string(i) + "/Color";
        std::string color = nt->GetString(key, "");
        
        if (!color.empty()) {
            state.slots[i] = color;
        } else {
            state.slots[i] = std::nullopt;
        }
    }
}

void StoreAppleCommand::Initialize() {
    std::cout << "=== StoreAppleCommand::Initialize ===" << std::endl;
    
    m_state = State::INITIALIZING;
    m_timer.Reset();
    m_timer.Start();
    m_stateTimer.Reset();
    m_stateTimer.Start();
    
    loadConfiguration();
    
    // Auto-detect color and maxtime if needed
    if (m_autoDetect) {
        try {
            m_color = readColorFromSmartDashboard();
            m_maxTime = m_extender.GetMaxTimeFrontToBack();
        } catch (const std::exception& e) {
            setError(std::string("Auto-detection failed: ") + e.what());
            return;
        }
    }
    
    // Validate inputs
    if (!std::isfinite(m_maxTime)) {
        setError("Invalid maxTime value (NaN or Inf)");
        return;
    }
    
    if (m_color != "red" && m_color != "green" && m_color != "yellow") {
        setError("Invalid color '" + m_color + "'. Allowed: red, green, yellow");
        return;
    }
    
    // Check extender calibration
    m_maxTimeFrontToBack = m_extender.GetMaxTimeFrontToBack();
    if (m_maxTimeFrontToBack <= 0.0) {
        setError("Extender not calibrated");
        return;
    }
    
    // Compute target slot
    try {
        m_targetSlot = SlotFromMaxTime(m_maxTime, m_config.t1, m_config.t2);
        m_targetTravelTime = TimeForSlot(m_targetSlot, m_maxTimeFrontToBack);
    } catch (const std::exception& e) {
        setError(std::string("Slot computation failed: ") + e.what());
        return;
    }
    
    // Check slot availability and apply fallback
    StorageState& state = StorageState::getInstance();
    LoadStateFromNT(state);
    
    for (int attempt = 0; attempt < 3; ++attempt) {
        int candidateSlot = (m_targetSlot + attempt) % 3;
        
        if (!state.slots[candidateSlot]) {
            // Slot is free
            m_targetSlot = candidateSlot;
            m_targetTravelTime = TimeForSlot(m_targetSlot, m_maxTimeFrontToBack);
            break;
        }
        
        if (attempt == 2) {
            // All slots occupied
            setError("StorageFullError: All 3 slots are occupied");
            return;
        }
    }
    
    // Publish telemetry
    updateTelemetry();
    
    std::cout << "StoreAppleCommand initialized - Color: " << m_color 
              << ", Slot: " << m_targetSlot 
              << ", Travel time: " << m_targetTravelTime.count() << "ms" << std::endl;
    
    m_state = State::ELEVATOR_POSITIONING;
}

void StoreAppleCommand::Execute() {
    updateTelemetry();
    
    switch (m_state) {
        case State::INITIALIZING:
            // Handled in Initialize()
            break;
            
        case State::ELEVATOR_POSITIONING:
            if (m_storageHeightMM > 0.0) {
                m_elevator.MoveTo(static_cast<float>(m_storageHeightMM));
                if (m_elevator.IsAtTarget()) {
                    m_state = State::ARM_JOINT_SAFE_CARRY;
                    m_stateTimer.Reset();
                }
            } else {
                // Skip elevator positioning
                m_state = State::ARM_JOINT_SAFE_CARRY;
                m_stateTimer.Reset();
            }
            break;
            
        case State::ARM_JOINT_SAFE_CARRY:
            // FIXED: Use correct methods from ArmSubsystem and GripperJointSubsystem
            m_arm.SetServoSpeed(1.0);
            m_arm.SetHomePosition();               // Move arm to HOME_ANGLE (storage position)
            
            m_joint.SetSpeedNormal();
            m_joint.SetGripperDownAngle();         // Move joint to DOWN_ANGLE (only angle for extender movement)
            
            if (isArmAtTarget(HOME_ANGLE) && isJointAtTarget(JOINT_DOWN_ANGLE)) {
                m_state = State::EXTENDER_RETRACTING;
                m_stateTimer.Reset();
                std::cout << "Arm/Joint positioned for storage - Ready to move extender" << std::endl;
            }
            break;
            
        case State::EXTENDER_RETRACTING:
            m_extender.MoveToBack();
            
            if (m_extender.IsAtBack()) {
                m_state = State::EXTENDER_POSITIONING;
                m_stateTimer.Reset();
                std::cout << "Starting extender positioning for " << m_targetTravelTime.count() << "ms" << std::endl;
            }
            break;
            
        case State::EXTENDER_POSITIONING:
            m_extender.MoveToFront();
            
            if (m_stateTimer.Get() >= m_targetTravelTime.count() / 1000.0 || m_extender.IsAtFront()) {
                m_extender.Stop();
                m_state = State::ARM_JOINT_DROP_POSE;
                m_stateTimer.Reset();
                std::cout << "Extender positioned at target slot " << m_targetSlot << std::endl;
            }
            break;
            
        case State::ARM_JOINT_DROP_POSE:
            // FIXED: Keep arm at HOME and joint at DOWN (already in correct positions)
            m_arm.SetServoSpeed(1.0);
            m_arm.SetHomePosition();               // Keep arm at HOME_ANGLE (storage angle)
            
            m_joint.SetSpeedNormal();
            m_joint.SetGripperDownAngle();         // Keep joint at DOWN_ANGLE (only position for dropping)
            
            if (isArmAtTarget(HOME_ANGLE) && isJointAtTarget(JOINT_DOWN_ANGLE)) {
                m_state = State::GRIPPER_OPENING;
                m_stateTimer.Reset();
                std::cout << "Ready to drop apple - Arm at HOME, Joint at DOWN" << std::endl;
            }
            break;
            
        case State::GRIPPER_OPENING:
            m_gripper.SetOpenGripper(); // Drop the apple!
            m_state = State::DROP_DWELL;
            m_stateTimer.Reset();
            std::cout << "Apple dropped into slot " << m_targetSlot << std::endl;
            break;
            
        case State::DROP_DWELL:
            if (m_stateTimer.Get() >= m_dropDwellMs.count() / 1000.0) {
                // Apple dropped successfully - update storage state
                StorageState& state = StorageState::getInstance();
                state.slots[m_targetSlot] = m_color;
                SaveStateToNT(state);
                
                std::cout << "Apple stored successfully in slot " << m_targetSlot << " with color " << m_color << std::endl;
                
                m_state = State::EXTENDER_RETRACTING_FINAL;
                m_stateTimer.Reset();
            }
            break;
            
        case State::EXTENDER_RETRACTING_FINAL:
            m_extender.MoveToBack();
            
            if (m_extender.IsAtBack()) {
                m_state = State::ARM_JOINT_SAFE_FINAL;
                m_stateTimer.Reset();
                std::cout << "Extender retracted safely" << std::endl;
            }
            break;
            
        case State::ARM_JOINT_SAFE_FINAL:
            // FIXED: Move to final positions - extender to FRONT and joint to DOWN
            m_arm.SetServoSpeed(1.0);
            m_arm.SetHomePosition();               // Keep arm at HOME_ANGLE (storage angle)
            
            m_joint.SetSpeedNormal();
            m_joint.SetGripperDownAngle();         // Keep joint at DOWN_ANGLE as requested
            
            m_gripper.SetClosedGripper();          // Close gripper for safety
            
            // Start moving extender to front position (final position as requested)
            m_extender.MoveToFront();
            
            if (isArmAtTarget(HOME_ANGLE) && isJointAtTarget(JOINT_DOWN_ANGLE) && m_extender.IsAtFront()) {
                m_state = State::COMPLETED;
                std::cout << "Storage operation completed - Extender at FRONT, Joint at DOWN" << std::endl;
            }
            break;
            
        case State::COMPLETED:
        case State::ERROR:
            // Terminal states
            break;
    }
}

bool StoreAppleCommand::IsFinished() {
    return m_state == State::COMPLETED || m_state == State::ERROR;
}

void StoreAppleCommand::End(bool interrupted) {
    std::cout << "=== StoreAppleCommand::End (interrupted=" << interrupted << ") ===" << std::endl;
    
    // Stop all motion for safety
    m_extender.Stop();
    m_elevator.Stop();
    
    if (interrupted) {
        auto nt = nt::NetworkTableInstance::GetDefault().GetTable("Storage");
        nt->PutString("LastError", "Command interrupted");
    }
    
    m_timer.Stop();
    m_stateTimer.Stop();
    
    if (m_state == State::COMPLETED) {
        std::cout << "StoreAppleCommand completed successfully" << std::endl;
        std::cout << "Final state: Extender at FRONT, Joint at DOWN_ANGLE" << std::endl;
    } else {
        std::cout << "StoreAppleCommand ended in state: " << static_cast<int>(m_state) << std::endl;
    }
}

void StoreAppleCommand::loadConfiguration() {
    auto nt = nt::NetworkTableInstance::GetDefault().GetTable("Storage");
    
    // Load thresholds
    m_config.t1 = nt->GetNumber("T1", 1.0);
    m_config.t2 = nt->GetNumber("T2", 2.0);
    
    // Load slot fractions
    m_config.slot0Frac = nt->GetNumber("Slot0Frac", 0.17);
    m_config.slot1Frac = nt->GetNumber("Slot1Frac", 0.50);
    m_config.slot2Frac = nt->GetNumber("Slot2Frac", 0.83);
    
    // Load other settings
    m_storageHeightMM = nt->GetNumber("HeightMM", 0.0);
    m_dropDwellMs = std::chrono::milliseconds(static_cast<long>(nt->GetNumber("DropDwellMs", 500.0)));
}

std::string StoreAppleCommand::readColorFromSmartDashboard() {
    std::string colorCode = frc::SmartDashboard::GetString("Camera/Apple/Color", "0");
    return mapColorCodeToString(colorCode);
}

std::string StoreAppleCommand::mapColorCodeToString(const std::string& colorCode) {
    if (colorCode == "1") return "red";
    if (colorCode == "2") return "yellow"; 
    if (colorCode == "3") return "green";
    
    throw std::runtime_error("Invalid color code '" + colorCode + "'. Expected 1=red, 2=yellow, 3=green");
}

void StoreAppleCommand::setError(const std::string& error) {
    std::cout << "ERROR: StoreAppleCommand - " << error << std::endl;
    
    auto nt = nt::NetworkTableInstance::GetDefault().GetTable("Storage");
    nt->PutString("LastError", error);
    
    m_state = State::ERROR;
}

void StoreAppleCommand::updateTelemetry() {
    auto nt = nt::NetworkTableInstance::GetDefault().GetTable("Storage");
    
    nt->PutNumber("ChosenSlot", m_targetSlot);
    nt->PutNumber("MaxTimeFrontToBack", m_maxTimeFrontToBack);
    nt->PutNumber("TargetMs", m_targetTravelTime.count());
    nt->PutString("CurrentColor", m_color);
    nt->PutNumber("CurrentState", static_cast<int>(m_state));
    nt->PutNumber("ElapsedTime", m_timer.Get());
    nt->PutNumber("StateElapsedTime", m_stateTimer.Get());
    
    // Also publish to SmartDashboard for easy viewing
    frc::SmartDashboard::PutNumber("Storage/ChosenSlot", m_targetSlot);
    frc::SmartDashboard::PutString("Storage/CurrentColor", m_color);
    frc::SmartDashboard::PutNumber("Storage/StateEnum", static_cast<int>(m_state));
}

bool StoreAppleCommand::isArmAtTarget(double targetAngle) const {
    // Assuming some tolerance for arm positioning
    constexpr double ARM_TOLERANCE = 2.0; // degrees
    double currentAngle = m_arm.GetCurrentAngle(); 
    return std::abs(currentAngle - targetAngle) < ARM_TOLERANCE;
}

bool StoreAppleCommand::isJointAtTarget(double targetAngle) const {
    // Assuming some tolerance for joint positioning  
    constexpr double JOINT_TOLERANCE = 2.0; // degrees
    double currentAngle = m_joint.GetCurrentAngle(); 
    return std::abs(currentAngle - targetAngle) < JOINT_TOLERANCE;
}

} // namespace storage