#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "storage/AppleStorage.h"
#include "subsystems/elevator/ExtenderSubsystem.h"
#include "subsystems/elevator/ElevatorSubsystem.h"
#include "subsystems/elevator/ArmSubsystem.h"
#include "subsystems/gripper/GripperJointSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"
#include <string>
#include <chrono>
#include <array>
#include <optional>

namespace storage {

/**
 * @brief Storage state singleton for slot→color persistence
 */
struct StorageState {
    std::array<std::optional<std::string>, 3> slots;
    
    static StorageState& getInstance() {
        static StorageState instance;
        return instance;
    }
};

/**
 * @brief Command to store an apple in the 3-slot storage system
 * 
 * This command orchestrates the full sequence of:
 * 1. Moving elevator to storage height
 * 2. Positioning arm/joint for transport
 * 3. Extending to target slot using time-based positioning
 * 4. Dropping apple and retracting safely
 */
class StoreAppleCommand : public frc2::CommandHelper<frc2::CommandBase, StoreAppleCommand> {
public:
    /**
     * @brief State machine phases
     */
    enum class State {
        INITIALIZING,
        ELEVATOR_POSITIONING,
        ARM_JOINT_SAFE_CARRY,
        EXTENDER_RETRACTING,
        EXTENDER_POSITIONING,
        ARM_JOINT_DROP_POSE,
        GRIPPER_OPENING,
        DROP_DWELL,
        EXTENDER_RETRACTING_FINAL,
        ARM_JOINT_SAFE_FINAL,
        COMPLETED,
        ERROR
    };

    /**
     * @brief Constructor with explicit color and maxtime
     * @param ext Extender subsystem reference
     * @param elev Elevator subsystem reference
     * @param arm Arm subsystem reference
     * @param joint Gripper joint subsystem reference
     * @param gripper Gripper subsystem reference
     * @param color Explicit apple color
     * @param pickMaxTime Explicit maxtime for slot selection
     */
    StoreAppleCommand(ExtenderSubsystem& ext, ElevatorSubsystem& elev, ArmSubsystem& arm,
                     GripperJointSubsystem& joint, GripperSubsystem& gripper,
                     const std::string& color, double pickMaxTime);

    /**
     * @brief Constructor with automatic color/maxtime detection
     * @param ext Extender subsystem reference
     * @param elev Elevator subsystem reference  
     * @param arm Arm subsystem reference
     * @param joint Gripper joint subsystem reference
     * @param gripper Gripper subsystem reference
     */
    StoreAppleCommand(ExtenderSubsystem& ext, ElevatorSubsystem& elev, ArmSubsystem& arm,
                     GripperJointSubsystem& joint, GripperSubsystem& gripper);

    /**
     * @brief Map maxtime to slot using thresholds
     * @param maxtime Time value for mapping
     * @param t1 First threshold
     * @param t2 Second threshold
     * @return Slot number (0, 1, or 2)
     */
    static int SlotFromMaxTime(double maxtime, double t1, double t2);

    /**
     * @brief Calculate travel time for slot positioning
     * @param slot Target slot (0, 1, or 2)
     * @param maxTimeFrontToBack Calibrated max travel time
     * @return Travel time from back limit to slot center
     */
    static std::chrono::milliseconds TimeForSlot(int slot, double maxTimeFrontToBack);

    /**
     * @brief Save storage state to NetworkTables
     * @param state Storage state to save
     */
    static void SaveStateToNT(const StorageState& state);

    /**
     * @brief Load storage state from NetworkTables
     * @param state Storage state to populate
     */
    static void LoadStateFromNT(StorageState& state);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    // Subsystem references
    ExtenderSubsystem& m_extender;
    ElevatorSubsystem& m_elevator;
    ArmSubsystem& m_arm;
    GripperJointSubsystem& m_joint;
    GripperSubsystem& m_gripper;

    // Command parameters
    std::string m_color;
    double m_maxTime;
    bool m_autoDetect;

    // State machine
    State m_state;
    frc::Timer m_timer;
    frc::Timer m_stateTimer;

    // Computed values
    int m_targetSlot;
    std::chrono::milliseconds m_targetTravelTime;
    double m_maxTimeFrontToBack;
    storage::StorageConfig m_config;

    // Configuration from NetworkTables
    double m_storageHeightMM;
    std::chrono::milliseconds m_dropDwellMs;

    // Helper methods
    void loadConfiguration();
    std::string readColorFromSmartDashboard();
    std::string mapColorCodeToString(const std::string& colorCode);
    void setError(const std::string& error);
    void updateTelemetry();
    bool isArmAtTarget(double targetAngle) const;
    bool isJointAtTarget(double targetAngle) const;
};

} // namespace storage