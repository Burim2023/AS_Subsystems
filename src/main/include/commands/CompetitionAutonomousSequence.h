#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <memory>

class AMCU;
class SensorManager;  // ✅ Add forward declaration
#include "subsystems/elevator/ArmSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"
#include "subsystems/gripper/GripperJointSubsystem.h"
#include "subsystems/vision/CameraSubsystem.h"
#include "subsystems/elevator/ElevatorSubsystem.h"
#include "subsystems/elevator/ExtenderSubsystem.h"

#include "commands/DriveSmartPickupGround.h"
#include "commands/DriveForDuration.h"
#include "commands/GripperOperate.h"
#include "commands/DriveDistanceCommand.h"
#include "commands/SmartPickSequence.h"
#include "commands/SpeedDriveCommand.h"
#include "commands/CalibrateExtender.h"
#include "commands/ExtendForDuration.h"
#include "commands/MoveElevatorToPosition.h"
#include "commands/MoveArmToPosition.h"
#include "commands/MoveGripperJointToPosition.h"
#include "commands/Drive/DriveUntilWallCommand.h"

/**
 * CompetitionAutonomousSequence
 *
 * A ready-to-use autonomous sequence for competition that composes existing
 * commands. Sequence:
 *  - DriveSmartPickupGround (locate & pick an apple)
 *  - StoreAppleCommand (store picked apple into the given slot)
 *  - DriveForDuration forward to a scoring area
 *  - GripperOperate to open and release any held object
 *  - DriveForDuration backward to retreat
 */
class CompetitionAutonomousSequence : public frc2::SequentialCommandGroup {
public:
    /**
     * Constructor
     * @param amcu Drive subsystem pointer
     * @param sensorManager Sensor manager pointer  // ✅ Added
     * @param arm Arm subsystem pointer
     * @param gripper Gripper subsystem pointer
     * @param gripperJoint Gripper joint subsystem pointer
     * @param camera Camera subsystem pointer
     * @param elevator Elevator subsystem pointer
     * @param extender Extender subsystem pointer
     * @param storeSlot Slot index to store apple (0..2)
     */
    CompetitionAutonomousSequence(AMCU* amcu,
                                  SensorManager* sensorManager,  // ✅ Added
                                  ArmSubsystem* arm,
                                  GripperSubsystem* gripper,
                                  GripperJointSubsystem* gripperJoint,
                                  CameraSubsystem* camera,
                                  ElevatorSubsystem* elevator,
                                  ExtenderSubsystem* extender);

    // ✅ Fixed: Add parameter to SetAMCU
    void SetAMCU(AMCU* amcu);
    void SetSensorManager(SensorManager* sensorManager);  // ✅ Added

private:
    // ✅ Add member variables
    AMCU* m_amcu;
    SensorManager* m_sensorManager;  // ✅ Added
    ArmSubsystem* m_arm;
    GripperSubsystem* m_gripper;
    GripperJointSubsystem* m_gripperJoint;
    CameraSubsystem* m_camera;
    ElevatorSubsystem* m_elevator;
    ExtenderSubsystem* m_extender;

    void BuildSequence();  // ✅ Helper to build command sequence
    
    bool m_sequenceBuilt = false;  // ✅ Track if sequence was built
};
