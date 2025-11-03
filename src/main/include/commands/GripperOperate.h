#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/gripper/GripperJointSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"

/**
 * GripperOperate
 * A command that moves the gripper joint to a preset position (Down/Mid/Up)
 * and then opens or closes the gripper. Uses controlled servo speed via
 * the subsystem's Periodic() method.
 *
 * Usage examples:
 *  - GripperOperate(&m_gripperJoint, &m_gripper, GripperOperate::Position::UP, true)
 *  - GripperOperate(&m_gripperJoint, &m_gripper, GripperOperate::Position::MID, false, 1.0)
 */
class GripperOperate : public frc2::CommandHelper<frc2::CommandBase, GripperOperate> {
public:
    enum class Position { DOWN, MID, UP };

    /**
     * Constructor
     * @param joint Pointer to GripperJointSubsystem
     * @param gripper Pointer to GripperSubsystem  
     * @param pos Target position (DOWN, MID, UP)
     * @param openAfter Whether to open (true) or close (false) gripper after moving
     * @param speed Servo movement speed (0.1 to 10.0, default 2.0)
     */
    GripperOperate(GripperJointSubsystem* joint, GripperSubsystem* gripper, 
                   Position pos, bool openAfter = true, double speed = 2.0);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    GripperJointSubsystem* m_joint;
    GripperSubsystem* m_gripper;
    Position m_targetPos;
    bool m_openAfter;
    double m_servoSpeed;

    bool m_jointReached = false;
    bool m_gripperActuated = false;
};