#include "commands/MoveGripperToPosition.h"
#include <iostream>

MoveGripperToPosition::MoveGripperToPosition(GripperJointSubsystem* subsystem, double targetAngle)
    : m_gripperJoint(subsystem), m_targetAngle(targetAngle) {
    // Declare subsystem dependency - prevents conflicts with other commands
    AddRequirements({m_gripperJoint});
}

void MoveGripperToPosition::Initialize() {
    // Start the movement to the target angle using existing methods
    if (m_targetAngle == JOINT_UP_ANGLE) {
        m_gripperJoint->SetSpeedNormal();
        m_gripperJoint->SetGripperUpAngle();
    } else if (m_targetAngle == JOINT_MID_ANGLE) {
        m_gripperJoint->SetSpeedNormal();
        m_gripperJoint->SetGripperMidAngle();
    } else if (m_targetAngle == JOINT_DOWN_ANGLE) {
        m_gripperJoint->SetSpeedNormal();
        m_gripperJoint->SetGripperDownAngle();
    }
    std::cout << "MoveGripperToPosition: Starting movement to " << m_targetAngle << " degrees" << std::endl;
}

void MoveGripperToPosition::Execute() {
    // The subsystem's Periodic() method handles the movement
    // This can be empty for "set-and-forget" actions
}

bool MoveGripperToPosition::IsFinished() {
    // Command finishes when the gripper stops moving
    return !m_gripperJoint->IsMoving();
}

void MoveGripperToPosition::End(bool interrupted) {
    if (interrupted) {
        std::cout << "MoveGripperToPosition: Command was interrupted" << std::endl;
    } else {
        std::cout << "MoveGripperToPosition: Finished at " << m_gripperJoint->GetCurrentAngle() << " degrees" << std::endl;
    }
}