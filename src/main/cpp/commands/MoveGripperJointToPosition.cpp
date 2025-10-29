#include "commands/MoveGripperJointToPosition.h"
#include "Constants.h"
#include <iostream>

MoveGripperJointToPosition::MoveGripperJointToPosition(GripperJointSubsystem* subsystem, double targetAngle, bool holdPosition)
    : m_gripperJoint(subsystem), m_targetAngle(targetAngle), m_holdPosition(holdPosition) {
    // Declare subsystem dependency - prevents conflicts with other commands
    AddRequirements({m_gripperJoint});
}

void MoveGripperJointToPosition::Initialize() {
    // Start the movement to the target angle using existing methods
    if (m_targetAngle == JOINT_UP_ANGLE) {
        m_gripperJoint->SetSpeedNormal();
        m_gripperJoint->SetGripperUpAngle();
    } else if (m_targetAngle == JOINT_MID_ANGLE) {
        m_gripperJoint->SetSpeedNormal();
        m_gripperJoint->SetGripperMidAngle();
    } else if (m_targetAngle == JOINT_CAM_ANGLE) {
        m_gripperJoint->SetSpeedNormal();
        m_gripperJoint->SetGripperCamAngle();
    } else if (m_targetAngle == JOINT_DOWN_ANGLE) {
        m_gripperJoint->SetSpeedNormal();
        m_gripperJoint->SetGripperDownAngle();
    }
    std::cout << "MoveGripperToPosition: Starting movement to " << m_targetAngle 
              << " degrees" << (m_holdPosition ? " (holding)" : "") << std::endl;
}

void MoveGripperJointToPosition::Execute() {
    if (m_holdPosition) {
        // Keep commanding the position every cycle to hold it
        if (m_targetAngle == JOINT_UP_ANGLE) {
            m_gripperJoint->SetGripperUpAngle();
        } else if (m_targetAngle == JOINT_MID_ANGLE) {
            m_gripperJoint->SetGripperMidAngle();
        } else if (m_targetAngle == JOINT_CAM_ANGLE) {
            m_gripperJoint->SetGripperCamAngle();
        } else if (m_targetAngle == JOINT_DOWN_ANGLE) {
            m_gripperJoint->SetGripperDownAngle();
        }
    }
    // If not holding, the subsystem's Periodic() method handles the movement
}

bool MoveGripperJointToPosition::IsFinished() {
    if (m_holdPosition) {
        // If holding, never finish (must be interrupted)
        return false;
    } else {
        // Normal behavior - finish when gripper stops moving
        return !m_gripperJoint->IsMoving();
    }
}

void MoveGripperJointToPosition::End(bool interrupted) {
    if (interrupted) {
        std::cout << "MoveGripperToPosition: Command was interrupted" << std::endl;
    } else {
        std::cout << "MoveGripperToPosition: Finished at " << m_gripperJoint->GetCurrentAngle() << " degrees" << std::endl;
    }
}