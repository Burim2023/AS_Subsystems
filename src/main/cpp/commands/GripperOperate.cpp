#include "commands/GripperOperate.h"
#include <iostream>

GripperOperate::GripperOperate(GripperJointSubsystem* joint, GripperSubsystem* gripper, 
                               Position pos, bool openAfter, double speed)
    : m_joint(joint), m_gripper(gripper), m_targetPos(pos), m_openAfter(openAfter), m_servoSpeed(speed) {
    // Require both subsystems using initializer list syntax
    // This ensures their Periodic() methods are called automatically
    AddRequirements({m_joint, m_gripper});
}

void GripperOperate::Initialize() {
    m_jointReached = false;
    m_gripperActuated = false;

    // Set the desired speed for joint movement
    m_joint->SetServoSpeed(m_servoSpeed);
    
    // Set the target joint position
    switch(m_targetPos) {
        case Position::DOWN:
            m_joint->SetSpeedNormal();
            m_joint->SetGripperDownAngle();
            std::cout << "GripperOperate: moving joint to DOWN position" << std::endl;
            break;
        case Position::MID:
            m_joint->SetSpeedNormal();
            m_joint->SetGripperMidAngle();
            std::cout << "GripperOperate: moving joint to MID position" << std::endl;
            break;
        case Position::UP:
            m_joint->SetSpeedNormal();
            m_joint->SetGripperUpAngle();
            std::cout << "GripperOperate: moving joint to UP position" << std::endl;
            break;
    }
}

void GripperOperate::Execute() {
    // The GripperJointSubsystem::Periodic() method is automatically called by the 
    // command scheduler since we added it to requirements. This handles the gradual
    // servo movement with controlled speed.
    
    // Check if joint has reached its target
    if (!m_jointReached) {
        if (!m_joint->IsMoving()) {
            m_jointReached = true;
            std::cout << "GripperOperate: joint reached target at angle " 
                      << m_joint->GetCurrentAngle() << std::endl;
        }
    }

    // Once joint has reached target, actuate gripper if not already done
    if (m_jointReached && !m_gripperActuated) {
        if (m_openAfter) {
            m_gripper->SetOpenGripper();
            std::cout << "GripperOperate: opening gripper" << std::endl;
        } else {
            m_gripper->SetClosedGripper();
            std::cout << "GripperOperate: closing gripper" << std::endl;
        }
        m_gripperActuated = true;
    }
}

void GripperOperate::End(bool interrupted) {
    if (interrupted) {
        std::cout << "GripperOperate: interrupted" << std::endl;
    } else {
        std::cout << "GripperOperate: completed" << std::endl;
    }
}

bool GripperOperate::IsFinished() {
    // Finish when joint reached AND gripper actuated
    return m_jointReached && m_gripperActuated;
}
