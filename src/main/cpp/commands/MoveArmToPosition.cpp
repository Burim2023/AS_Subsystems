#include "commands/MoveArmToPosition.h"
#include <iostream>

MoveArmToPosition::MoveArmToPosition(ArmSubsystem* subsystem, double targetAngle)
    : m_arm(subsystem), m_targetAngle(targetAngle) {
    // Declare subsystem dependency - prevents conflicts with other commands
    AddRequirements({m_arm});
}

void MoveArmToPosition::Initialize() {
    // Start the movement to the target angle using existing method
    // We'll simulate SetCustomAngle by directly setting the target
    // This would need a proper method in ArmSubsystem, but for now we'll use a workaround
    if (m_targetAngle == HOME_ANGLE) {
        m_arm->SetServoSpeed(1.0);
        m_arm->SetHomePosition();
    } else if (m_targetAngle == PICK_APPLE_ANGLE) {
        m_arm->SetServoSpeed(1.0);
        m_arm->SetPickApplePosition();
    } else if (m_targetAngle == DROP_APPLE_ANGLE) {
        m_arm->SetServoSpeed(1.0);
        m_arm->SetDropApplePosition();
    }
    std::cout << "MoveArmToPosition: Starting movement to " << m_targetAngle << " degrees" << std::endl;
}

void MoveArmToPosition::Execute() {
    // The subsystem's Periodic() method handles the movement
    // This can be empty for "set-and-forget" actions
}

bool MoveArmToPosition::IsFinished() {
    // Command finishes when the arm stops moving
    return !m_arm->IsMoving();
}

void MoveArmToPosition::End(bool interrupted) {
    if (interrupted) {
        std::cout << "MoveArmToPosition: Command was interrupted" << std::endl;
    } else {
        std::cout << "MoveArmToPosition: Finished at " << m_arm->GetCurrentAngle() << " degrees" << std::endl;
    }
}