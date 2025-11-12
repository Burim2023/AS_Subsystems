#include "commands/CompetitionAutonomousSequence.h"
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include "commands/DriveDistanceCommand.h"
#include "Constants.h"
#include <iostream>

CompetitionAutonomousSequence::CompetitionAutonomousSequence(
    AMCU* amcu,
    ArmSubsystem* arm,
    GripperSubsystem* gripper,
    GripperJointSubsystem* gripperJoint,
    CameraSubsystem* camera,
    ElevatorSubsystem* elevator,
    ExtenderSubsystem* extender)
    : m_amcu(amcu),
      m_arm(arm),
      m_gripper(gripper),
      m_gripperJoint(gripperJoint),
      m_camera(camera),
      m_elevator(elevator),
      m_extender(extender),
      m_sequenceBuilt(false)
{
    // ✅ DON'T build sequence here if AMCU is null
    if (m_amcu) {
        BuildSequence();
        m_sequenceBuilt = true;
        std::cout << "CompetitionAutonomousSequence: Built with valid AMCU" << std::endl;
    } else {
        std::cout << "CompetitionAutonomousSequence: AMCU is null, sequence will be built later" << std::endl;
    }
}

void CompetitionAutonomousSequence::SetAMCU(AMCU* amcu) {
    m_amcu = amcu;
    std::cout << "CompetitionAutonomousSequence: AMCU updated to " << amcu << std::endl;
    
    // ✅ Build sequence now that AMCU is valid
    if (m_amcu && !m_sequenceBuilt) {
        BuildSequence();
        m_sequenceBuilt = true;
        std::cout << "CompetitionAutonomousSequence: Sequence built with AMCU" << std::endl;
    }
}

void CompetitionAutonomousSequence::BuildSequence() {
    if (!m_amcu) {
        std::cerr << "ERROR: Cannot build sequence - AMCU is still null!" << std::endl;
        return;
    }
    
    std::cout << "CompetitionAutonomousSequence: Building sequence with AMCU at " << m_amcu << std::endl;
    
    AddCommands(
        frc2::InstantCommand([]() {
            std::cout << "\n========================================" << std::endl;
            std::cout << ">>> COMPETITION AUTO SEQUENCE START <<<" << std::endl;
            std::cout << "========================================\n" << std::endl;
        }),

        // ✅ Now m_amcu is valid!
        //SpeedDriveCommand(m_amcu, 5, 15, 0, 0),
        DriveDistanceCommand(m_amcu, 0, 0, 1, 10),
        frc2::WaitCommand(2.0_s),

        std::move(CalibrateExtender(m_extender)),

        ExtendForDuration(m_extender, ExtendForDuration::Direction::RETRACT, 0.1, 2.0),
        
        frc2::WaitCommand(2.0_s),

        //MoveGripperJointToPosition(m_gripperJoint, JOINT_MID_ANGLE, true),

        SmartPickSequence(m_arm, m_gripper, m_gripperJoint, m_camera, m_elevator),

        frc2::WaitCommand(2.0_s),

        GripperOperate(m_gripperJoint, m_gripper, GripperOperate::Position::DOWN, false, 2.0),
        
        frc2::WaitCommand(2.0_s),

        GripperOperate(m_gripperJoint, m_gripper, GripperOperate::Position::DOWN, true, 2.0),

        frc2::WaitCommand(2.0_s),

        MoveElevatorToPosition(m_elevator, 150.0f, 1.0f),

        frc2::WaitCommand(2.0_s),

        GripperOperate(m_gripperJoint, m_gripper, GripperOperate::Position::DOWN, false, 2.0),

        frc2::WaitCommand(3.0_s),

        MoveElevatorToPosition(m_elevator, 195.0f, 2.0f),

        GripperOperate(m_gripperJoint, m_gripper, GripperOperate::Position::MID, false, 2.0),
        
        frc2::WaitCommand(2.0_s),

        frc2::WaitCommand(2.0_s),

        MoveArmToPosition(m_arm, PICK_APPLE_ANGLE),

        frc2::InstantCommand([]() {
            std::cout << "\n========================================" << std::endl;
            std::cout << ">>> COMPETITION AUTO SEQUENCE COMPLETE <<<" << std::endl;
            std::cout << "========================================\n" << std::endl;
        })
    );
}
