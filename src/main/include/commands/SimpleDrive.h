#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

// Forward declaration to avoid including AMCU.h in header
class AMCU;

/**
 * Simple drive command that only uses AMCU driving methods.
 * This command runs continuously and allows manual control of the drivetrain.
 * It's designed to test basic AMCU functionality without any complexity.
 */
class SimpleDrive : public frc2::CommandHelper<frc2::CommandBase, SimpleDrive> {
public:
    /**
     * Constructor
     * @param amcu Pointer to the AMCU drivetrain system
     * @param forward Forward/backward speed (-1.0 to 1.0)
     * @param strafe Left/right speed (-1.0 to 1.0) 
     * @param rotate Rotation speed (-1.0 to 1.0)
     */
    SimpleDrive(AMCU* amcu, double forward, double strafe, double rotate);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    AMCU* m_amcu;
    double m_forward;
    double m_strafe;  
    double m_rotate;
};