#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/amcu/AMCU.h"

class DriveDistanceCommand : public frc2::CommandHelper<frc2::CommandBase, DriveDistanceCommand> {
public:
    // ✅ Accept double (meters) for user-friendly interface
    DriveDistanceCommand(AMCU* amcu, 
                        double xMeter,
                        double yMeter,
                        double omega_degree,
                        double timeoutSeconds);
    
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    AMCU* m_amcu;
    double m_xMeter;
    double m_yMeter;
    double m_omega_degree;
    double m_timeout;
    frc::Timer m_timer;
};