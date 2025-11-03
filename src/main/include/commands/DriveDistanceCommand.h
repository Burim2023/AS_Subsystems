#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/amcu/AMCU.h"

class DriveDistanceCommand : public frc2::CommandHelper<frc2::CommandBase, DriveDistanceCommand> {
public:
    // timeoutSeconds: estimated time to cover the distance (adjust based on speed)
    DriveDistanceCommand(AMCU* amcu, uint8_t xMeter, uint8_t yMeter, uint16_t omega_degree, double timeoutSeconds);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    AMCU* m_amcu;
    uint8_t m_xMeter;
    uint8_t m_yMeter;
    uint16_t m_omega_degree;
    double m_timeout;
    frc::Timer m_timer;
};