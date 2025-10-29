#pragma once
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <frc/Timer.h>
#include "AMCU.h"

class SpeedDriveCommand : public frc2::CommandHelper<frc2::CommandBase, SpeedDriveCommand> {
public:
    // timeoutSeconds = 0 => run until interrupted
    SpeedDriveCommand(AMCU* amcu, double timeoutSeconds, uint8_t forward, uint8_t strafe = 0, uint8_t rot = 0);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    AMCU* m_amcu;
    double m_timeout;
    uint8_t m_fwd, m_strafe, m_rot;  // Change to uint8_t
    frc::Timer m_timer;
    frc::Timer m_keepalive;
    static constexpr double kKeepaliveInterval = 0.25; // seconds (adjust as needed)
};