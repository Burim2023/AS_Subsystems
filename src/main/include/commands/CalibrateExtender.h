#pragma once

#include <frc2/command/CommandBase.h>
#include <frc/Timer.h>
#include <memory>
#include "subsystems/elevator/ExtenderSubsystem.h"
#include <frc2/command/WaitCommand.h>

class CalibrateExtender : public frc2::CommandBase {
public:
    explicit CalibrateExtender(ExtenderSubsystem* extender);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;
    
    // Add this required method for WPILib 2020:
    std::unique_ptr<frc2::Command> TransferOwnership() && override {
        return std::make_unique<CalibrateExtender>(std::move(*this));
    }

private:
    ExtenderSubsystem* m_extender;
    frc::Timer m_timer;
    
    enum class CalibrationState {
        INITIAL,           // Starting state
        MOVING_TO_BACK,    // Ensuring we start at back limit
        AT_BACK_LIMIT,     // Confirmed at back limit
        TIMING_TO_FRONT,   // Moving front and timing the travel
        CALIBRATION_DONE   // Calibration complete
    };
    
    CalibrationState m_state;
    double m_startTime;
    bool m_calibrationComplete;
};