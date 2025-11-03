#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include <string>  // Add this include
#include "subsystems/elevator/ExtenderSubsystem.h"

/**
 * Command to control the extender using timing based on calibrated max travel time.
 * Uses MaxTimeFrontToBack as reference for positioning.
 */
class ExtendForDuration : public frc2::CommandHelper<frc2::CommandBase, ExtendForDuration> {
public:
    enum class Direction {
        EXTEND,     // Move to front (forward)
        RETRACT     // Move to back (backward)
    };

    /**
     * Constructor for time-based positioning
     * @param subsystem Pointer to the ExtenderSubsystem
     * @param direction Direction to move (EXTEND or RETRACT)
     * @param timeRatio Ratio of max time (0.0 to 1.0) - 0.5 = halfway, 1.0 = full travel
     * @param safetyTimeoutMultiplier Safety timeout as multiple of calculated time (default 2.0)
     */
    ExtendForDuration(ExtenderSubsystem* subsystem, Direction direction, double timeRatio, double safetyTimeoutMultiplier = 2.0);
    
    /**
     * Constructor for full travel to limit switches
     * @param subsystem Pointer to the ExtenderSubsystem  
     * @param direction Direction to move (EXTEND or RETRACT)
     */
    ExtendForDuration(ExtenderSubsystem* subsystem, Direction direction);
    
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    ExtenderSubsystem* m_extender;
    Direction m_direction;
    double m_targetTime;        
    double m_safetyTimeout;     
    bool m_useTimeBased;        
    frc::Timer m_timer;
    
    // Constructor parameters stored for later use:
    double m_timeRatio;         // Store the time ratio for later calculation
    double m_timeoutMultiplier; // Store the timeout multiplier
    
    // Helper method - ADD THIS DECLARATION:
    std::string GetDirectionName() const;
};