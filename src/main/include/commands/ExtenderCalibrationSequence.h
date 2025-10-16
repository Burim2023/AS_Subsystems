#pragma once

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include "subsystems/ExtenderSubsystem.h"
#include "commands/ExtendForDuration.h"
#include "commands/CalibrateExtender.h"  // Add this include

/**
 * ExtenderCalibrationSequence
 * ---------------------------
 * A complete calibration and positioning demonstration sequence:
 * 1. Calibrate the extender (get MaxTimeFrontToBack)
 * 2. Extend to front limit
 * 3. Retract in 25% increments (4 steps) back to starting position
 * 
 * This sequence demonstrates time-based positioning using the calibrated values.
 */
class ExtenderCalibrationSequence : public frc2::SequentialCommandGroup {
public:
    /**
     * Constructor for ExtenderCalibrationSequence
     * @param extender Pointer to the ExtenderSubsystem
     * @param pauseBetweenSteps Optional pause between each step (default: 1.0 seconds)
     */
    explicit ExtenderCalibrationSequence(ExtenderSubsystem* extender, double pauseBetweenSteps = 1.0);

private:
    ExtenderSubsystem* m_extender;
    double m_pauseTime;
};