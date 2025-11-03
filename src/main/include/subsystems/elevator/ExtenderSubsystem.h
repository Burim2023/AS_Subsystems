#pragma once

#include <string>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"

enum class ExtenderState
{
    Error,
    Idle,
    AtFront,
    AtBack,
    MovingForward,
    MovingBackward,
    CalibratingAtBack    // New state for calibration
};

class ExtenderSubsystem : public frc2::SubsystemBase
{
public:
    ExtenderSubsystem();
    
    /**
     * Initialize the extender servo and limit switches
     */
    void Init();
    
    void ExtenderSubsystemCurrentState();
    std::string GetCurrentStateString();
    void Periodic();
    void SetMaxTimeFrontToBack(double time);
    double GetMaxTimeFrontToBack() const;
    void calibrate();
    
    // New methods for command integration
    void MoveToFront();      // Start extending to front limit
    void MoveToBack();       // Start retracting to back limit
    void Stop();             // Stop movement
    ExtenderState GetCurrentState() const;
    bool IsAtFront() const;
    bool IsAtBack() const;
    bool IsMoving() const;
    
    // Safety limit switch methods
    bool IsFrontLimitPressed() const;
    bool IsBackLimitPressed() const;
    bool IsExtending() const;
    bool IsRetracting() const;
    
    // Calibration methods
    bool IsCalibrated() const;
    void ResetCalibration();
    
    // State control methods for CalibrateExtender command
    void SetExtendState();
    void SetRetractState();
    void SetStopState();

private:
    ExtenderState currentState = ExtenderState::Idle;
    double MaxTimeFrontToBack = 0.0;
};
