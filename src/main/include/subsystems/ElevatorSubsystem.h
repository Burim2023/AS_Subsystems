#pragma once

#include <frc2/command/SubsystemBase.h>
#include "subsystems/elevator.h"
#include "AMCU.h"

/**
 * Elevator subsystem wrapper for command-based framework
 */
class ElevatorSubsystem : public frc2::SubsystemBase {
public:
    ElevatorSubsystem();
    ~ElevatorSubsystem();
    
    /**
     * Initialize the elevator with AMCU instance
     * @param amcu Pointer to AMCU instance
     */
    void Init(AMCU* amcu);
    
    /**
     * Move elevator to specified position
     * @param position Target position in mm
     */
    void MoveTo(float position);
    
    /**
     * Start calibration sequence
     */
    void Calibrate();
    
    /**
     * Get current elevator position
     * @return Current position in mm
     */
    float GetCurrentPosition();
    
    /**
     * Check if elevator is at target position
     * @param tolerance Acceptable error in mm (default 1.0mm)
     * @return True if at target position
     */
    bool IsAtTarget(float tolerance = 1.0f);
    
    /**
     * Stop elevator movement
     */
    void Stop();
    
    /**
     * Check if elevator is initialized
     * @return True if initialized
     */
    bool IsInitialized() const { return m_isInitialized; }
    
    /**
     * Periodic method called by command scheduler
     */
    void Periodic() override;
    
private:
    bool m_isInitialized;
    float m_lastTargetPosition;
    AMCU* m_amcu;
};