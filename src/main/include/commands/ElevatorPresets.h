#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/elevator/ElevatorSubsystem.h"

/**
 * Command to move elevator to predefined preset positions
 */
class ElevatorPresets : public frc2::CommandHelper<frc2::CommandBase, ElevatorPresets> {
public:
    enum class Position {
        GROUND = 10,      // 10mm - ground level
        LOW = 45,        // 30mm - low position
        MEDIUM = 60,     // 60mm - medium position
        HIGH = 170,       //200mm - high position
        MAX = 200        // 200mm - maximum safe height
    };

    /**
     * Constructor
     * @param elevator Pointer to elevator subsystem
     * @param position Target preset position
     */
    ElevatorPresets(ElevatorSubsystem* elevator, Position position);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    ElevatorSubsystem* m_elevator;
    Position m_targetPosition;
    float GetPositionValue(Position pos);
    std::string GetPositionName(Position pos);
};