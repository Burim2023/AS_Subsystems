#pragma once
#include <frc2/command/CommandHelper.h>
#include <frc2/command/FunctionalCommand.h>
#include <chrono>
#include <optional>

class AMCU;
class LineFollower;

class CobraLineFollowCommand
    : public frc2::CommandHelper<frc2::FunctionalCommand, CobraLineFollowCommand>
{
public:
  // Constructor requires both AMCU and LineFollower for proper scheduler requirements
  CobraLineFollowCommand(AMCU *amcu,
                         LineFollower *lf,
                         double kP = 40.0,
                         uint8_t fwd = 15,
                         double lostDebounce = 0.50);

private:
  // live params the lambdas will read
  AMCU *m_amcu = nullptr;
  LineFollower *m_lf = nullptr;
  double m_kP = 40.0;
  uint8_t m_fwd = 15;
  double m_lost = 0.50;
};
