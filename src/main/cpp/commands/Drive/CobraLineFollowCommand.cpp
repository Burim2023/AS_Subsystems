#include "commands/Drive/CobraLineFollowCommand.h"
#include "subsystems/sensor/LineFollower.h"
#include "subsystems/amcu/AMCU.h"

#include <algorithm>
#include <cmath>
#include <iostream>

using Clock = std::chrono::steady_clock;

CobraLineFollowCommand::CobraLineFollowCommand(
    LineFollower* lf, double kP, uint8_t fwd, double lostDebounce)
  : m_lf(lf), m_kP(kP), m_fwd(fwd), m_lost(lostDebounce),
    CommandHelper(
      // -------- Initialize --------
      [this]() {
        std::cout << "CobraLineFollow: start\n";
        if (!m_amcu) std::cout << "CobraLineFollow: AMCU is NULL!\n";
        if (!m_lf)   std::cout << "CobraLineFollow: LineFollower is NULL!\n";
        if (m_amcu) m_amcu->stop();
        if (m_lf)   m_lf->update();
      },
      // -------- Execute (~50 Hz) --------
      [this]() {
        static auto last = Clock::now();
        auto now = Clock::now();
        if (now - last < std::chrono::milliseconds(20)) return;
        last = now;

        if (!m_amcu || !m_lf) return;

        m_lf->update();

        // occasional prints (about 2/s)
        static int tick = 0;
        if ((++tick % 25) == 0) {
          auto v = m_lf->getVoltages();
          std::cout << "[LF] V=[" << v[0] << "," << v[1] << "," << v[2] << "," << v[3]
                    << "] det=" << (m_lf->isLineDetected() ? "1" : "0")
                    << " err=" << m_lf->getPositionError() << "\n";
        }

        static std::optional<Clock::time_point> lostSince;
        if (!m_lf->isLineDetected()) {
          if (!lostSince) lostSince = now;
        } else {
          lostSince.reset();
        }

        const bool lost = lostSince &&
          std::chrono::duration<double>(now - *lostSince).count() >= m_lost;

        if (lost) { m_amcu->stop(); return; }

        // P-control on continuous error [-1..+1]
        const double err = m_lf->getPositionError();
        double rotCmd = std::clamp(m_kP * err, -30.0, 30.0); // deg/s clamp
        const uint8_t rot_u8 =
            static_cast<uint8_t>(static_cast<int8_t>(std::lround(rotCmd)));

        m_amcu->speedDrive(static_cast<uint8_t>(m_fwd), 0, rot_u8);

        if ((tick % 25) == 0) {
          std::cout << "[LF] drive f=" << int(m_fwd) << " w=" << int(rot_u8) << "\n";
        }
      },
      // -------- End --------
      [this](bool interrupted) {
        if (m_amcu) m_amcu->stop();
        std::cout << "CobraLineFollow: end (interrupted="
                  << (interrupted ? "true" : "false") << ")\n";
      },
      // -------- IsFinished --------
      [this]() -> bool { return false; }
    )
{}
