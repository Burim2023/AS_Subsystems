#include "subsystems/sensor/LineFollower.h"
#include <frc/smartdashboard/SmartDashboard.h>

// Constructor
LineFollower::LineFollower(int ch0, int ch1, int ch2, int ch3, float vRef)
    : m_cobra(vRef),
      m_ch{ch0, ch1, ch2, ch3}
{
}

void LineFollower::update()
{
  std::lock_guard<std::mutex> lock(m_mutex);

  // Read raw volts and compute darkness (0=white, 1=black)
  for (int i = 0; i < 4; ++i)
  {
    const double v = m_volt[i] = static_cast<double>(m_cobra.GetVoltage(m_ch[i]));

    // On Cobra, WHITE < BLACK. If captured out of order, correct it:
    double white = m_white[i];
    double black = m_black[i];
    if (black < white)
      std::swap(black, white);

    const double denom = std::max(1e-6, black - white); // (black - white) > 0
    const double t = (v - white) / denom;               // 0 at white, 1 at black (unclamped)
    m_dark[i] = clamp01(t);
  }

  // Average “signal strength”
  m_signal = (m_dark[0] + m_dark[1] + m_dark[2] + m_dark[3]) / 4.0;

  // Weighted centroid -> [-1..+1] (left..right), 0=centered
  static constexpr double w[4] = {-3.0, -1.0, +1.0, +3.0};
  double num = 0.0, den = 0.0;
  for (int i = 0; i < 4; ++i)
  {
    num += w[i] * m_dark[i];
    den += m_dark[i];
  }
  m_posErr = (den < 1e-6) ? 0.0 : std::clamp((num / den) / 3.0, -1.0, 1.0);
}

void LineFollower::CaptureWhite()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  for (int i = 0; i < 4; ++i)
  {
    m_white[i] = static_cast<double>(m_cobra.GetVoltage(m_ch[i]));
  }
}

void LineFollower::CaptureBlack()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  for (int i = 0; i < 4; ++i)
  {
    m_black[i] = static_cast<double>(m_cobra.GetVoltage(m_ch[i]));
  }
}

// ---- Persistence to NetworkTables (optional) ----
void LineFollower::SaveCalibrationNT(const std::string &p)
{
  for (int i = 0; i < 4; ++i)
  {
    frc::SmartDashboard::PutNumber(p + "white" + std::to_string(i), m_white[i]);
    frc::SmartDashboard::PutNumber(p + "black" + std::to_string(i), m_black[i]);
    frc::SmartDashboard::SetPersistent(p + "white" + std::to_string(i));
    frc::SmartDashboard::SetPersistent(p + "black" + std::to_string(i));
  }
  frc::SmartDashboard::PutNumber(p + "minSignal", m_minSignal);
  frc::SmartDashboard::SetPersistent(p + "minSignal");
}

void LineFollower::LoadCalibrationNT(const std::string &p)
{
  for (int i = 0; i < 4; ++i)
  {
    m_white[i] = frc::SmartDashboard::GetNumber(p + "white" + std::to_string(i), m_white[i]);
    m_black[i] = frc::SmartDashboard::GetNumber(p + "black" + std::to_string(i), m_black[i]);
  }
  m_minSignal = frc::SmartDashboard::GetNumber(p + "minSignal", m_minSignal);
}

// ---- Shuffleboard ----
void LineFollower::InitShuffleboard(const std::string &tabName)
{
  if (m_sbInit)
    return;
  m_tabName = tabName;

  auto &tab = frc::Shuffleboard::GetTab(m_tabName);

  // Raw voltages
  m_entVolt[0] = tab.Add("V0", 0.0).WithPosition(0, 0).GetEntry();
  m_entVolt[1] = tab.Add("V1", 0.0).WithPosition(1, 0).GetEntry();
  m_entVolt[2] = tab.Add("V2", 0.0).WithPosition(2, 0).GetEntry();
  m_entVolt[3] = tab.Add("V3", 0.0).WithPosition(3, 0).GetEntry();

  // Darkness per channel
  m_entDark[0] = tab.Add("Dark0", 0.0).WithPosition(0, 1).GetEntry();
  m_entDark[1] = tab.Add("Dark1", 0.0).WithPosition(1, 1).GetEntry();
  m_entDark[2] = tab.Add("Dark2", 0.0).WithPosition(2, 1).GetEntry();
  m_entDark[3] = tab.Add("Dark3", 0.0).WithPosition(3, 1).GetEntry();

  // Aggregates
  m_entSignal = tab.Add("Signal", 0.0).WithPosition(0, 2).GetEntry();
  m_entErr = tab.Add("PosErr", 0.0).WithPosition(1, 2).GetEntry();
  m_entDetected = tab.Add("Detected", false).WithPosition(2, 2).GetEntry();

  m_sbInit = true;
}

void LineFollower::UpdateShuffleboard(int rateDiv)
{
  if (!m_sbInit)
    InitShuffleboard(m_tabName);

  static int tick = 0;
  if (++tick % std::max(1, rateDiv) != 0)
    return;

  std::lock_guard<std::mutex> lock(m_mutex);

  for (int i = 0; i < 4; ++i)
  {
    m_entVolt[i].SetDouble(m_volt[i]);
    m_entDark[i].SetDouble(m_dark[i]);
  }
  m_entSignal.SetDouble(m_signal);
  m_entErr.SetDouble(m_posErr);
  m_entDetected.SetBoolean(m_signal >= m_minSignal);
}
