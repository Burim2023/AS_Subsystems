#pragma once

#include <array>
#include <string>
#include <algorithm>
#include <cstdint>
#include <mutex>
#include <fstream>  // ✅ Add for file I/O

#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/json.h>  // ✅ Add for JSON support

#include "studica/Cobra.h" 

class LineFollower
{
public:
  LineFollower(int ch0, int ch1, int ch2, int ch3, float vRef = 5.0f);
  
  void update();
  
  void setMinMax(double whiteV, double blackV)
  {
    m_white.fill(whiteV);
    m_black.fill(blackV);
  }
  
  void setPerChannelMinMax(const std::array<double, 4> &whiteV,
                           const std::array<double, 4> &blackV)
  {
    m_white = whiteV;
    m_black = blackV;
  }

  void setMinSignal(double s) { m_minSignal = std::clamp(s, 0.0, 1.0); }

  // One-touch calibration
  void CaptureWhite();
  void CaptureBlack();

  // ✅ NEW: File-based persistence (recommended)
  bool SaveCalibrationToFile(const std::string &filePath = "/home/lvuser/linefollower_cal.json");
  bool LoadCalibrationFromFile(const std::string &filePath = "/home/lvuser/linefollower_cal.json");

  // NetworkTables persistence (legacy - optional)
  void SaveCalibrationNT(const std::string &keyPrefix = "LF/");
  void LoadCalibrationNT(const std::string &keyPrefix = "LF/");

  void InitShuffleboard(const std::string &tabName = "Line");
  void UpdateShuffleboard(int rateDiv = 10);

  std::array<double, 4> getVoltages() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_volt;
  }

  std::array<double, 4> getDarkness() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_dark;
  }

  double getPositionError() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_posErr;
  }

  bool isLineDetected() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_signal >= m_minSignal;
  }

  double getSignal() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_signal;
  }

private:
  static inline double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }

  studica::Cobra m_cobra;
  std::array<int, 4> m_ch{};

  std::array<double, 4> m_white{{0.65, 0.65, 0.65, 0.65}};
  std::array<double, 4> m_black{{1.20, 1.20, 1.20, 1.20}};

  std::array<double, 4> m_volt{{0, 0, 0, 0}};
  std::array<double, 4> m_dark{{0, 0, 0, 0}};
  double m_signal = 0.0;
  double m_posErr = 0.0;
  double m_minSignal = 0.80;

  bool m_sbInit = false;
  std::string m_tabName = "Line";
  std::array<nt::NetworkTableEntry, 4> m_entVolt{};
  std::array<nt::NetworkTableEntry, 4> m_entDark{};
  nt::NetworkTableEntry m_entSignal;
  nt::NetworkTableEntry m_entErr;
  nt::NetworkTableEntry m_entDetected;
  
  mutable std::mutex m_mutex;
};
