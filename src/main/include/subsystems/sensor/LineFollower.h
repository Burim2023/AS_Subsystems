#pragma once

#include <array>
#include <string>
#include <algorithm>
#include <cstdint>

#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "studica/Cobra.h"  // studica::Cobra

/**
 * LineFollower built on the Studica Cobra 4-channel analog sensor.
 * NOTE: On Cobra, BLACK returns a HIGHER voltage than WHITE.
 * Darkness in [0..1] is computed as (v - white) / (black - white), clamped.
 */
class LineFollower {
public:
  // channels left→right (0..3 on Cobra board). vRef is Cobra ADC reference (usually 5.0f).
  LineFollower(int ch0, int ch1, int ch2, int ch3, float vRef = 5.0f);

  // call every loop to refresh readings
  void update();

  // ---- Calibration / tuning ----------------------------------------------

  // Set uniform expected voltages.  PASS WHITE first, then BLACK.
  void setMinMax(double whiteV, double blackV) {
    m_white.fill(whiteV);
    m_black.fill(blackV);
  }

  // Per-channel expected voltages.  PASS WHITE array, then BLACK array.
  void setPerChannelMinMax(const std::array<double,4>& whiteV,
                           const std::array<double,4>& blackV) {
    m_white = whiteV;
    m_black = blackV;
  }

  // Threshold for "Detected" (based on average darkness)
  void setMinSignal(double s) { m_minSignal = std::clamp(s, 0.0, 1.0); }

  // One-touch calibration (call while aiming at the target):
  void CaptureWhite();  // sets per-channel WHITE (floor) voltages
  void CaptureBlack();  // sets per-channel BLACK (line) voltages

  // Optional: persist calibration across reboots via NetworkTables
  void SaveCalibrationNT(const std::string& keyPrefix = "LF/");
  void LoadCalibrationNT(const std::string& keyPrefix = "LF/");

  // ---- Telemetry ----------------------------------------------------------

  void InitShuffleboard(const std::string& tabName = "Line");
  void UpdateShuffleboard(int rateDiv = 10); // push every Nth call

  // ---- Accessors ----------------------------------------------------------

  std::array<double,4> getVoltages() const { return m_volt; }   // raw volts
  std::array<double,4> getDarkness() const { return m_dark; }   // 0=white,1=black
  double getPositionError() const { return m_posErr; }          // [-1..+1]
  bool   isLineDetected() const { return m_signal >= m_minSignal; }
  double getSignal() const { return m_signal; }                 // avg darkness

private:
  static inline double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }

  // Hardware
  studica::Cobra m_cobra;
  std::array<int,4> m_ch{};

  // Expected range (WHITE < BLACK) — initial guesses; you will calibrate.
  std::array<double,4> m_white{{0.65, 0.65, 0.65, 0.65}}; // floor
  std::array<double,4> m_black{{1.20, 1.20, 1.20, 1.20}}; // tape

  // Last readings
  std::array<double,4> m_volt{{0,0,0,0}};
  std::array<double,4> m_dark{{0,0,0,0}};
  double m_signal   = 0.0;   // avg darkness [0..1]
  double m_posErr   = 0.0;   // lateral error [-1..+1]
  double m_minSignal= 0.80;  // default detection threshold (tune 0.75..0.90)

  // Shuffleboard
  bool m_sbInit = false;
  std::string m_tabName = "Line";
  std::array<nt::NetworkTableEntry,4> m_entVolt{};
  std::array<nt::NetworkTableEntry,4> m_entDark{};
  nt::NetworkTableEntry m_entSignal;
  nt::NetworkTableEntry m_entErr;
  nt::NetworkTableEntry m_entDetected;
};
