#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTableEntry.h>

#include <thread>
#include <atomic>
#include <string>
#include <mutex>

class CameraSubsystem : public frc2::SubsystemBase {
public:
  struct Settings {
    int deviceIndex = 0;
    int width = 320;
    int height = 240;
    int fps = 30;
    bool autoExposure = true;
  };

  CameraSubsystem();
  CameraSubsystem(const Settings& settings);
  ~CameraSubsystem() override;

  void Start();
  void Stop();
  bool IsRunning() const { return m_running.load(); }

  void SetResolution(int w, int h);
  void SetFPS(int fps);
  void SetAutoExposure(bool en);

  void InitDashboard();
  void Periodic() override;

  double GetAppleDistance();
  
  // getter method for the namespace
  const std::string& GetNamespace() const { return m_ns; }

private:
  // Configuration
  Settings m_cfg;
  std::string m_ns = "Camera/"; // NT/SD path prefix

  // Camera/capture state
  std::atomic_bool m_running{false};
  std::thread m_thread;
  std::atomic<double> m_measuredFps{0.0};

  // NetworkTables entries
  nt::NetworkTableEntry m_ntRunning;
  nt::NetworkTableEntry m_ntFps;

  // Processing thread
  void VisionThread_();
};