#include "subsystems/vision/CameraSubsystem.h"

#include <networktables/NetworkTableInstance.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>
#include <chrono>
#include <mutex>
#include <libobsensor/ObSensor.hpp>

using namespace std::chrono_literals;

// PIMPL implementation with all OpenCV types
struct CameraSubsystem::Impl {
  std::unique_ptr<ob::Context> obContext;
  std::unique_ptr<ob::Pipeline> pipeline;
  std::unique_ptr<cs::CvSource> colorSource;
  std::unique_ptr<cs::CvSource> processedSource;
  std::atomic<bool> cameraRunning{false};
  std::unique_ptr<cv::Mat> latestFrame;
  std::mutex frameMutex;
  std::unique_ptr<cv::QRCodeDetector> qrDecoder;
  
  Impl() : latestFrame(std::make_unique<cv::Mat>()), qrDecoder(std::make_unique<cv::QRCodeDetector>()) {}
};

// IMPROVED Apple detection from your old version
struct AppleResult
{
  cv::Rect rect;
  int color; // 1=red, 2=yellow, 3=green
  std::string colorName;
};

std::string detectQRCode(const cv::Mat &frame, std::vector<cv::Point> &bbox, cv::QRCodeDetector& decoder)
{
  return decoder.detectAndDecode(frame, bbox);
}

// IMPROVED: Better apple detection with wider HSV ranges from old version
std::vector<AppleResult> detectApples(const cv::Mat &frame)
{
  cv::Mat hsv, maskRed1, maskRed2, maskYellow1, maskYellow2, maskGreen1, maskGreen2, blurred;
  
  // Convert to HSV color space
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
  
  // IMPROVED: Color filters with wider ranges from old version
  cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), maskRed1);    // Light red
  cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), maskRed2); // Dark red
  
  cv::inRange(hsv, cv::Scalar(15, 100, 100), cv::Scalar(25, 255, 255), maskYellow1); // Light yellow
  cv::inRange(hsv, cv::Scalar(25, 80, 80), cv::Scalar(35, 255, 255), maskYellow2);   // Dark yellow
  
  cv::inRange(hsv, cv::Scalar(35, 80, 80), cv::Scalar(60, 255, 255), maskGreen1);    // Light green
  cv::inRange(hsv, cv::Scalar(60, 100, 100), cv::Scalar(85, 255, 255), maskGreen2);  // Dark green

  std::vector<AppleResult> appleResults;

  // IMPROVED: Process each color with better masks from old version
  std::vector<std::pair<cv::Mat, std::pair<int, std::string>>> colorMasks = {
      {maskRed1 | maskRed2, {1, "RED"}},
      {maskYellow1 | maskYellow2, {2, "YELLOW"}},
      {maskGreen1 | maskGreen2, {3, "GREEN"}}};

  for (const auto &maskInfo : colorMasks)
  {
    cv::Mat mask = maskInfo.first;
    int colorCode = maskInfo.second.first;
    std::string colorName = maskInfo.second.second;

    // Remove noise
    cv::GaussianBlur(mask, blurred, cv::Size(5, 5), 0);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(blurred, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Store all valid contours with their color
    for (const auto &contour : contours)
    {
      double area = cv::contourArea(contour);
      if (area > 500) // Minimum area to avoid noise
      {
        AppleResult result;
        result.rect = cv::boundingRect(contour);
        result.color = colorCode;
        result.colorName = colorName;
        appleResults.push_back(result);
      }
    }
  }

  return appleResults;
}

CameraSubsystem::CameraSubsystem()
    : CameraSubsystem(Settings{}) {}

CameraSubsystem::CameraSubsystem(const Settings &settings)
    : m_cfg(settings), m_impl(std::make_unique<Impl>())
{
  auto nt = nt::NetworkTableInstance::GetDefault().GetTable("Camera");
  m_ntFps = nt->GetEntry("fps");
  m_ntRunning = nt->GetEntry("running");
  m_ntRunning.SetBoolean(false);
  m_ntFps.SetDouble(0.0);
}

CameraSubsystem::~CameraSubsystem()
{
  Stop();
}

void CameraSubsystem::Start()
{
  if (m_running.exchange(true))
    return;

  frc::SmartDashboard::PutString(m_ns + "Debug", "Starting Orbbec camera...");

  try
  {
    // Initialize Orbbec camera
    m_impl->obContext = std::make_unique<ob::Context>();
    std::shared_ptr<ob::DeviceList> deviceList = m_impl->obContext->queryDeviceList();

    if (deviceList->deviceCount() <= 0)
    {
      throw std::runtime_error("No Orbbec devices found");
    }

    frc::SmartDashboard::PutNumber(m_ns + "CamerasFound", deviceList->deviceCount());
    frc::SmartDashboard::PutString(m_ns + "Debug", "Found " + std::to_string(deviceList->deviceCount()) + " Orbbec devices");

    std::shared_ptr<ob::Device> depthCamera = deviceList->getDevice(0);
    m_impl->pipeline = std::make_unique<ob::Pipeline>(depthCamera);

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Configure color stream
    std::shared_ptr<ob::StreamProfileList> colorProfiles = m_impl->pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    std::shared_ptr<ob::VideoStreamProfile> colorStreamProfile =
        colorProfiles->getVideoStreamProfile(m_cfg.width, m_cfg.height, OB_FORMAT_RGB, m_cfg.fps);

    if (!colorStreamProfile)
    {
      throw std::runtime_error("Failed to get color stream profile");
    }

    config->enableStream(colorStreamProfile);

    // Create camera streams
    auto server = frc::CameraServer::GetInstance();
    m_impl->colorSource = std::make_unique<cs::CvSource>(server->PutVideo("OrbbecColor", m_cfg.width, m_cfg.height));
    m_impl->processedSource = std::make_unique<cs::CvSource>(server->PutVideo("Processed", m_cfg.width, m_cfg.height));

    frc::SmartDashboard::PutString(m_ns + "Debug", "Starting camera pipeline...");

    // Start pipeline with frame processing
    m_impl->pipeline->start(config, [this](std::shared_ptr<ob::FrameSet> frameSet) {
      if (!m_impl->cameraRunning.load()) {
        return;
      }

      std::shared_ptr<ob::ColorFrame> obColorFrame = frameSet->colorFrame();
      if (!obColorFrame) return;

      cv::Mat colorFrame(obColorFrame->height(), obColorFrame->width(), CV_8UC3);
      std::memcpy(colorFrame.data, obColorFrame->data(), colorFrame.total() * colorFrame.elemSize());
      cv::cvtColor(colorFrame, colorFrame, cv::COLOR_RGB2BGR);

      // Store latest frame for processing
      {
        std::lock_guard<std::mutex> lock(m_impl->frameMutex);
        *m_impl->latestFrame = colorFrame.clone();
      }

      // Send original frame to camera stream
      if (m_impl->colorSource) {
        m_impl->colorSource->PutFrame(colorFrame);
      }
    });

    m_impl->cameraRunning = true;

    frc::SmartDashboard::PutString(m_ns + "Debug", "Starting vision thread...");
    m_thread = std::thread(&CameraSubsystem::VisionThread_, this);
    m_ntRunning.SetBoolean(true);

    frc::SmartDashboard::PutString(m_ns + "Debug", "Orbbec camera started successfully!");
  }
  catch (const std::exception &e)
  {
    frc::SmartDashboard::PutString(m_ns + "Debug", "ERROR: " + std::string(e.what()));
    m_running.store(false);
    m_ntRunning.SetBoolean(false);

    // Cleanup on error
    m_impl->cameraRunning = false;
    if (m_impl->pipeline)
    {
      try { m_impl->pipeline->stop(); } catch (...) {}
      m_impl->pipeline.reset();
    }
    if (m_impl->obContext)
    {
      m_impl->obContext.reset();
    }
    if (m_impl->colorSource) m_impl->colorSource.reset();
    if (m_impl->processedSource) m_impl->processedSource.reset();
  }
}

void CameraSubsystem::Stop()
{
  if (!m_running.exchange(false))
    return;

  m_impl->cameraRunning = false;

  try
  {
    if (m_thread.joinable())
      m_thread.join();

    // Stop Orbbec camera
    if (m_impl->pipeline)
    {
      m_impl->pipeline->stop();
      m_impl->pipeline.reset();
    }

    if (m_impl->obContext)
    {
      m_impl->obContext.reset();
    }

    if (m_impl->colorSource) m_impl->colorSource.reset();
    if (m_impl->processedSource) m_impl->processedSource.reset();

    m_ntRunning.SetBoolean(false);
    frc::SmartDashboard::PutString(m_ns + "Debug", "Camera stopped");
  }
  catch (const std::exception &e)
  {
    std::cout << "Warning: Exception during camera shutdown: " << e.what() << std::endl;
    m_ntRunning.SetBoolean(false);
  }
}

void CameraSubsystem::VisionThread_()
{
  auto last = std::chrono::steady_clock::now();
  int frames = 0;

  frc::SmartDashboard::PutString(m_ns + "Debug", "Vision thread started");

  while (m_running.load() && m_impl->cameraRunning.load())
  {
    cv::Mat frame;

    // Get latest frame from camera
    {
      std::lock_guard<std::mutex> lock(m_impl->frameMutex);
      if (!m_impl->latestFrame || m_impl->latestFrame->empty())
      {
        std::this_thread::sleep_for(10ms);
        continue;
      }
      frame = m_impl->latestFrame->clone();
    }

    frames++;
    frc::SmartDashboard::PutNumber(m_ns + "FrameCount", frames);

    // Process frame for detection
    cv::Mat processedFrame = frame.clone();

    // 1) QR Code detection with IMPROVED drawing from old version
    std::vector<cv::Point> qrBbox;
    std::string qrText = detectQRCode(frame, qrBbox, *m_impl->qrDecoder);
    bool qrFound = !qrText.empty();
    frc::SmartDashboard::PutBoolean(m_ns + "QR/Found", qrFound);
    frc::SmartDashboard::PutString(m_ns + "QR/Text", qrFound ? qrText : "—");

    // IMPROVED: Draw QR code bbox with better visualization from old version
    if (qrFound && qrBbox.size() >= 4)
    {
      cv::polylines(processedFrame, qrBbox, true, cv::Scalar(255, 0, 0), 2);
      cv::putText(processedFrame, "QR: " + qrText, qrBbox[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    }

    // 2) IMPROVED Apple detection with better color identification from old version
    std::vector<AppleResult> appleResults = detectApples(frame);
    bool appleFound = !appleResults.empty();
    frc::SmartDashboard::PutBoolean(m_ns + "Apple/Found", appleFound);

    if (appleFound)
    {
      // IMPROVED: Use the largest apple (most likely to be the target) from old version
      AppleResult largestApple = *std::max_element(appleResults.begin(), appleResults.end(),
                                                   [](const AppleResult &a, const AppleResult &b)
                                                   { return a.rect.area() < b.rect.area(); });

      // IMPROVED: Calculate apple properties from old version
      cv::Point2f center(largestApple.rect.x + largestApple.rect.width / 2.0f,
                         largestApple.rect.y + largestApple.rect.height / 2.0f);
      float radius = std::max(largestApple.rect.width, largestApple.rect.height) / 2.0f;

      frc::SmartDashboard::PutNumber(m_ns + "Apple/Cx", center.x);
      frc::SmartDashboard::PutNumber(m_ns + "Apple/Cy", center.y);
      frc::SmartDashboard::PutNumber(m_ns + "Apple/Radius", radius);
      frc::SmartDashboard::PutString(m_ns + "Apple/Color", std::to_string(largestApple.color));

      // Calculate and store distance
      double distance = GetAppleDistance();
      frc::SmartDashboard::PutString(m_ns + "Debug", largestApple.colorName + " apple detected at " +
                                                         (distance > 0 ? std::to_string(distance) + "mm" : "unknown distance"));

      // IMPROVED: Choose drawing color based on apple color from old version
      cv::Scalar drawColor;
      switch (largestApple.color)
      {
      case 1:
        drawColor = cv::Scalar(0, 0, 255); break; // Red apple = red square
      case 2:
        drawColor = cv::Scalar(0, 255, 255); break; // Yellow apple = yellow square
      case 3:
        drawColor = cv::Scalar(0, 255, 0); break; // Green apple = green square
      default:
        drawColor = cv::Scalar(255, 255, 255); break; // White for unknown
      }

      // IMPROVED: Draw colored square around detected apple from old version
      cv::rectangle(processedFrame, largestApple.rect, drawColor, 3);
      
      // IMPROVED: Draw center point from old version
      cv::circle(processedFrame, cv::Point(center.x, center.y), 3, drawColor, -1);
      
      // IMPROVED: Add text label with color name and number from old version
      std::string label = largestApple.colorName + " (" + std::to_string(largestApple.color) + ")";
      cv::putText(processedFrame, label, cv::Point(largestApple.rect.x, largestApple.rect.y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, drawColor, 2);

      // IMPROVED: Draw all detected apples with their colors (for debugging) from old version
      for (const auto &apple : appleResults)
      {
        if (apple.rect != largestApple.rect)
        {
          cv::Scalar debugColor;
          switch (apple.color)
          {
          case 1:
            debugColor = cv::Scalar(0, 0, 128); break; // Dark red
          case 2:
            debugColor = cv::Scalar(0, 128, 128); break; // Dark yellow
          case 3:
            debugColor = cv::Scalar(0, 128, 0); break; // Dark green
          default:
            debugColor = cv::Scalar(128, 128, 128); break;
          }
          cv::rectangle(processedFrame, apple.rect, debugColor, 1);
          cv::putText(processedFrame, std::to_string(apple.color),
                      cv::Point(apple.rect.x, apple.rect.y + 15),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, debugColor, 1);
        }
      }
    }
    else
    {
      frc::SmartDashboard::PutString(m_ns + "Apple/Color", "0"); // 0 = no apple
    }

    // Send processed frame with all drawings
    if (m_impl->processedSource) {
      m_impl->processedSource->PutFrame(processedFrame);
    }

    // FPS calculation
    const auto now = std::chrono::steady_clock::now();
    if (now - last >= 1s)
    {
      m_measuredFps.store(static_cast<double>(frames));
      m_ntFps.SetDouble(m_measuredFps.load());
      frames = 0;
      last = now;
    }

    std::this_thread::sleep_for(16ms); // ~60 FPS
  }

  frc::SmartDashboard::PutString(m_ns + "Debug", "Vision thread stopped");
}

void CameraSubsystem::SetResolution(int w, int h) {
  m_cfg.width = w;
  m_cfg.height = h;
  frc::SmartDashboard::PutNumber(m_ns + "Width", w);
  frc::SmartDashboard::PutNumber(m_ns + "Height", h);
  frc::SmartDashboard::PutString(m_ns + "Debug", "Resolution change requires camera restart");
}

void CameraSubsystem::SetFPS(int fps) {
  m_cfg.fps = fps;
  frc::SmartDashboard::PutNumber(m_ns + "FPS_Set", fps);
  frc::SmartDashboard::PutString(m_ns + "Debug", "FPS change requires camera restart");
}

void CameraSubsystem::SetAutoExposure(bool en) {
  m_cfg.autoExposure = en;
  frc::SmartDashboard::PutBoolean(m_ns + "AutoExp", en);
  frc::SmartDashboard::PutString(m_ns + "Debug", "Orbbec exposure handled automatically");
}

void CameraSubsystem::InitDashboard() {
  // Seed SmartDashboard keys for widgets
  frc::SmartDashboard::PutString(m_ns + "Color", "none");
  frc::SmartDashboard::PutBoolean(m_ns + "QR/Found", false);
  frc::SmartDashboard::PutString(m_ns + "QR/Text", "—");
  frc::SmartDashboard::PutBoolean(m_ns + "Apple/Found", false);
  frc::SmartDashboard::PutString(m_ns + "Apple/Color", "none");
  frc::SmartDashboard::PutNumber(m_ns + "Apple/Cx", -1);
  frc::SmartDashboard::PutNumber(m_ns + "Apple/Cy", -1);
  frc::SmartDashboard::PutNumber(m_ns + "Apple/Radius", -1);
  
  // Camera config
  frc::SmartDashboard::PutString(m_ns + "Name", "OrbbecCam");
  frc::SmartDashboard::PutNumber(m_ns + "Width", m_cfg.width);
  frc::SmartDashboard::PutNumber(m_ns + "Height", m_cfg.height);
  frc::SmartDashboard::PutNumber(m_ns + "FPS_Set", m_cfg.fps);
  frc::SmartDashboard::PutBoolean(m_ns + "AutoExp", m_cfg.autoExposure);
  
  // Debug dashboard entries
  frc::SmartDashboard::PutString(m_ns + "Debug", "Not started");
  frc::SmartDashboard::PutNumber(m_ns + "CamerasFound", 0);
  frc::SmartDashboard::PutNumber(m_ns + "FrameCount", 0);
}

void CameraSubsystem::Periodic() {
  // Update FPS measurement
  frc::SmartDashboard::PutNumber(m_ns + "FPS_Measured", m_measuredFps.load());
}

double CameraSubsystem::GetAppleDistance() {
  // IMPROVED: Better distance calculation from old version
  bool appleFound = frc::SmartDashboard::GetBoolean(m_ns + "Apple/Found", false);
  if (!appleFound) {
    return -1.0; // No apple detected
  }

  // Get apple center coordinates
  double appleCx = frc::SmartDashboard::GetNumber(m_ns + "Apple/Cx", -1);
  double appleCy = frc::SmartDashboard::GetNumber(m_ns + "Apple/Cy", -1);
  if (appleCx < 0 || appleCy < 0) {
    return -1.0; // Invalid coordinates
  }

  // IMPROVED: Use radius-based calculation from old version
  double appleRadius = frc::SmartDashboard::GetNumber(m_ns + "Apple/Radius", -1);
  if (appleRadius > 0) {
    // Camera calibration constants for depth estimation
    const double FOCAL_LENGTH_PIXELS = 320.0;
    const double REAL_APPLE_DIAMETER_MM = 70.0; // 7cm in mm
    double applePixelDiameter = appleRadius * 2.0;
    double distanceMM = (REAL_APPLE_DIAMETER_MM * FOCAL_LENGTH_PIXELS) / applePixelDiameter;
    
    // Clamp to valid depth range
    if (distanceMM < 200.0) distanceMM = 200.0;
    if (distanceMM > 2500.0) distanceMM = 2500.0;
    
    // Put distance on SmartDashboard
    frc::SmartDashboard::PutNumber(m_ns + "Apple/Distance_MM", distanceMM);
    return distanceMM;
  }

  return -1.0;
}