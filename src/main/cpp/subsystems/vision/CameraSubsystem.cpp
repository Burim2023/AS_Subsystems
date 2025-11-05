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

// FIXED: Moved globals to safer static variables (will be cleaned up properly)
// Note: These should ideally be member variables, but keeping as statics for minimal changes
static std::unique_ptr<ob::Context> obContext = nullptr; // FIXED: Use smart pointer
static std::unique_ptr<ob::Pipeline> pipeline = nullptr; // FIXED: Use smart pointer
static cs::CvSource colorSource;
static cs::CvSource processedSource;
static std::atomic<bool> cameraRunning{false};
static cv::Mat latestFrame;
static std::mutex frameMutex;

// QR Code detector
static cv::QRCodeDetector qrDecoder;

std::string detectQRCode(const cv::Mat &frame, std::vector<cv::Point> &bbox)
{
  return qrDecoder.detectAndDecode(frame, bbox);
}

struct AppleResult
{
  cv::Rect rect;
  int color; // 1=red, 2=yellow, 3=green
  std::string colorName;
};

std::vector<AppleResult> detectApples(const cv::Mat &frame)
{
  cv::Mat hsv, maskRed1, maskRed2, maskYellow, maskGreen, blurred;

  // Convert to HSV color space
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  // Color filters
  cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), maskRed1);    // Light red
  cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), maskRed2); // Dark red
  cv::inRange(hsv, cv::Scalar(15, 100, 100), cv::Scalar(35, 255, 255), maskYellow); // Yellow
  cv::inRange(hsv, cv::Scalar(35, 80, 80), cv::Scalar(85, 255, 255), maskGreen);    // Green

  std::vector<AppleResult> appleResults;

  // Process each color separately to identify the dominant color
  std::vector<std::pair<cv::Mat, std::pair<int, std::string>>> colorMasks = {
      {maskRed1 | maskRed2, {1, "RED"}},
      {maskYellow, {2, "YELLOW"}},
      {maskGreen, {3, "GREEN"}}};

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
      if (area > 500)
      { // Minimum area to avoid noise
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

void processFrameset(std::shared_ptr<ob::FrameSet> frameSet)
{
  std::shared_ptr<ob::ColorFrame> obColorFrame = frameSet->colorFrame();

  if (!obColorFrame)
    return;

  cv::Mat colorFrame(obColorFrame->height(), obColorFrame->width(), CV_8UC3);
  std::memcpy(colorFrame.data, obColorFrame->data(), colorFrame.total() * colorFrame.elemSize());
  cv::cvtColor(colorFrame, colorFrame, cv::COLOR_RGB2BGR);

  // Store latest frame for processing
  {
    std::lock_guard<std::mutex> lock(frameMutex);
    latestFrame = colorFrame.clone();
  }

  // Send original frame to camera stream
  colorSource.PutFrame(colorFrame);
}

CameraSubsystem::CameraSubsystem()
    : CameraSubsystem(Settings{}) {}

CameraSubsystem::CameraSubsystem(const Settings &settings)
    : m_cfg(settings)
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
    obContext = std::make_unique<ob::Context>(); // FIXED: Use smart pointer
    std::shared_ptr<ob::DeviceList> deviceList = obContext->queryDeviceList();

    if (deviceList->deviceCount() <= 0)
    {
      throw std::runtime_error("No Orbbec devices found");
    }

    frc::SmartDashboard::PutNumber(m_ns + "CamerasFound", deviceList->deviceCount());
    frc::SmartDashboard::PutString(m_ns + "Debug", "Found " + std::to_string(deviceList->deviceCount()) + " Orbbec devices");

    std::shared_ptr<ob::Device> depthCamera = deviceList->getDevice(0);
    pipeline = std::make_unique<ob::Pipeline>(depthCamera); // FIXED: Use smart pointer

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Configure color stream
    std::shared_ptr<ob::StreamProfileList> colorProfiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    std::shared_ptr<ob::VideoStreamProfile> colorStreamProfile =
        colorProfiles->getVideoStreamProfile(m_cfg.width, m_cfg.height, OB_FORMAT_RGB, m_cfg.fps);

    if (!colorStreamProfile)
    {
      throw std::runtime_error("Failed to get color stream profile");
    }

    config->enableStream(colorStreamProfile);

    // Create camera streams
    auto server = frc::CameraServer::GetInstance();
    colorSource = server->PutVideo("OrbbecColor", m_cfg.width, m_cfg.height);
    processedSource = server->PutVideo("Processed", m_cfg.width, m_cfg.height);

    frc::SmartDashboard::PutString(m_ns + "Debug", "Starting camera pipeline...");
    pipeline->start(config, processFrameset);

    cameraRunning = true;

    // Start vision processing thread
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

    // FIXED: Cleanup on error - smart pointers handle this automatically
    if (pipeline)
    {
      pipeline->stop();
      pipeline.reset(); // FIXED: Use reset() for smart pointer
    }
    if (obContext)
    {
      obContext.reset(); // FIXED: Use reset() for smart pointer
    }
  }
}

void CameraSubsystem::Stop()
{
  if (!m_running.exchange(false))
    return;

  cameraRunning = false;

  if (m_thread.joinable())
    m_thread.join();

  // FIXED: Stop Orbbec camera - smart pointers handle cleanup automatically
  if (pipeline)
  {
    pipeline->stop();
    pipeline.reset(); // FIXED: Use reset() for smart pointer
  }

  if (obContext)
  {
    obContext.reset(); // FIXED: Use reset() for smart pointer
  }

  m_ntRunning.SetBoolean(false);
  frc::SmartDashboard::PutString(m_ns + "Debug", "Camera stopped");
}

void CameraSubsystem::SetResolution(int w, int h)
{
  m_cfg.width = w;
  m_cfg.height = h;
  frc::SmartDashboard::PutNumber(m_ns + "Width", w);
  frc::SmartDashboard::PutNumber(m_ns + "Height", h);
  frc::SmartDashboard::PutString(m_ns + "Debug", "Resolution change requires camera restart");
}

void CameraSubsystem::SetFPS(int fps)
{
  m_cfg.fps = fps;
  frc::SmartDashboard::PutNumber(m_ns + "FPS_Set", fps);
  frc::SmartDashboard::PutString(m_ns + "Debug", "FPS change requires camera restart");
}

void CameraSubsystem::SetAutoExposure(bool en)
{
  m_cfg.autoExposure = en;
  frc::SmartDashboard::PutBoolean(m_ns + "AutoExp", en);
  frc::SmartDashboard::PutString(m_ns + "Debug", "Orbbec exposure handled automatically");
}

void CameraSubsystem::InitDashboard()
{
  // Seed SmartDashboard keys for widgets
  frc::SmartDashboard::PutString(m_ns + "Color", "none");
  frc::SmartDashboard::PutBoolean(m_ns + "QR/Found", false);
  frc::SmartDashboard::PutString(m_ns + "QR/Text", "—");
  frc::SmartDashboard::PutBoolean(m_ns + "Barcode/Found", false);
  frc::SmartDashboard::PutString(m_ns + "Barcode/Text", "—");
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

void CameraSubsystem::Periodic()
{
  // Update FPS measurement
  frc::SmartDashboard::PutNumber(m_ns + "FPS_Measured", m_measuredFps.load());
}

void CameraSubsystem::VisionThread_()
{
  auto last = std::chrono::steady_clock::now();
  int frames = 0;

  frc::SmartDashboard::PutString(m_ns + "Debug", "Vision thread started");

  while (m_running.load() && cameraRunning.load())
  {
    cv::Mat frame;

    // Get latest frame from camera
    {
      std::lock_guard<std::mutex> lock(frameMutex);
      if (latestFrame.empty())
      {
        std::this_thread::sleep_for(10ms);
        continue;
      }
      frame = latestFrame.clone();
    }

    frames++;
    frc::SmartDashboard::PutNumber(m_ns + "FrameCount", frames);

    // Process frame for apple detection
    cv::Mat processedFrame = frame.clone();

    // 1) QR Code detection
    std::vector<cv::Point> qrBbox;
    std::string qrText = detectQRCode(frame, qrBbox);
    bool qrFound = !qrText.empty();
    frc::SmartDashboard::PutBoolean(m_ns + "QR/Found", qrFound);
    frc::SmartDashboard::PutString(m_ns + "QR/Text", qrFound ? qrText : "—");

    // Draw QR code bbox if found
    if (qrFound && qrBbox.size() >= 4)
    {
      cv::polylines(processedFrame, qrBbox, true, cv::Scalar(255, 0, 0), 2);
      cv::putText(processedFrame, "QR: " + qrText, qrBbox[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    }

    // 2) Apple detection with color identification
    std::vector<AppleResult> appleResults = detectApples(frame);
    bool appleFound = !appleResults.empty();

    frc::SmartDashboard::PutBoolean(m_ns + "Apple/Found", appleFound);

    if (appleFound)
    {
      // Use the largest apple (most likely to be the target)
      AppleResult largestApple = *std::max_element(appleResults.begin(), appleResults.end(),
                                                   [](const AppleResult &a, const AppleResult &b)
                                                   { return a.rect.area() < b.rect.area(); });

      // Calculate apple properties
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
                                                         (distance > 0 ? std::to_string(distance) + "cm" : "unknown distance"));

      // Choose drawing color based on apple color
      cv::Scalar drawColor;
      switch (largestApple.color)
      {
      case 1:
        drawColor = cv::Scalar(0, 0, 255);
        break; // Red apple = red square
      case 2:
        drawColor = cv::Scalar(0, 255, 255);
        break; // Yellow apple = yellow square
      case 3:
        drawColor = cv::Scalar(0, 255, 0);
        break; // Green apple = green square
      default:
        drawColor = cv::Scalar(255, 255, 255);
        break; // White for unknown
      }

      // Draw colored square around detected apple
      cv::rectangle(processedFrame, largestApple.rect, drawColor, 3);

      // Draw center point
      cv::circle(processedFrame, cv::Point(center.x, center.y), 3, drawColor, -1);

      // Add text label with color name and number
      std::string label = largestApple.colorName + " (" + std::to_string(largestApple.color) + ")";
      cv::putText(processedFrame, label, cv::Point(largestApple.rect.x, largestApple.rect.y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, drawColor, 2);

      // Draw all detected apples with their colors (for debugging)
      for (const auto &apple : appleResults)
      {
        if (apple.rect != largestApple.rect)
        {
          cv::Scalar debugColor;
          switch (apple.color)
          {
          case 1:
            debugColor = cv::Scalar(0, 0, 128);
            break; // Dark red
          case 2:
            debugColor = cv::Scalar(0, 128, 128);
            break; // Dark yellow
          case 3:
            debugColor = cv::Scalar(0, 128, 0);
            break; // Dark green
          default:
            debugColor = cv::Scalar(128, 128, 128);
            break;
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

    // Send processed frame with drawings
    processedSource.PutFrame(processedFrame);

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

double CameraSubsystem::GetAppleDistance()
{
  // Get apple detection status
  bool appleFound = frc::SmartDashboard::GetBoolean(m_ns + "Apple/Found", false);

  if (!appleFound)
  {
    return -1.0; // No apple detected
  }

  // Get apple center coordinates
  double appleCx = frc::SmartDashboard::GetNumber(m_ns + "Apple/Cx", -1);
  double appleCy = frc::SmartDashboard::GetNumber(m_ns + "Apple/Cy", -1);

  if (appleCx < 0 || appleCy < 0)
  {
    return -1.0; // Invalid coordinates
  }

  // TODO: Replace this with actual depth camera access
  // For Orbbec Gemini E, you would access the depth frame here
  // Example: depth_value = depth_frame.at<uint16_t>(appleCy, appleCx);

  // Temporary: Use the existing radius-based calculation converted to mm
  double appleRadius = frc::SmartDashboard::GetNumber(m_ns + "Apple/Radius", -1);

  if (appleRadius > 0)
  {
    // Camera calibration constants for depth estimation
    const double FOCAL_LENGTH_PIXELS = 320.0;
    const double REAL_APPLE_DIAMETER_MM = 70.0; // 7cm in mm

    double applePixelDiameter = appleRadius * 2.0;
    double distanceMM = (REAL_APPLE_DIAMETER_MM * FOCAL_LENGTH_PIXELS) / applePixelDiameter;

    // Clamp to valid depth range
    if (distanceMM < 200.0)
      distanceMM = 200.0;
    if (distanceMM > 2500.0)
      distanceMM = 2500.0;

    // Put distance on SmartDashboard
    frc::SmartDashboard::PutNumber(m_ns + "Apple/Distance_MM", distanceMM);

    return distanceMM;
  }

  return -1.0;
}