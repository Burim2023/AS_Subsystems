#include "subsystems/vision/CameraSubsystem.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// WPILib / CameraServer
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTableInstance.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>      // ArUco

// Orbbec
#include <libobsensor/ObSensor.hpp>

using namespace std::chrono_literals;

//============================================================
// PIMPL implementation with all OpenCV types
//============================================================
struct CameraSubsystem::Impl {
  std::unique_ptr<ob::Context>        obContext;
  std::unique_ptr<ob::Pipeline>       pipeline;
  std::unique_ptr<cs::CvSource>       colorSource;
  std::unique_ptr<cs::CvSource>       processedSource;
  std::atomic<bool>                   cameraRunning{false};
  std::unique_ptr<cv::Mat>            latestFrame;
  std::mutex                          frameMutex;

  // ArUco reusable objects
  cv::Ptr<cv::aruco::Dictionary>        arucoDict;
  cv::Ptr<cv::aruco::DetectorParameters> arucoParams;

  // cache of last detected marker id
  std::atomic<int> lastArucoId{-1};

  Impl() : latestFrame(std::make_unique<cv::Mat>()) {
    // Choose the SAME dictionary you’ll use on the field
    arucoDict  = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    arucoParams = cv::aruco::DetectorParameters::create();
    arucoParams->cornerRefinementMethod   = cv::aruco::CORNER_REFINE_SUBPIX;
    arucoParams->minDistanceToBorder      = 2;
    arucoParams->minMarkerPerimeterRate   = 0.03f; // accept a bit smaller markers
  }
};

//============================================================
// Apple detection (HSV masks)
//============================================================
struct AppleResult {
  cv::Rect rect;
  int color; // 1=red, 2=yellow, 3=green
  std::string colorName;
};

static std::vector<AppleResult> detectApples(const cv::Mat &frame) {
  cv::Mat hsv, maskRed1, maskRed2, maskYellow1, maskYellow2, maskGreen1, maskGreen2, blurred;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  // Red
  cv::inRange(hsv, cv::Scalar(0, 100, 100),   cv::Scalar(10, 255, 255),  maskRed1);
  cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), maskRed2);
  // Yellow
  cv::inRange(hsv, cv::Scalar(15, 100, 100),  cv::Scalar(25, 255, 255),  maskYellow1);
  cv::inRange(hsv, cv::Scalar(25, 80, 80),    cv::Scalar(35, 255, 255),  maskYellow2);
  // Green
  cv::inRange(hsv, cv::Scalar(35, 80, 80),    cv::Scalar(60, 255, 255),  maskGreen1);
  cv::inRange(hsv, cv::Scalar(60, 100, 100),  cv::Scalar(85, 255, 255),  maskGreen2);

  std::vector<AppleResult> appleResults;
  std::vector<std::pair<cv::Mat, std::pair<int, std::string>>> colorMasks = {
      {maskRed1 | maskRed2,     {1, "RED"}},
      {maskYellow1 | maskYellow2,{2, "YELLOW"}},
      {maskGreen1 | maskGreen2, {3, "GREEN"}}
  };

  for (const auto &maskInfo : colorMasks) {
    cv::Mat mask = maskInfo.first;
    int colorCode = maskInfo.second.first;
    std::string colorName = maskInfo.second.second;

    cv::GaussianBlur(mask, blurred, cv::Size(5, 5), 0);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(blurred, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto &contour : contours) {
      double area = cv::contourArea(contour);
      if (area > 500) {
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

//============================================================
// ArUco detection helper
//============================================================
struct ArucoResult {
  int id = -1;
  std::vector<cv::Point2f> corners; // 4 points
  cv::Point2f center{-1.f, -1.f};
  double area = 0.0;
};

static std::vector<ArucoResult> detectArucoMarkers(
    const cv::Mat &bgr,
    const cv::Ptr<cv::aruco::Dictionary> &dict,
    const cv::Ptr<cv::aruco::DetectorParameters> &params) {

  cv::Mat gray;
  if (bgr.channels() == 3) cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  else                     gray = bgr;

  std::vector<std::vector<cv::Point2f>> corners, rejected;
  std::vector<int> ids;

#if (defined(CV_VERSION_MAJOR) && (CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)))
  cv::aruco::ArucoDetector detector(dict, params);
  detector.detectMarkers(gray, corners, ids, rejected);
#else
  cv::aruco::detectMarkers(gray, dict, corners, ids, params, rejected);
#endif

  std::vector<ArucoResult> out;
  out.reserve(ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    const auto &c = corners[i];
    cv::Point2f ctr(0, 0);
    for (const auto &p : c) ctr += p;
    ctr.x /= static_cast<float>(c.size());
    ctr.y /= static_cast<float>(c.size());
    double area = c.size() >= 4 ? std::fabs(cv::contourArea(c)) : 0.0;
    out.push_back(ArucoResult{ ids[i], c, ctr, area });
  }

  std::sort(out.begin(), out.end(),
            [](const ArucoResult &a, const ArucoResult &b) { return a.area > b.area; });
  return out;
}

//============================================================
// CameraSubsystem impl
//============================================================

CameraSubsystem::CameraSubsystem() : CameraSubsystem(Settings{}) {}

CameraSubsystem::CameraSubsystem(const Settings &settings)
    : m_cfg(settings), m_impl(std::make_unique<Impl>()) {
  auto nt = nt::NetworkTableInstance::GetDefault().GetTable("Camera");
  m_ntFps = nt->GetEntry("fps");
  m_ntRunning = nt->GetEntry("running");
  m_ntRunning.SetBoolean(false);
  m_ntFps.SetDouble(0.0);
}

CameraSubsystem::~CameraSubsystem() { Stop(); }

void CameraSubsystem::Start() {
  if (m_running.exchange(true))
    return;

  frc::SmartDashboard::PutString(m_ns + "Debug", "Starting Orbbec camera...");

  try {
    // Initialize Orbbec camera
    m_impl->obContext = std::make_unique<ob::Context>();
    std::shared_ptr<ob::DeviceList> deviceList = m_impl->obContext->queryDeviceList();

    if (deviceList->deviceCount() <= 0) {
      throw std::runtime_error("No Orbbec devices found");
    }

    frc::SmartDashboard::PutNumber(m_ns + "CamerasFound", deviceList->deviceCount());
    frc::SmartDashboard::PutString(m_ns + "Debug",
                                   "Found " + std::to_string(deviceList->deviceCount()) + " Orbbec devices");

    std::shared_ptr<ob::Device> depthCamera = deviceList->getDevice(0);
    m_impl->pipeline = std::make_unique<ob::Pipeline>(depthCamera);

    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Configure color stream
    std::shared_ptr<ob::StreamProfileList> colorProfiles =
        m_impl->pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    std::shared_ptr<ob::VideoStreamProfile> colorStreamProfile =
        colorProfiles->getVideoStreamProfile(m_cfg.width, m_cfg.height, OB_FORMAT_RGB, m_cfg.fps);

    if (!colorStreamProfile) {
      throw std::runtime_error("Failed to get color stream profile");
    }

    config->enableStream(colorStreamProfile);

    // Create camera streams
    auto server = frc::CameraServer::GetInstance();
    m_impl->colorSource = std::make_unique<cs::CvSource>(
        server->PutVideo("OrbbecColor", m_cfg.width, m_cfg.height));
    m_impl->processedSource = std::make_unique<cs::CvSource>(
        server->PutVideo("Processed", m_cfg.width, m_cfg.height));

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
  } catch (const std::exception &e) {
    frc::SmartDashboard::PutString(m_ns + "Debug", "ERROR: " + std::string(e.what()));
    m_running.store(false);
    m_ntRunning.SetBoolean(false);

    // Cleanup on error
    m_impl->cameraRunning = false;
    if (m_impl->pipeline) {
      try {
        m_impl->pipeline->stop();
      } catch (...) {}
      m_impl->pipeline.reset();
    }
    if (m_impl->obContext) {
      m_impl->obContext.reset();
    }
    if (m_impl->colorSource)     m_impl->colorSource.reset();
    if (m_impl->processedSource) m_impl->processedSource.reset();
  }
}

void CameraSubsystem::Stop() {
  if (!m_running.exchange(false))
    return;

  m_impl->cameraRunning = false;

  try {
    if (m_thread.joinable())
      m_thread.join();

    // Stop Orbbec camera
    if (m_impl->pipeline) {
      m_impl->pipeline->stop();
      m_impl->pipeline.reset();
    }

    if (m_impl->obContext) {
      m_impl->obContext.reset();
    }

    if (m_impl->colorSource)     m_impl->colorSource.reset();
    if (m_impl->processedSource) m_impl->processedSource.reset();

    m_ntRunning.SetBoolean(false);
    frc::SmartDashboard::PutString(m_ns + "Debug", "Camera stopped");
  } catch (const std::exception &e) {
    std::cout << "Warning: Exception during camera shutdown: " << e.what() << std::endl;
    m_ntRunning.SetBoolean(false);
  }
}

void CameraSubsystem::VisionThread_() {
  auto last = std::chrono::steady_clock::now();
  int frames = 0;

  frc::SmartDashboard::PutString(m_ns + "Debug", "Vision thread started");

  while (m_running.load() && m_impl->cameraRunning.load()) {
    cv::Mat frame;

    // Get latest frame from camera
    {
      std::lock_guard<std::mutex> lock(m_impl->frameMutex);
      if (!m_impl->latestFrame || m_impl->latestFrame->empty()) {
        std::this_thread::sleep_for(10ms);
        continue;
      }
      frame = m_impl->latestFrame->clone();
    }

    frames++;
    frc::SmartDashboard::PutNumber(m_ns + "FrameCount", frames);

    // Process frame for detection
    cv::Mat processedFrame = frame.clone();

    // ------------------------------------------------------
    // 1) ArUco marker detection (replaces QR code)
    // ------------------------------------------------------
    auto arucos = detectArucoMarkers(frame, m_impl->arucoDict, m_impl->arucoParams);
    bool arucoFound = !arucos.empty();
    frc::SmartDashboard::PutBoolean(m_ns + "Aruco/Found", arucoFound);

    if (arucoFound) {
      const auto &best = arucos.front();

      // publish to dashboard
      frc::SmartDashboard::PutNumber(m_ns + "Aruco/Id", best.id);
      frc::SmartDashboard::PutNumber(m_ns + "Aruco/Cx", best.center.x);
      frc::SmartDashboard::PutNumber(m_ns + "Aruco/Cy", best.center.y);

      // cache for programmatic reads
      m_impl->lastArucoId.store(best.id);

      // draw all markers with IDs
      std::vector<std::vector<cv::Point2f>> allCorners;
      std::vector<int> allIds;
      allCorners.reserve(arucos.size());
      allIds.reserve(arucos.size());
      for (const auto &r : arucos) { allCorners.push_back(r.corners); allIds.push_back(r.id); }
      cv::aruco::drawDetectedMarkers(processedFrame, allCorners, allIds);

      // emphasize chosen marker
      cv::circle(processedFrame, best.center, 4, cv::Scalar(0, 255, 0), -1);
      cv::putText(processedFrame, "ID=" + std::to_string(best.id),
                  best.corners[0] + cv::Point2f(0, -8),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    } else {
      m_impl->lastArucoId.store(-1);
      frc::SmartDashboard::PutNumber(m_ns + "Aruco/Id", -1);
      frc::SmartDashboard::PutNumber(m_ns + "Aruco/Cx", -1);
      frc::SmartDashboard::PutNumber(m_ns + "Aruco/Cy", -1);
    }

    // ------------------------------------------------------
    // 2) Apple color detection (unchanged)
    // ------------------------------------------------------
    std::vector<AppleResult> appleResults = detectApples(frame);
    bool appleFound = !appleResults.empty();
    frc::SmartDashboard::PutBoolean(m_ns + "Apple/Found", appleFound);

    if (appleFound) {
      AppleResult largestApple = *std::max_element(
          appleResults.begin(), appleResults.end(),
          [](const AppleResult &a, const AppleResult &b) {
            return a.rect.area() < b.rect.area();
          });

      cv::Point2f center(largestApple.rect.x + largestApple.rect.width / 2.0f,
                         largestApple.rect.y + largestApple.rect.height / 2.0f);
      float radius = std::max(largestApple.rect.width, largestApple.rect.height) / 2.0f;

      frc::SmartDashboard::PutNumber(m_ns + "Apple/Cx", center.x);
      frc::SmartDashboard::PutNumber(m_ns + "Apple/Cy", center.y);
      frc::SmartDashboard::PutNumber(m_ns + "Apple/Radius", radius);
      frc::SmartDashboard::PutString(m_ns + "Apple/Color", std::to_string(largestApple.color));

      double distance = GetAppleDistance();
      frc::SmartDashboard::PutString(
          m_ns + "Debug",
          largestApple.colorName + " apple detected at " +
              (distance > 0 ? std::to_string(distance) + "mm" : "unknown distance"));

      cv::Scalar drawColor;
      switch (largestApple.color) {
        case 1: drawColor = cv::Scalar(0, 0, 255); break;      // red
        case 2: drawColor = cv::Scalar(0, 255, 255); break;    // yellow
        case 3: drawColor = cv::Scalar(0, 255, 0);   break;    // green
        default: drawColor = cv::Scalar(255, 255, 255); break; // unknown
      }

      cv::rectangle(processedFrame, largestApple.rect, drawColor, 3);
      cv::circle(processedFrame, cv::Point(center.x, center.y), 3, drawColor, -1);
      std::string label = largestApple.colorName + " (" + std::to_string(largestApple.color) + ")";
      cv::putText(processedFrame, label,
                  cv::Point(largestApple.rect.x, largestApple.rect.y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, drawColor, 2);

      for (const auto &apple : appleResults) {
        if (apple.rect != largestApple.rect) {
          cv::Scalar debugColor;
          switch (apple.color) {
            case 1: debugColor = cv::Scalar(0, 0, 128); break;
            case 2: debugColor = cv::Scalar(0, 128, 128); break;
            case 3: debugColor = cv::Scalar(0, 128, 0); break;
            default: debugColor = cv::Scalar(128, 128, 128); break;
          }
          cv::rectangle(processedFrame, apple.rect, debugColor, 1);
          cv::putText(processedFrame, std::to_string(apple.color),
                      cv::Point(apple.rect.x, apple.rect.y + 15),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, debugColor, 1);
        }
      }
    } else {
      frc::SmartDashboard::PutString(m_ns + "Apple/Color", "0"); // 0 = no apple
    }

    // Send processed frame with all drawings
    if (m_impl->processedSource) {
      m_impl->processedSource->PutFrame(processedFrame);
    }

    // FPS calc
    const auto now = std::chrono::steady_clock::now();
    if (now - last >= 1s) {
      m_measuredFps.store(static_cast<double>(frames));
      m_ntFps.SetDouble(m_measuredFps.load());
      frames = 0;
      last = now;
    }

    std::this_thread::sleep_for(16ms); // ~60 FPS cap
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

  // ArUco
  frc::SmartDashboard::PutBoolean(m_ns + "Aruco/Found", false);
  frc::SmartDashboard::PutNumber(m_ns + "Aruco/Id", -1);
  frc::SmartDashboard::PutNumber(m_ns + "Aruco/Cx", -1);
  frc::SmartDashboard::PutNumber(m_ns + "Aruco/Cy", -1);

  // Apple
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

  // Debug
  frc::SmartDashboard::PutString(m_ns + "Debug", "Not started");
  frc::SmartDashboard::PutNumber(m_ns + "CamerasFound", 0);
  frc::SmartDashboard::PutNumber(m_ns + "FrameCount", 0);
}

void CameraSubsystem::Periodic() {
  // Update FPS measurement
  frc::SmartDashboard::PutNumber(m_ns + "FPS_Measured", m_measuredFps.load());
}

// Distance estimate to the biggest apple (2D heuristic)
// Uses the apparent pixel diameter and a nominal focal length.
// Returns -1.0 when not available.
double CameraSubsystem::GetAppleDistance() {
  bool appleFound = frc::SmartDashboard::GetBoolean(m_ns + "Apple/Found", false);
  if (!appleFound) {
    return -1.0; // No apple detected
  }

  double appleCx = frc::SmartDashboard::GetNumber(m_ns + "Apple/Cx", -1);
  double appleCy = frc::SmartDashboard::GetNumber(m_ns + "Apple/Cy", -1);
  if (appleCx < 0 || appleCy < 0) {
    return -1.0; // Invalid coordinates
  }

  double appleRadius = frc::SmartDashboard::GetNumber(m_ns + "Apple/Radius", -1);
  if (appleRadius > 0) {
    const double FOCAL_LENGTH_PIXELS     = 320.0;
    const double REAL_APPLE_DIAMETER_MM  = 70.0; // 7cm in mm
    double applePixelDiameter = appleRadius * 2.0;
    double distanceMM = (REAL_APPLE_DIAMETER_MM * FOCAL_LENGTH_PIXELS) / applePixelDiameter;

    if (distanceMM < 200.0)  distanceMM = 200.0;
    if (distanceMM > 2500.0) distanceMM = 2500.0;

    frc::SmartDashboard::PutNumber(m_ns + "Apple/Distance_MM", distanceMM);
    return distanceMM;
  }

  return -1.0;
}

// OPTIONAL convenience: current ArUco ID (returns -1 if none)
// NOTE: add the declaration to CameraSubsystem.h: int GetArucoId() const;
int CameraSubsystem::GetArucoId() const {
  return m_impl->lastArucoId.load();
}