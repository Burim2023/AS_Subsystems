#include "commands/QRCodeReaderCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <iostream>

// Add this method to get the QR Data tab
std::shared_ptr<nt::NetworkTable> GetQRDataTab()
{
    return nt::NetworkTableInstance::GetDefault().GetTable("QR Data");
}

QRCodeReaderCommand::QRCodeReaderCommand(CameraSubsystem *camera, ReadMode mode)
    : m_camera(camera),
      m_mode(mode),
      m_timeout(0.0),
      m_lastQRText(""),
      m_qrFound(false),
      m_hasReadCode(false),
      m_consecutiveReads(0)
{

    SetName("QRCodeReader");
    // Declare camera subsystem requirement to prevent conflicts
    AddRequirements({m_camera});

    if (!m_camera)
    {
        std::cout << "ERROR: QRCodeReaderCommand - Camera subsystem is null!" << std::endl;
    }
}

QRCodeReaderCommand::QRCodeReaderCommand(CameraSubsystem *camera, double timeoutSeconds)
    : m_camera(camera),
      m_mode(ReadMode::TIMED_READ),
      m_timeout(timeoutSeconds),
      m_lastQRText(""),
      m_qrFound(false),
      m_hasReadCode(false),
      m_consecutiveReads(0)
{

    SetName("QRCodeReaderTimed");
    // Declare camera subsystem requirement to prevent conflicts
    AddRequirements({m_camera});

    if (!m_camera)
    {
        std::cout << "ERROR: QRCodeReaderCommand - Camera subsystem is null!" << std::endl;
    }
}

void QRCodeReaderCommand::Initialize()
{
    std::cout << "=== QRCodeReaderCommand::Initialize ===" << std::endl;
    std::cout << "Mode: " << static_cast<int>(m_mode) << std::endl;
    std::cout << "Timeout: " << m_timeout << " seconds" << std::endl;

    // Reset state
    m_lastQRText = "";
    m_qrFound = false;
    m_hasReadCode = false;
    m_consecutiveReads = 0;

    // Start timer for timed mode
    m_timer.Reset();
    m_timer.Start();

    // Get QR Data tab
    auto qrTab = GetQRDataTab();

    // FIXED: Use correct WPILib 2020 NetworkTables API
    qrTab->PutString("Status", "🔍 Initializing Scanner...");
    qrTab->PutString("Current Text", "Scanning...");
    qrTab->PutString("Confirmed Text", "");
    qrTab->PutString("Final Result", "");
    qrTab->PutBoolean("Read Complete", false);
    qrTab->PutBoolean("QR Found", false);
    qrTab->PutNumber("Consecutive Reads", 0);                     // FIXED
    qrTab->PutNumber("Min Required Reads", kMinConsecutiveReads); // FIXED
    qrTab->PutString("Mode",
                     m_mode == ReadMode::SINGLE_READ ? "Single Read" : m_mode == ReadMode::CONTINUOUS_READ ? "Continuous"
                                                                                                           : "Timed Read");
    qrTab->PutNumber("Timeout (sec)", m_timeout);
    qrTab->PutNumber("Elapsed Time", 0.0);

    // Camera status
    qrTab->PutBoolean("Camera Available", m_camera != nullptr);

    if (!m_camera)
    {
        std::cout << "ERROR: Camera not available for QR reading!" << std::endl;
        qrTab->PutString("Status", "❌ Camera Unavailable");
        return;
    }

    qrTab->PutString("Status", "🔍 Scanning for QR Code...");
    std::cout << "QR Code reading started..." << std::endl;
}

void QRCodeReaderCommand::Execute()
{
    if (!m_camera)
    {
        return;
    }

    auto qrTab = GetQRDataTab();

    // Read QR code status from SmartDashboard (set by CameraSubsystem)
    bool qrDetected = frc::SmartDashboard::GetBoolean("Camera/QR/Found", false);
    std::string qrText = frc::SmartDashboard::GetString("Camera/QR/Text", "");

    // Update elapsed time
    double elapsedTime = m_timer.Get();
    qrTab->PutNumber("Elapsed Time", elapsedTime);

    // Update current detection status
    qrTab->PutBoolean("QR Detected This Frame", qrDetected);
    qrTab->PutString("Raw QR Text", qrText);

    if (qrDetected && !qrText.empty() && qrText != "—")
    {
        // QR code detected with valid text
        if (m_lastQRText == qrText)
        {
            // Same code as before - increment consecutive reads
            m_consecutiveReads++;
            std::cout << "QR Code consistent read #" << m_consecutiveReads
                      << ": '" << qrText << "'" << std::endl;
        }
        else
        {
            // New/different code - reset consecutive counter
            m_consecutiveReads = 1;
            m_lastQRText = qrText;
            std::cout << "QR Code new reading: '" << qrText << "'" << std::endl;
        }

        // FIXED: Use PutNumber for integer values
        qrTab->PutString("Current Text", m_lastQRText);
        qrTab->PutNumber("Consecutive Reads", m_consecutiveReads);
        qrTab->PutString("Progress",
                         std::to_string(m_consecutiveReads) + "/" + std::to_string(kMinConsecutiveReads));

        // Progress bar simulation
        double progressPercent = (double)m_consecutiveReads / kMinConsecutiveReads * 100.0;
        if (progressPercent > 100.0)
            progressPercent = 100.0;
        qrTab->PutNumber("Stability Progress (%)", progressPercent);

        // Consider code "found" after multiple consecutive reads for stability
        if (m_consecutiveReads >= kMinConsecutiveReads)
        {
            if (!m_qrFound)
            {
                // First time achieving stable read
                m_qrFound = true;
                m_hasReadCode = true;

                std::cout << "✅ QR Code CONFIRMED after " << m_consecutiveReads
                          << " consecutive reads: '" << m_lastQRText << "'" << std::endl;

                qrTab->PutBoolean("QR Found", true);
                qrTab->PutBoolean("Read Complete", true);
                qrTab->PutString("Confirmed Text", m_lastQRText);
                qrTab->PutString("Status", "✅ QR Code Confirmed!");
                qrTab->PutNumber("Confirmation Time", elapsedTime);
            }
        }
        else
        {
            // Still building up consecutive reads
            qrTab->PutString("Status", "🔄 Building Stability... " +
                                           std::to_string(m_consecutiveReads) + "/" + std::to_string(kMinConsecutiveReads));
        }
    }
    else
    {
        // No QR code detected or empty text
        if (m_consecutiveReads > 0)
        {
            std::cout << "QR Code lost - resetting consecutive counter" << std::endl;
            m_consecutiveReads = 0;
        }

        qrTab->PutNumber("Consecutive Reads", 0); // FIXED
        qrTab->PutString("Progress", "0/" + std::to_string(kMinConsecutiveReads));
        qrTab->PutNumber("Stability Progress (%)", 0.0);
        qrTab->PutString("Current Text", "");
        qrTab->PutString("Status", "🔍 Scanning for QR Code...");
    }

    // Update scan statistics
    static int totalScans = 0;
    static int validDetections = 0;
    totalScans++;
    if (qrDetected)
        validDetections++;

    // FIXED: Use PutNumber for integer values
    qrTab->PutNumber("Total Scans", totalScans);
    qrTab->PutNumber("Valid Detections", validDetections);
    qrTab->PutNumber("Detection Rate (%)",
                     totalScans > 0 ? (double)validDetections / totalScans * 100.0 : 0.0);

    // Debug output every 2 seconds
    static int debugCounter = 0;
    if (++debugCounter % 100 == 0)
    { // Assuming 50Hz execution
        std::cout << "QR Reader - Detected: " << qrDetected
                  << ", Text: '" << qrText
                  << "', Consecutive: " << m_consecutiveReads
                  << ", Confirmed: " << m_qrFound << std::endl;
    }
}

void QRCodeReaderCommand::End(bool interrupted)
{
    std::cout << "=== QRCodeReaderCommand::End ===" << std::endl;
    std::cout << "Interrupted: " << interrupted << std::endl;
    std::cout << "QR Found: " << m_qrFound << std::endl;
    std::cout << "Final Text: '" << m_lastQRText << "'" << std::endl;

    auto qrTab = GetQRDataTab();

    double totalElapsed = m_timer.Get();

    // Final status update
    qrTab->PutBoolean("Command Interrupted", interrupted);
    qrTab->PutNumber("Total Elapsed Time", totalElapsed);

    if (m_qrFound)
    {
        std::cout << "✅ QR Code reading SUCCESS: '" << m_lastQRText << "'" << std::endl;
        qrTab->PutString("Final Result", m_lastQRText);
        qrTab->PutString("Status", "✅ Complete - Code Found");
        qrTab->PutBoolean("Success", true);
    }
    else
    {
        std::cout << "❌ QR Code reading FAILED - No stable code found" << std::endl;
        qrTab->PutString("Final Result", "NOT_FOUND");
        qrTab->PutString("Status", "❌ Complete - No Code Found");
        qrTab->PutBoolean("Success", false);

        // Add failure reason
        if (interrupted)
        {
            qrTab->PutString("Failure Reason", "Command Interrupted");
        }
        else if (m_mode == ReadMode::TIMED_READ && totalElapsed >= m_timeout)
        {
            qrTab->PutString("Failure Reason", "Timeout Elapsed");
        }
        else
        {
            qrTab->PutString("Failure Reason", "No Stable Detection");
        }
    }

    // Summary statistics
    qrTab->PutBoolean("Session Complete", true);
    qrTab->PutString("End Timestamp", std::to_string(frc::Timer::GetFPGATimestamp()));

    m_timer.Stop();
}

bool QRCodeReaderCommand::IsFinished()
{
    switch (m_mode)
    {
    case ReadMode::SINGLE_READ:
        return m_qrFound;

    case ReadMode::CONTINUOUS_READ:
        return false;

    case ReadMode::TIMED_READ:
        return (m_timer.Get() >= m_timeout) || m_qrFound;

    default:
        return true;
    }
}