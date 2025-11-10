#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/vision/CameraSubsystem.h"
#include <frc/Timer.h>
#include <string>

class QRCodeReaderCommand : public frc2::CommandHelper<frc2::CommandBase, QRCodeReaderCommand> {
public:
    enum class ReadMode {
        SINGLE_READ,        // Read once and finish
        CONTINUOUS_READ,    // Keep reading until interrupted
        TIMED_READ         // Read for specified duration
    };

    // Constructor for single read or continuous read
    QRCodeReaderCommand(CameraSubsystem* camera, ReadMode mode = ReadMode::SINGLE_READ);
    
    // Constructor for timed read
    QRCodeReaderCommand(CameraSubsystem* camera, double timeoutSeconds);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

    // Get the last read QR code text
    std::string GetQRCodeText() const { return m_lastQRText; }
    bool IsQRCodeFound() const { return m_qrFound; }

private:
    CameraSubsystem* m_camera;
    ReadMode m_mode;
    double m_timeout;
    frc::Timer m_timer;
    
    // Results
    std::string m_lastQRText;
    bool m_qrFound;
    
    // State tracking
    bool m_hasReadCode;
    int m_consecutiveReads;
    static constexpr int kMinConsecutiveReads = 3; // Require 3 consecutive reads for stability
};