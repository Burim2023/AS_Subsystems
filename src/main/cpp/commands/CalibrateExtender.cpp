#include "commands/CalibrateExtender.h"
#include <iostream>

CalibrateExtender::CalibrateExtender(ExtenderSubsystem* extender) 
    : m_extender(extender), m_state(CalibrationState::INITIAL), 
      m_startTime(0.0), m_calibrationComplete(false) {
    
    SetName("CalibrateExtender");
    AddRequirements(m_extender);
    
    std::cout << "CalibrateExtender: Command created" << std::endl;
}

void CalibrateExtender::Initialize() {
    std::cout << "=== CalibrateExtender::Initialize() ===" << std::endl;
    
    m_timer.Reset();
    m_timer.Start();
    m_state = CalibrationState::INITIAL;
    m_calibrationComplete = false;
    
    // Reset any existing calibration
    m_extender->ResetCalibration();
    
    std::cout << "CalibrateExtender: Starting calibration process..." << std::endl;
}

void CalibrateExtender::Execute() {
    //m_extender->ExtenderSubsystemCurrentState();
    bool frontPressed = m_extender->IsFrontLimitPressed();
    bool backPressed = m_extender->IsBackLimitPressed();
    
    switch (m_state) {
        case CalibrationState::INITIAL:
            std::cout << "CalibrateExtender: INITIAL - Moving to back limit" << std::endl;
            m_extender->SetRetractState();  // Move toward back limit
            m_state = CalibrationState::MOVING_TO_BACK;
            break;
            
        case CalibrationState::MOVING_TO_BACK:
            if (backPressed) {
                std::cout << "CalibrateExtender: Back limit reached - Stopping" << std::endl;
                m_extender->SetStopState();
                m_state = CalibrationState::AT_BACK_LIMIT;
            }
            // Continue moving back until limit is hit
            break;
            
        case CalibrationState::AT_BACK_LIMIT:
            std::cout << "CalibrateExtender: Starting timing measurement - Moving to front" << std::endl;
            
            // Reset timer and start timing the back-to-front travel
            m_timer.Reset();
            m_timer.Start();
            m_startTime = m_timer.Get();

            //frc2::WaitCommand(0.5_s);
            
            // Start moving toward front limit
            m_extender->SetExtendState();
            m_state = CalibrationState::TIMING_TO_FRONT;
            break;
            
        case CalibrationState::TIMING_TO_FRONT:
            if (frontPressed) {
                // Front limit reached - calculate travel time
                double travelTime = m_timer.Get() - m_startTime;
                
                std::cout << "CalibrateExtender: Front limit reached!" << std::endl;
                std::cout << "CalibrateExtender: Travel time = " << travelTime << " seconds" << std::endl;
                
                // Stop movement
                m_extender->SetStopState();
                
                // Save the calibration time
                m_extender->SetMaxTimeFrontToBack(travelTime);
                
                m_state = CalibrationState::CALIBRATION_DONE;
                m_calibrationComplete = true;
                
                std::cout << "CalibrateExtender: CALIBRATION COMPLETE - MaxTime = " << travelTime << "s" << std::endl;
            }
            break;
            
        case CalibrationState::CALIBRATION_DONE:
            // Do nothing - wait for IsFinished()
            break;
    }
}

bool CalibrateExtender::IsFinished() {
    // Fix: Use Get() instead of HasElapsed() for WPILib 2020
    if (m_timer.Get() > 30.0) {  // 30 second timeout
        std::cout << "CalibrateExtender: TIMEOUT - Calibration failed" << std::endl;
        return true;
    }
    
    return m_calibrationComplete;
}

void CalibrateExtender::End(bool interrupted) {
    m_extender->SetStopState();
    
    if (interrupted) {
        std::cout << "CalibrateExtender: INTERRUPTED - Calibration incomplete" << std::endl;
    } else {
        // Fix: Use correct method name (uppercase)
        double maxTime = m_extender->GetMaxTimeFrontToBack();
        std::cout << "CalibrateExtender: SUCCESS - MaxTimeFrontToBack = " << maxTime << "s" << std::endl;
    }
}