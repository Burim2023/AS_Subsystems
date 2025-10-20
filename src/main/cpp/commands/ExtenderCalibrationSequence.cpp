#include "commands/ExtenderCalibrationSequence.h"
#include <iostream>
#include <memory>  // Add this include

ExtenderCalibrationSequence::ExtenderCalibrationSequence(ExtenderSubsystem* extender, double pauseBetweenSteps) 
    : m_extender(extender), m_pauseTime(pauseBetweenSteps) {
    
    SetName("ExtenderCalibrationSequence");
    
    std::cout << "ExtenderCalibrationSequence: Building complete calibration and demo sequence" << std::endl;
    
    AddCommands(
        // Fix: Create CalibrateExtender as a temporary object properly
        std::move(CalibrateExtender(extender)),         // Proper calibration
        frc2::WaitCommand(2.0_s),                       
        
                            
        
        ExtendForDuration(extender, ExtendForDuration::Direction::RETRACT, 0.25, 2.0),
        frc2::WaitCommand(3.0_s),                       
        
        ExtendForDuration(extender, ExtendForDuration::Direction::RETRACT, 0.25, 2.0),
        frc2::WaitCommand(3.0_s),                       
        
        ExtendForDuration(extender, ExtendForDuration::Direction::RETRACT, 0.25, 2.0),
        frc2::WaitCommand(2.0_s),

        ExtendForDuration(extender, ExtendForDuration::Direction::EXTEND, 0.5, 2.0)

    );
    
    std::cout << "ExtenderCalibrationSequence: Complete sequence built" << std::endl;
}