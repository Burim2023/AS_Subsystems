#pragma once

#include <frc2/command/SubsystemBase.h>

class OI : public frc2::SubsystemBase
{
    public: 
        double GetRightDriveY(void);
        double GetRightDriveX(void);
        double GetLeftDriveY(void);
        double GetLeftDriveX(void);
        bool GetDriveRightTrigger(void);
        bool GetDriveRightBumper(void);
        bool getDriveLeftTrigger(void);
        bool GetDriveLeftBumper(void);
        bool GetDriveXButton(void);
        bool GetDriveSquareButton(void);
        bool GetDriveCircleButton(void);
        bool GetDriveTriangleButton(void);
        bool GetDriveOptionsButton(void);
        bool GetDriveShareButton(void);
        bool GetDriveRightAnalogButton(void);
        bool GetDriveLeftAnalogButton(void);
        bool GetDrivePS4Button(void);
        bool GetDriveTouchpadButton(void);
    
    private:
        //Controller Port
        #define DRIVE_USB_PORT              0

        // Sudica Robotc Multi Controller PID WD801XM Button Map
        #define X_BUTTON                    1   // A button
        #define SQUARE_BUTTON               2   // B button  
        #define CIRCLE_BUTTON               3   // X button
        #define TRIANGLE_BUTTON             4   // Y button
        #define LEFT_BUMPER                 5   // Left shoulder
        #define RIGHT_BUMPER                6   // Right shoulder
        #define LEFT_TRIGGER                7   // Left trigger
        #define RIGHT_TRIGGER               8   // Right trigger
        #define SHARE_BUTTON                9   // Back/Select button
        #define OPTIONS_BUTTON              10  // Start button
        #define LEFT_ANALOG_BUTTON          11  // Left stick press
        #define RIGHT_ANALOG_BUTTON         12  // Right stick press
        #define PS4_BUTTON                  13  // Home/Guide button
        #define TOUCHPAD_BUTTON             14  // Extra button (if available)

        // Sudica Robotc Multi Controller PID WD801XM Joystick Map
        #define LEFT_ANALOG_X               0   // Left stick X-axis
        #define LEFT_ANALOG_Y               1   // Left stick Y-axis
        #define RIGHT_ANALOG_X              2   // Right stick X-axis  
        #define RIGHT_ANALOG_Y              3   // Right stick Y-axis (changed from 5 to 3)
        
        // Trigger axes (if analog triggers)
        #define LEFT_TRIGGER_AXIS           4   // Left trigger analog
        #define RIGHT_TRIGGER_AXIS          5   // Right trigger analog
};