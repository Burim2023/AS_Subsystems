#pragma once

#include <frc2/command/SubsystemBase.h>

class Gamepad : public frc2::SubsystemBase {
public:
    // Joystick Axes
    double GetLeftStickX(void);   
    double GetLeftStickY(void);   

    double GetRightStickY(void);  
    double GetRightStickX(void);

    bool GetLeftBumper(void);        
    bool GetRightBumper(void);       

    bool GetLeftTrigger(void);              
    bool GetRightTrigger(void);             

    bool GetAButton(void);           
    bool GetXButton(void);           
    bool GetYButton(void);           
    bool GetBButton(void);           

    bool GetStickLeftButton(void); 
    bool GetStickRightButton(void); 

    bool GetBackButton(void);
    bool GetStartButton(void);


    // D-pad / POV
    int GetPOV(void);                     // returns -1 or angle in degrees
    bool IsDPadUp(void);
    bool IsDPadRight(void);
    bool IsDPadDown(void);
    bool IsDPadLeft(void);

    void Periodic() override;

public:
    static constexpr int DRIVE_USB_PORT = 0;

private:

    // Joystick Axis
    static constexpr int LEFT_ANALOG_X   = 0;
    static constexpr int LEFT_ANALOG_Y   = 1;

    static constexpr int RIGHT_ANALOG_X  = 2;
    static constexpr int RIGHT_ANALOG_Y  = 3;

    // Button Map
    static constexpr int A_BUTTON            = 1;
    static constexpr int B_BUTTON            = 2;
    static constexpr int X_BUTTON            = 3;
    static constexpr int Y_BUTTON            = 4;

    static constexpr int LEFT_BUMPER         = 5;
    static constexpr int RIGHT_BUMPER        = 6;
    static constexpr int LEFT_TRIGGER        = 7;
    static constexpr int RIGHT_TRIGGER       = 8;
    static constexpr int BACK_BUTTON         = 9;
    static constexpr int START_BUTTON        = 10;
    static constexpr int LEFT_STICK_BUTTON   = 11;
    static constexpr int RIGHT_STICK_BUTTON  = 12;
};