#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Joystick.h>

class AMCU;

class OI : public frc2::SubsystemBase
{
public:
    OI();

    frc::Joystick& GetDriveJoystick();

    // Axes
    double GetRightDriveY() const;
    double GetRightDriveX() const;
    double GetLeftDriveY()  const;
    double GetLeftDriveX()  const;

    // Buttons
    bool GetDriveRightTrigger()     const;
    bool GetDriveRightShoulder()    const;
    bool GetDriveLeftTrigger()      const; // fixed casing
    bool GetDriveLeftShoulder()     const;
    bool GetDriveAButton()          const;
    bool GetDriveBButton()          const;
    bool GetDriveXButton()          const;
    bool GetDriveYButton()          const;
    bool GetDriveStartButton()      const;
    bool GetDriveBackSelectButton() const;
    bool GetDriveRightStickPress()  const;
    bool GetDriveLeftStickPress()   const;
    bool GetDriveHomeGuideButton()  const;
    bool GetDriveExtraButton()      const;

    // Button Map
    static constexpr int A_BUTTON = 1;
    static constexpr int B_BUTTON = 2;
    static constexpr int X_BUTTON = 3;
    static constexpr int Y_BUTTON = 4;
    static constexpr int LEFT_SHOULDER = 5;
    static constexpr int RIGHT_SHOULDER = 6;
    static constexpr int LEFT_TRIGGER = 7;   // verify on your device
    static constexpr int RIGHT_TRIGGER = 8;  // verify on your device
    static constexpr int BACK_SELECT_BUTTON = 9;
    static constexpr int START_BUTTON = 10;
    static constexpr int LEFT_STICK_PRESS = 11;
    static constexpr int RIGHT_STICK_PRESS = 12;
    static constexpr int HOME_GUIDE_BUTTON = 13;
    static constexpr int EXTRA_BUTTON = 14;

    // Axis Map
    static constexpr int LEFT_ANALOG_X  = 0;
    static constexpr int LEFT_ANALOG_Y  = 1;
    static constexpr int RIGHT_ANALOG_X = 2;
    static constexpr int RIGHT_ANALOG_Y = 3;

private:
    frc::Joystick m_driveJoystick;
    static constexpr int DRIVE_USB_PORT = 0;
};
