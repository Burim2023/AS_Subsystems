#include "gamepad/OI.h"
#include "AMCU.h"
#include <cmath>

// Constructor - initialize the joystick
OI::OI() : m_driveJoystick(DRIVE_USB_PORT) {
    // Constructor initializes the joystick on the specified USB port
}

// ADD THIS FUNCTION DEFINITION
frc::Joystick& OI::GetDriveJoystick() {
    return m_driveJoystick;
}

// Axis methods
double OI::GetRightDriveY() const {
    return -m_driveJoystick.GetRawAxis(RIGHT_ANALOG_Y);
}

double OI::GetRightDriveX() const {
    return m_driveJoystick.GetRawAxis(RIGHT_ANALOG_X);
}

double OI::GetLeftDriveY() const {
    return -m_driveJoystick.GetRawAxis(LEFT_ANALOG_Y);
}

double OI::GetLeftDriveX() const {
    return m_driveJoystick.GetRawAxis(LEFT_ANALOG_X);
}


// Button methods - updated to use renamed constants
bool OI::GetDriveRightTrigger() const {
    return m_driveJoystick.GetRawButton(RIGHT_TRIGGER);
}

bool OI::GetDriveRightShoulder() const {
    return m_driveJoystick.GetRawButton(RIGHT_SHOULDER);
}

bool OI::GetDriveLeftTrigger() const {
    return m_driveJoystick.GetRawButton(LEFT_TRIGGER);
}

bool OI::GetDriveLeftShoulder() const {
    return m_driveJoystick.GetRawButton(LEFT_SHOULDER);
}

bool OI::GetDriveAButton() const {
    return m_driveJoystick.GetRawButton(A_BUTTON);
}

bool OI::GetDriveBButton() const {
    return m_driveJoystick.GetRawButton(B_BUTTON);
}

bool OI::GetDriveXButton() const {
    return m_driveJoystick.GetRawButton(X_BUTTON);
}

bool OI::GetDriveYButton() const {
    return m_driveJoystick.GetRawButton(Y_BUTTON);
}

bool OI::GetDriveStartButton() const {
    return m_driveJoystick.GetRawButton(START_BUTTON);
}

bool OI::GetDriveBackSelectButton() const {
    return m_driveJoystick.GetRawButton(BACK_SELECT_BUTTON);
}

bool OI::GetDriveRightStickPress() const {
    return m_driveJoystick.GetRawButton(RIGHT_STICK_PRESS);
}

bool OI::GetDriveLeftStickPress() const {
    return m_driveJoystick.GetRawButton(LEFT_STICK_PRESS);
}

bool OI::GetDriveHomeGuideButton() const {
    return m_driveJoystick.GetRawButton(HOME_GUIDE_BUTTON);
}

bool OI::GetDriveExtraButton() const {
    return m_driveJoystick.GetRawButton(EXTRA_BUTTON);
}
