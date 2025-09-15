#include "gamepad/OI.h"
#include <frc/Joystick.h>

// The joystick object for the driver controller
static frc::Joystick driveJoystick(DRIVE_USB_PORT);

// Axis methods
double OI::GetRightDriveY(void) {
    return driveJoystick.GetRawAxis(RIGHT_ANALOG_Y);
}

double OI::GetRightDriveX(void) {
    return driveJoystick.GetRawAxis(RIGHT_ANALOG_X);
}

double OI::GetLeftDriveY(void) {
    return driveJoystick.GetRawAxis(LEFT_ANALOG_Y);
}

double OI::GetLeftDriveX(void) {
    return driveJoystick.GetRawAxis(LEFT_ANALOG_X);
}

// Button methods
bool OI::GetDriveRightTrigger(void) {
    return driveJoystick.GetRawButton(RIGHT_TRIGGER);
}

bool OI::GetDriveRightBumper(void) {
    return driveJoystick.GetRawButton(RIGHT_BUMPER);
}

bool OI::getDriveLeftTrigger(void) {
    return driveJoystick.GetRawButton(LEFT_TRIGGER);
}

bool OI::GetDriveLeftBumper(void) {
    return driveJoystick.GetRawButton(LEFT_BUMPER);
}

bool OI::GetDriveXButton(void) {
    return driveJoystick.GetRawButton(X_BUTTON);
}

bool OI::GetDriveSquareButton(void) {
    return driveJoystick.GetRawButton(SQUARE_BUTTON);
}

bool OI::GetDriveCircleButton(void) {
    return driveJoystick.GetRawButton(CIRCLE_BUTTON);
}

bool OI::GetDriveTriangleButton(void) {
    return driveJoystick.GetRawButton(TRIANGLE_BUTTON);
}

bool OI::GetDriveOptionsButton(void) {
    return driveJoystick.GetRawButton(OPTIONS_BUTTON);
}

bool OI::GetDriveShareButton(void) {
    return driveJoystick.GetRawButton(SHARE_BUTTON);
}

bool OI::GetDriveRightAnalogButton(void) {
    return driveJoystick.GetRawButton(RIGHT_ANALOG_BUTTON);
}

bool OI::GetDriveLeftAnalogButton(void) {
    return driveJoystick.GetRawButton(LEFT_ANALOG_BUTTON);
}

bool OI::GetDrivePS4Button(void) {
    return driveJoystick.GetRawButton(PS4_BUTTON);
}

bool OI::GetDriveTouchpadButton(void) {
    return driveJoystick.GetRawButton(TOUCHPAD_BUTTON);
}
