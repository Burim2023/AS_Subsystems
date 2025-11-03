#include "subsystems/joystick/Gamepad.h"
#include <networktables/NetworkTableInstance.h>
#include <frc/Joystick.h>
#include <cmath>

using namespace frc;

static Joystick drivePad{Gamepad::DRIVE_USB_PORT};

/**
 * Helper function to apply deadband.
 */
static double ApplyDeadband(double value, double deadband = 0.05) {
    return (std::fabs(value) < deadband) ? 0.0 : value;
}

// === Joystick Axis Accessors ===
double Gamepad::GetLeftStickX()  { return ApplyDeadband(drivePad.GetRawAxis(LEFT_ANALOG_X)); }
double Gamepad::GetLeftStickY()  { return ApplyDeadband(drivePad.GetRawAxis(LEFT_ANALOG_Y)); }

double Gamepad::GetRightStickY() { return ApplyDeadband(drivePad.GetRawAxis(RIGHT_ANALOG_Y)); }
double Gamepad::GetRightStickX()  { return drivePad.GetRawAxis(RIGHT_ANALOG_X); }


// === Button Accessors ===
bool Gamepad::GetLeftBumper()        { return drivePad.GetRawButton(LEFT_BUMPER); }
bool Gamepad::GetRightBumper()       { return drivePad.GetRawButton(RIGHT_BUMPER); }
bool Gamepad::GetLeftTrigger()              { return drivePad.GetRawButton(LEFT_TRIGGER); }
bool Gamepad::GetRightTrigger()             { return drivePad.GetRawButton(RIGHT_TRIGGER); }

bool Gamepad::GetAButton()           { return drivePad.GetRawButton(A_BUTTON); }
bool Gamepad::GetXButton()           { return drivePad.GetRawButton(X_BUTTON); }
bool Gamepad::GetYButton()           { return drivePad.GetRawButton(Y_BUTTON); }
bool Gamepad::GetBButton()           { return drivePad.GetRawButton(B_BUTTON); }

bool Gamepad::GetStickLeftButton()        { return drivePad.GetRawButton(LEFT_STICK_BUTTON); }
bool Gamepad::GetStickRightButton()       { return drivePad.GetRawButton(RIGHT_STICK_BUTTON); }

bool Gamepad::GetBackButton()        { return drivePad.GetRawButton(BACK_BUTTON); }
bool Gamepad::GetStartButton()       { return drivePad.GetRawButton(START_BUTTON); }


// POV for the DPAD
int Gamepad::GetPOV() { return drivePad.GetPOV(); }

bool Gamepad::IsDPadUp()    { return GetPOV() == 0; }
bool Gamepad::IsDPadRight() { return GetPOV() == 90; }
bool Gamepad::IsDPadDown()  { return GetPOV() == 180; }
bool Gamepad::IsDPadLeft()  { return GetPOV() == 270; }

// Update the Dashboard 
void Gamepad::Periodic() {

    auto controllerTable = nt::NetworkTableInstance::GetDefault().GetTable("Controller");

    // Publish primary sticks, triggers, and POV
    controllerTable->PutNumber("LeftX", GetLeftStickX());
    controllerTable->PutNumber("LeftY", GetLeftStickY());

    controllerTable->PutNumber("RightY", GetRightStickY());
    controllerTable->PutNumber("RightX", GetRightStickX());

    controllerTable->PutNumber("DPAD POV", GetPOV());

    // Buttons (1..16)
    controllerTable->PutNumber("A", GetAButton());
    controllerTable->PutNumber("B", GetBButton());
    controllerTable->PutNumber("X", GetXButton());
    controllerTable->PutNumber("Y", GetYButton());

    controllerTable->PutNumber("Left Bumper", GetLeftBumper());
    controllerTable->PutNumber("Right Bumper", GetRightBumper());

    controllerTable->PutNumber("Left Trigger", GetLeftTrigger());
    controllerTable->PutNumber("Right Trigger", GetRightTrigger());

    controllerTable->PutNumber("Left Stick Button", GetStickLeftButton());
    controllerTable->PutNumber("Right Stick Button", GetStickRightButton());

    controllerTable->PutNumber("Back Button", GetBackButton());
    controllerTable->PutNumber("Start Button", GetStartButton());

}