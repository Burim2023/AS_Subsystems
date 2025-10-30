/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "studica/Servo.h"
#include "AMCU.h"

namespace Constants
{
    // // logfile path
    // inline constexpr const char* FILE_PATH = "/home/lvuser/deploy/robot.log";
    // // widget logfile - matches what the website expects
    // inline constexpr const char* WIDGET_FILE_PATH = "/home/lvuser/deploy/status.json";
    //amcu constants
    inline constexpr int kWheelRadius = 50;
    inline constexpr int kRobotRadius = 162;

    inline constexpr Motor kMotorLeft = MOTOR_1;
    inline constexpr Motor kMotorRight = MOTOR_2;
    inline constexpr Motor kMotorBack = MOTOR_3;
    inline constexpr Motor kMotorElevator = MOTOR_0;

    static constexpr int kLeftTriggerPort = 0;
    static constexpr int kLeftEchoPort = 1;
    static constexpr int kRightTriggerPort = 2;
    static constexpr int kRightEchoPort = 3;

    // // ultra sonic right
    // inline constexpr int RIGHT_TRIG_PORT = 2;
    // inline constexpr int RIGHT_ECHO_PORT = 3;
    // // ultra sonic left
    // inline constexpr int LEFT_TRIG_PORT = 0;
    // inline constexpr int LEFT_ECHO_PORT = 1;

    // // sensor update rate
    // inline constexpr int SENSOR_UPDATE_RATE = 25;

    // // elevator
    // inline constexpr int GRIPPER_TILT_PORT = 10;
    // inline constexpr int GRIPPER_OPEN_PORT = 11;

    // inline constexpr int ELEVATOR_MOVE_PORT = 12;
    // inline constexpr int ELEVEVATOR_UPDOWN_PORT = 13;
    // inline constexpr int ELEVATOR_ROTATE_PORT = 14;

    // extender
    inline constexpr int EXTENDER_SERVO_PORT = 19;

    inline constexpr int LIMIT_SWITCH_STOP_BACK_PORT = 6;
    inline constexpr int LIMIT_SWITCH_STOP_FRONT_PORT = 4;

    // Swapped due to 1:1 gear ratio mechanical reversal
    inline constexpr double EXTENDER_DRIVE_FORWARD = 0.6;   // Was 0.0, now 1.0
    inline constexpr double EXTENDER_DRIVE_BACKWARD = 0.4;  // Was 1.0, now 0.0
    inline constexpr double EXTENDER_DRIVE_STOP = 0.5;

}
