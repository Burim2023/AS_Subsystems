#include "subsystems/LineFollower.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

LineFollower::LineFollower(float voltage) : cobra{voltage} {}

void LineFollower::init() {
    update();
    frc::SmartDashboard::PutString("Cobra volt:", std::to_string(sensorVoltage));
    std::cout<<"Cobra Voltage" << sensorVoltage << std::endl;
}

void LineFollower::update() {
    sensorVoltage = cobra.GetVoltage(sensorChannel);
    std::cout<<"Cobra Voltage" << sensorVoltage << std::endl;
}

bool LineFollower::isLineDetected() const {
    bool test;
    if (sensorVoltage < LINE_THRESHOLD) {
        test = true;
    } else {
        test = false;
    }

    frc::SmartDashboard::PutString("Linie erkannt", std::to_string(test));
    return sensorVoltage < LINE_THRESHOLD;
}

/*int LineFollower::getRawValue() const {
    return cobra.GetRawValue(sensorChannel);
}*/

float LineFollower::getVoltage() const {
    return sensorVoltage;
}