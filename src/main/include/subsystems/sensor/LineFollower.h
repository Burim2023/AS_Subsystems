#pragma once
#include "studica/Cobra.h"


class LineFollower {
public:
    LineFollower(float voltage = 5.0F);

    // Wird in init() aufgerufen
    void init();

    // Wird in robotPeriodic() aufgerufen
    void update();

    // Gibt zurück, ob die Linie erkannt wurde
    bool isLineDetected() const;

    // Optional: Zugriff auf Rohwert und Spannung
    //int getRawValue() const;
    float getVoltage() const;

private:
    studica::Cobra cobra;
    int sensorChannel = 0; // Der einzige Kanal, z. B. ADC0
    float sensorVoltage = 0.0F;
    const float LINE_THRESHOLD = 1.5F;
};