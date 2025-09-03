#ifndef CONTROLER_H
#define CONTROLER_H

#include <Arduino.h>
#include "config.h"

class Controler {
public:
    Controler();
    void update();
    int get_elevation() const;
    int get_aileron() const;
    bool unmanned_status() const;
    bool checkUnmanned();
    int get_control_value() const;
    int get_com_rudder() const;
    int get_com_sail() const;
    int get_scenario_number();
private:
    const int controlPin = 23;
    const int elevationPin = 3;
    const int aileronPin = 2;
    int elevation, aileron, comSail, comRud, controlValue;
    bool unmanned = true;
};

#endif // CONTROLER_H
