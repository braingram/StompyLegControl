/*
	valve.h - Library for controlling a hydraulic valve.
	Created by Brett Graham, May 7, 2017.
	Released into the public domain -- so help you God.
*/


#ifndef VALVE_H
#define VALVE_H

#include "Arduino.h"

#define VALVE_EXTENDING 1
#define VALVE_RETRACTING -1
#define VALVE_STOPPED 0

#define VALVE_PWM_RESOLUTION 13
// optimal for 13 bit assuming 96 MHz Teensy 3.2
#define VALVE_PWM_FREQUENCY 5859.375


class Valve {
  public:
    Valve(int extendPin, int retractPin, int enablePin, float frequency, int resolution);
    Valve(int extendPin, int retractPin, int enablePin, float frequency);
    Valve(int extendPin, int retractPin, int enablePin);
    void enable();
    void disable();

    // allow setting of pwm
    void extend_pwm(int pwm);
    void retract_pwm(int pwm);
    void stop();

    // and setting by ratio (value from 0 to 1)
    void extend_ratio(float ratio);
    void retract_ratio(float ratio);

    // allow getting of state
    int get_pwm();
    int get_direction();

  private:
    // state
    int _pwm;
    int _direction;

    // pins
    int _extendPin;
    int _retractPin;
    int _enablePin;

    int _pwm_max;
};

#endif
