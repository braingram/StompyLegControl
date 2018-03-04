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
    Valve(int extendPin, int retractPin, int enablePin, int disablePin, float frequency, int resolution);
    Valve(int extendPin, int retractPin, int enablePin, int disablePin, float frequency);
    Valve(int extendPin, int retractPin, int enablePin, int disablePin);
    void enable();
    void disable();

    // allow setting of pwm
    void set_pwm(int pwm);
    void extend_pwm(int pwm);
    void retract_pwm(int pwm);
    void stop();

    // and setting by ratio (value from 0 to 1)
    void set_pwm_limits(
    int extend_min, int extend_max, int retract_min, int retract_max);
    void set_extend_pwm_max(int max);
    int get_extend_pwm_max();
    void set_extend_pwm_min(int min);
    int get_extend_pwm_min();
    void set_retract_pwm_max(int max);
    int get_retract_pwm_max();
    void set_retract_pwm_min(int min);
    int get_retract_pwm_min();


    void set_ratio(float ratio);
    void extend_ratio(float ratio);
    void retract_ratio(float ratio);

    // allow getting of state
    bool get_enabled();
    int get_pwm();
    int get_direction();
    float get_ratio();

  private:
    // state
    int _pwm;
    int _direction;
    bool _enabled;

    // pins
    int _extendPin;
    int _retractPin;
    int _enablePin;
    int _disablePin;

    int _extend_pwm_min;
    int _extend_pwm_max;
    int _retract_pwm_min;
    int _retract_pwm_max;
};

#endif
