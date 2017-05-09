/*
	sensors.h - Library for reading sensors.
	Created by Brett Graham, May 7, 2017.
	Released into the public domain -- so help you God.
*/


#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"

/*
 * An ADC instance is used to read the analog sensors
 * the class will not do anything to configure
 * this ADC instance. So if you want HW averaging, etc
 * this must be configured elsewhere.
 */
#include <ADC.h>
#include "transforms.h"

/* ========================================================
 *                      Analog Sensor
 * A class to handle reading a sensor value from an adc instance
 * ========================================================*/

class AnalogSensor {
  public:
    AnalogSensor(int pin, ADC* adc, int adc_number);

    int read_adc();
    
    int get_adc_value();

  protected:
    int _adc_value;

  private:
    ADC* _adc;
    int _adc_number;
    int _pin;
};


/* ========================================================
 *                     StringPot Sensor
 * ========================================================*/

class StringPot : public AnalogSensor {
  public:
    StringPot(int pin, ADC* adc, int adc_number, Transform* transform);
    StringPot(int pin, ADC* adc, int adc_number, int adc_min, int adc_max, float length_min, float length_max);

    // read and return
    float read_length();

    // don't read, just return
    float get_length();

    float adc_value_to_length(int adc_value);
    int length_to_adc_value(float length);

  protected:
    Transform* _transform;
    float _length;
};


/* ========================================================
 *                     PressureSensor
 * ========================================================*/

class PressureSensor : public AnalogSensor {
  public:
    PressureSensor(int pin, ADC* adc, int adc_number, Transform* transform);
    PressureSensor(int pin, ADC* adc, int adc_number, int adc_min, int adc_max, float pressure_min, float pressure_max);

    float read_pressure();

    float get_pressure();

  protected:
    Transform* _transform;
    float _pressure;
};


/* ========================================================
 *                     PressureSensor
 * ========================================================*/

class JoystickAxis : public AnalogSensor {
  public:
    JoystickAxis(int pin, ADC* adc, int adc_number, Transform* transform);
    JoystickAxis(int pin, ADC* adc, int adc_number, int adc_min, int adc_deadband_min, int adc_deadband_max, int adc_max, float axis_min, float axis_mid, float axis_max);

    float read_axis();

    float get_axis();
  protected:
    Transform* _transform;
    float _axis;
};
// TODO force sensor
#endif
