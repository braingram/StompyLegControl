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

    unsigned int read_adc();
    
    unsigned int get_adc_value();

  protected:
    unsigned int _adc_value;

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
    StringPot(int pin, ADC* adc, int adc_number, unsigned int adc_min, unsigned int adc_max, float length_min, float length_max);

    // read and return
    float read_length();

    // don't read, just return
    float get_length();

    float adc_value_to_length(unsigned int adc_value);
    unsigned int length_to_adc_value(float length);

    void set_adc_min(float value);
    void set_adc_max(float value);
    void set_adc_range(float min_value, float max_value);

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
    PressureSensor(int pin, ADC* adc, int adc_number, unsigned int adc_min, unsigned int adc_max, float pressure_min, float pressure_max);

    float read_pressure();

    float get_pressure();

  protected:
    Transform* _transform;
    float _pressure;
};


/* ========================================================
 *                     JoystickAxis
 * ========================================================*/

class JoystickAxis : public AnalogSensor {
  public:
    JoystickAxis(int pin, ADC* adc, int adc_number, Transform* transform);
    JoystickAxis(int pin, ADC* adc, int adc_number, unsigned int adc_min, unsigned int adc_deadband_min, unsigned int adc_deadband_max, unsigned int adc_max, float axis_min, float axis_mid, float axis_max);

    float read_axis();

    float get_axis();
  protected:
    Transform* _transform;
    float _axis;
};


// TODO force sensor
#endif
