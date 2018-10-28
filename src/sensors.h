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
#include "defaults.h"
#include "transforms.h"
#include "stompy_pins.h"

#define STRING_POT_READY 1
#define STRING_POT_NO_SAMPLE 0


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
 *                 Filtered Analog Sensor
 * ========================================================*/

class FilteredAnalogSensor : public AnalogSensor {
  public:
    FilteredAnalogSensor(int pin, ADC* adc, int adc_number);

    void sample();  // take a raw sample of the current voltage
    void filter();  // filter all current samples

    unsigned int read_adc();  // filter samples and return value

    unsigned int get_adc_value();  // return pre-filtered value

    int get_index();

  protected:
    unsigned int _adc_value;

  private:
    ADC* _adc;
    int _adc_number;
    int _pin;

    int _samples[N_FILTER_SAMPLES];
    int _sample_index;
};


/* ========================================================
 *                     StringPot Sensor
 * ========================================================*/

class StringPot {
  public:
    StringPot(FilteredAnalogSensor* analog_sensor, Transform* transform);
    StringPot(
      FilteredAnalogSensor* analog_sensor,
      unsigned int adc_min, unsigned int adc_max,
      float length_min, float length_max);

    // read and return
    float read_length();
    unsigned int read_adc();

    // don't read, just return
    float get_length();
    unsigned int get_adc_value();

    float adc_value_to_length(unsigned int adc_value);
    unsigned int length_to_adc_value(float length);

    void set_adc_min(float value);
    float get_adc_min();
    void set_adc_max(float value);
    float get_adc_max();
    void set_adc_range(float min_value, float max_value);

    void set_length_min(float value);
    float get_length_min();
    void set_length_max(float value);
    float get_length_max();
    void set_length_range(float min_value, float max_value);

    bool adc_in_range();

    int update();

  protected:
    Transform* _transform;
    FilteredAnalogSensor* _analog_sensor;
    //float _length; // TODO cache length?
    //elapsedMillis _sample_timer;  // run at 500 Hz, 2 ms
    elapsedMicros _sample_timer;  // run at 500 Hz, 2 ms
    // int _sample_count;
    // elapsedMillis _filter_timer;  // run at 50 Hz, 20 ms
};


/* ========================================================
 *                     CalfSensor
 * ========================================================*/

class CalfSensor {
  public:
    CalfSensor(
      FilteredAnalogSensor* analog_sensor, CalfLoadTransform* transform);

    // read and return
    float read_load();
    unsigned int read_adc();

    // don't read, just return
    float get_load();
    unsigned int get_adc_value();

    float adc_value_to_load(unsigned int adc_value);
    unsigned int load_to_adc_value(float length);

    int update();

  protected:
    CalfLoadTransform* _transform;
    FilteredAnalogSensor* _analog_sensor;
    //float _load; // TODO cache load?
    //elapsedMillis _sample_timer;  // run at 500 Hz, 2 ms
    elapsedMicros _sample_timer;  // run at 500 Hz, 2 ms
    // int _sample_count;
};



/*

class CalfSensor {
  public:
    CalfSensor(AnalogSensor* analog_sensor, Transform* transform);
    CalfSensor(AnalogSensor* analog_sensor, unsigned int adc_min, unsigned int adc_max, float length_min, float length_max);

    // read and return
    float read_length();
    unsigned int read_adc();

    // don't read, just return
    float get_length();
    unsigned int get_adc_value();

    float adc_value_to_length(unsigned int adc_value);
    unsigned int length_to_adc_value(float length);

    void set_adc_min(float value);
    void set_adc_max(float value);
    void set_adc_range(float min_value, float max_value);

    int update();

  protected:
    Transform* _transform;
    FilteredAnalogSensor* _analog_sensor;
    float _length;
    elapsedMillis _sample_timer;  // run at 500 Hz, 2 ms
    int _sample_count;
    // elapsedMillis _filter_timer;  // run at 50 Hz, 20 ms
};
*/

/*
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
*/


/* ========================================================
 *                     JoystickAxis
 * ========================================================*/
/*
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
*/

#endif
