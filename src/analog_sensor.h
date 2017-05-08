/*
	analog_sensor.h - Library for reading an analog sensor.
	Created by Brett Graham, May 7, 2017.
	Released into the public domain -- so help you God.
*/


#ifndef ANALOG_SENSOR_H
#define ANALOG_SENSOR_H

#include "Arduino.h"

/*
 * An ADC instance is used to read the analog sensors
 * the AnalogSensor class will not do anything to configure
 * this ADC instance. So if you want HW averaging, etc
 * this must be configured elsewhere.
 */
#include <ADC.h>


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

/*
class LinearAnalogSensor : public AnalogSensor {
  public:
    LinearAnalogSensor(int pin, ADC*adc, int adc_number, int adc_min, int adc_max, int process_min, int process_max);

    float read_process();
    float get_process();

    void set_adc_min(int adc_min);
    int get_adc_min();

    void set_adc_max(int adc_max);
    int get_adc_max();

    void set_process_min(float process_min);
    float get_process_min();

    void set_process_max(float process_max);
    float get_process_max();

    float adc_value_to_process(int adc_value);
    int process_to_adc_value(float process);

  private:
    int _adc_min;
    int _adc_max;

    float _process_value;
    float _process_min;
    float _process_max;

    float _process_to_adc_ratio;

    void compute_process_to_adc_ratio();  // TODO make public?

};
*/
#endif
