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

#endif
