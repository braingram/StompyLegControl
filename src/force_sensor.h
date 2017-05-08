/*
	force_sensor.h - Library for reading the calf force sensor.
	Created by Brett Graham, May 8, 2017.
	Released into the public domain -- so help you God.
*/


#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include "Arduino.h"

/*
 * An ADC instance is used to read the string pots
 * the ForceSensor class will not do anything to configure
 * this ADC instance. So if you want HW averaging, etc
 * this must be configured elsewhere.
 */
#include <ADC.h>


class ForceSensor {
  public:
    ForceSensor(int pin, int adc_min, int adc_max, ADC* adc, int adc_number);

  private:
    ADC* _adc;
    int _adc_number;
    int _pin;
};
