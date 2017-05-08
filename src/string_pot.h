/*
	string_pot.h - Library for reading a string pot.
	Created by Brett Graham, May 7, 2017.
	Released into the public domain -- so help you God.
*/


#ifndef STRING_POT_H
#define STRING_POT_H

#include "Arduino.h"

/*
 * An ADC instance is used to read the string pots
 * the StringPot class will not do anything to configure
 * this ADC instance. So if you want HW averaging, etc
 * this must be configured elsewhere.
 */
#include <ADC.h>
#include "sensors.h"
#include "transforms.h"


class StringPot {
  public:
    StringPot(AnalogSensor* sensor, LinearTransform* transform);

    // read and return
    int read_adc();
    float read_length();

    // don't read, just return
    int get_adc_value();
    float get_length();

    float adc_value_to_length(int adc_value);
    int length_to_adc_value(float length);

    AnalogSensor* sensor;
    LinearTransform* transform;

  private:
    int _adc_value;
    float _length;
};

#endif
