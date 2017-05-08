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


class StringPot {
  public:
    StringPot(int pin, int adc_min, int adc_max, float length_min, float length_max, ADC* adc, int adc_number);

    // read and return
    int read_adc();
    float read_length();

    // don't read, just return
    int get_adc_value();
    float get_length();

    void set_adc_min(int adc_min);
    int get_adc_min();

    void set_adc_max(int adc_max);
    int get_adc_max();

    float get_length_min();
    float get_length_max();

    float adc_value_to_length(int adc_value);
    int length_to_adc_value(float length);


  private:
    ADC* _adc;
    int _adc_number;
    int _pin;

    int _adc_value;
    int _adc_min;
    int _adc_max;

    float _length;
    float _length_min;
    float _length_max;

    float _length_to_adc_ratio;

    void compute_length_to_adc_ratio();  // TODO make public?
};

#endif
