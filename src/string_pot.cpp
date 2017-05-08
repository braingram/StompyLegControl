#include "string_pot.h"

StringPot::StringPot(ADC* adc, int pin, int adc_min, int adc_max, float length_min, float length_max) {
  _adc = adc;
  _pin = pin;
  _adc_min = adc_min;
  _adc_max = adc_max;
  _length_min = length_min;
  _length_max = length_max;

  compute_length_to_adc_ratio();

  // default with value to min before first read
  // TODO should this just read instead?
  _adc_value = _adc_min;
  _length = adc_value_to_length(_adc_value);
}

void StringPot::compute_length_to_adc_ratio() {
  _length_to_adc_ratio = (_length_max - _length_min) / (_adc_max - _adc_min);
}

int StringPot::read_adc() {
  _adc_value = _adc->analogRead(_pin);
  _length = adc_value_to_length(_adc_value);
  return _adc_value;
}

float StringPot::read_length() {
  read_adc();
  return _length;
}

int StringPot::get_adc_value() {
  return _adc_value;
}

float StringPot::get_length() {
  return _length;
}

void StringPot::set_adc_min(int adc_min) {
  _adc_min = adc_min;
}

int StringPot::get_adc_min() {
  return _adc_min;
}

void StringPot::set_adc_max(int adc_max) {
  _adc_max = adc_max;
}

int StringPot::get_adc_max() {
  return _adc_max;
}

float StringPot::get_length_min() {
  return _length_min;
}

float StringPot::get_length_max() {
  return _length_max;
}

float StringPot::adc_value_to_length(int adc_value) {
  if (adc_value <= _adc_min) {
    return _length_min;
  } else if (adc_value >= _adc_max) {
    return _length_max;
  } else {
    return (adc_value - _adc_min) * _length_to_adc_ratio + _length_min;
  }
}

int StringPot::length_to_adc_value(float length) {
  if (length <= _length_min) {
    return _adc_min;
  } else if (length >= _length_max) {
    return _adc_max;
  } else {
    return (_length - _length_min) / _length_to_adc_ratio + _adc_min;
  }
}
