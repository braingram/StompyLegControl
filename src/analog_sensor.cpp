#include "analog_sensor.h"

AnalogSensor::AnalogSensor(int pin, ADC* adc, int adc_number) {
  _adc = adc;
  _adc_number = adc_number;
  _pin = pin;

  // initialize adc
  read_adc();
}

int AnalogSensor::read_adc() {
  _adc_value = _adc->analogRead(_pin, adc_number);
  return _adc_value;
}

int AnalogSensor::get_adc_value() {
  return _adc_value;
}

/*
LinearAnalogSensor::LinearAnalogSensor(int pin, ADC*adc, int adc_number, int adc_min, int adc_max, int process_min, int process_max) : AnalogSensor(pin, adc, adc_number) {
  _adc_min = adc_min;
  _adc_max = adc_max;
  _process_min = process_min;
  _process_max = process_max;

  compute_process_to_adc_ratio();

  _process_value = adc_value_to_process(_adc_value);
}

void LinearAnalogSensor::compute_process_to_adc_ratio() {
  _process_to_adc_ratio = (_process_max - _process_min) / (_adc_max - _adc_min);
}

float LinearAnalogSensor::read_process() {
  _process_value = adc_value_to_process(read_adc());
  return _process_value;
}

float LinearAnalogSensor::get_process() {
  return _process_value;
}

void LinearAnalogSensor::set_adc_min(int adc_min) {
  _adc_min = adc_min;
}

int LinearAnalogSensor::get_adc_min() {
  return _adc_min;
}

void LinearAnalogSensor::set_adc_max(int adc_max) {
  _adc_max = adc_max;
}

int LinearAnalogSensor::get_adc_max() {
  return _adc_max;
}

float LinearAnalogSensor::get_process_min() {
  return _process_min;
}

float LinearAnalogSensor::get_process_max() {
  return _process_max;
}

float LinearAnalogSensor::adc_value_to_process(int adc_value) {
  if (adc_value <= _adc_min) {
    return _process_min;
  } else if (adc_value >= _adc_max) {
    return _process_max;
  } else {
    return (adc_value - _adc_min) * _process_to_adc_ratio + _process_min;
  }
}

int LinearAnalogSensor::process_to_adc_value(float process) {
  if (process <= _process_min) {
    return _adc_min;
  } else if (process >= _process_max) {
    return _adc_max;
  } else {
    return (process - _process_min) / _process_to_adc_ratio + _adc_min;
  }
}
*/
