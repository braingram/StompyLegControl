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
