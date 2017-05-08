#include "sensors.h"

/* ========================================================
 *                      AnalogSensor
 * ========================================================*/

AnalogSensor::AnalogSensor(int pin, ADC* adc, int adc_number) {
  _adc = adc;
  _adc_number = adc_number;
  _pin = pin;

  // initialize adc
  read_adc();
}

int AnalogSensor::read_adc() {
  _adc_value = _adc->analogRead(_pin, _adc_number);
  return _adc_value;
}

int AnalogSensor::get_adc_value() {
  return _adc_value;
}


/* ========================================================
 *                      StringPot
 * ========================================================*/

StringPot::StringPot(int pin, ADC* adc, int adc_number, Transform* transform) : AnalogSensor(pin, adc, adc_number) {
  _transform = transform;
}

StringPot::StringPot(int pin, ADC* adc, int adc_number, int adc_min, int adc_max, float length_min, float length_max) : AnalogSensor(pin, adc, adc_number) {
  _transform = new LinearTransform(adc_min, adc_max, length_min, length_max);
}

float StringPot::read_length() {
  read_adc();
  _length = adc_value_to_length(_adc_value);
  return _length;
}

float StringPot::get_length() {
  return _length;
}

float StringPot::adc_value_to_length(int adc_value) {
  return _transform->src_to_dst(adc_value);
}

int StringPot::length_to_adc_value(float length) {
  return (int)(_transform->dst_to_src(length));
}


/* ========================================================
 *                   PressureSensor
 * ========================================================*/

PressureSensor::PressureSensor(int pin, ADC* adc, int adc_number, Transform* transform) : AnalogSensor(pin, adc, adc_number) {
  _transform = transform;
}

float PressureSensor::read_pressure() {
  read_adc();
  _pressure = _transform->src_to_dst(_adc_value);
  return _pressure;
}

float PressureSensor::get_pressure() {
  return _pressure;
}
