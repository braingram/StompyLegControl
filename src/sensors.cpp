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

PressureSensor::PressureSensor(int pin, ADC* adc, int adc_number, int adc_min, int adc_max, float pressure_min, float pressure_max) : AnalogSensor(pin, adc, adc_number) {
  _transform = new LinearTransform(adc_min, adc_max, pressure_min, pressure_max);
}

float PressureSensor::read_pressure() {
  read_adc();
  _pressure = _transform->src_to_dst(_adc_value);
  return _pressure;
}

float PressureSensor::get_pressure() {
  return _pressure;
}


/* ========================================================
 *                   JoystickAxis
 * ========================================================*/

JoystickAxis::JoystickAxis(int pin, ADC* adc, int adc_number, Transform* transform) : AnalogSensor(pin, adc, adc_number) {
  _transform = transform;
}


JoystickAxis::JoystickAxis(int pin, ADC* adc, int adc_number, int adc_min, int adc_deadband_min, int adc_deadband_max, int adc_max, float axis_min, float axis_mid, float axis_max) : AnalogSensor(pin, adc, adc_number) {
  _transform = new LinearDeadbandTransform(adc_min, adc_deadband_min, adc_deadband_max, adc_max, axis_min, axis_mid, axis_max);
}

float JoystickAxis::read_axis() {
  read_adc();
  _axis = _transform->src_to_dst(_adc_value);
  return _axis;
}

float JoystickAxis::get_axis() {
  return _axis;
}