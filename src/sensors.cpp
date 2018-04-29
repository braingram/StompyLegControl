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

unsigned int AnalogSensor::read_adc() {
  _adc_value = _adc->analogRead(_pin, _adc_number);
  return _adc_value;
}

unsigned int AnalogSensor::get_adc_value() {
  return _adc_value;
}


/* ========================================================
 *                FilteredAnalogSensor
 * ========================================================*/
FilteredAnalogSensor::FilteredAnalogSensor(int pin, ADC* adc, int adc_number):AnalogSensor(pin, adc, adc_number) {
  _adc = adc;
  _adc_number = adc_number;
  _pin = pin;

  // initialize adc
  unsigned int v = _adc->analogRead(_pin, _adc_number);
  for (int i=0; i<N_FILTER_SAMPLES; i++) {
    _samples[i] = v;
  };
  _sample_index = 0;
  _adc_value = v;
}

void FilteredAnalogSensor::sample() {
  _samples[_sample_index] = _adc->analogRead(_pin, _adc_number);
  _sample_index = (_sample_index + 1) % N_FILTER_SAMPLES;
}

int filter_sort(const void *a, const void *b) {
  return *((int *)a) - *((int *)b);
}

void FilteredAnalogSensor::filter() {
  qsort(_samples, N_FILTER_SAMPLES, sizeof(int), filter_sort);
  long s = 0;
  for (int i=FILTER_MIN_INDEX; i<=FILTER_MAX_INDEX; i++) {
    s += _samples[i];
  }
  _adc_value = (s / FILTER_AVG_N);
}

unsigned int FilteredAnalogSensor::read_adc() {
  filter();
  return get_adc_value();
}
  
unsigned int FilteredAnalogSensor::get_adc_value() {
  return _adc_value;
}

/* ========================================================
 *                      StringPot
 * ========================================================*/

StringPot::StringPot(FilteredAnalogSensor* analog_sensor, Transform* transform) {
  _analog_sensor = analog_sensor;
  _transform = transform;
}

StringPot::StringPot(FilteredAnalogSensor* analog_sensor, unsigned int adc_min, unsigned int adc_max, float length_min, float length_max) {
  _analog_sensor = analog_sensor;
  _transform = new LinearTransform(adc_min, adc_max, length_min, length_max);
}

float StringPot::read_length() {
  _analog_sensor->read_adc();
  return get_length();
}

unsigned int StringPot::read_adc() {
  return _analog_sensor->read_adc();
}

float StringPot::get_length() {
  return adc_value_to_length(_analog_sensor->get_adc_value());
}

unsigned int StringPot::get_adc_value() {
  return _analog_sensor->get_adc_value();
}

float StringPot::adc_value_to_length(unsigned int adc_value) {
  return _transform->src_to_dst(adc_value);
}

unsigned int StringPot::length_to_adc_value(float length) {
  return (unsigned int)(_transform->dst_to_src(length));
}

void StringPot::set_adc_min(float value) {
  ((LinearTransform *)_transform)->set_src_min(value);
}

float StringPot::get_adc_min() {
  return ((LinearTransform *)_transform)->get_src_min();
}

void StringPot::set_adc_max(float value) {
  ((LinearTransform *)_transform)->set_src_max(value);
}

float StringPot::get_adc_max() {
  return ((LinearTransform *)_transform)->get_src_max();
}

void StringPot::set_adc_range(float min_value, float max_value) {
  set_adc_min(min_value);
  set_adc_max(max_value);
}

void StringPot::set_length_min(float value) {
  ((LinearTransform *)_transform)->set_dst_min(value);
}

float StringPot::get_length_min() {
  return ((LinearTransform *)_transform)->get_dst_min();
}

void StringPot::set_length_max(float value) {
  ((LinearTransform *)_transform)->set_dst_max(value);
}

float StringPot::get_length_max() {
  return ((LinearTransform *)_transform)->get_dst_max();
}

void StringPot::set_length_range(float min_value, float max_value) {
  set_length_min(min_value);
  set_length_max(max_value);
}

int StringPot::update() {
  if (_sample_timer > STRING_POT_SAMPLE_TIME) {
    _analog_sensor->sample();
    _sample_count = (_sample_count + 1) % N_FILTER_SAMPLES;
    if (_sample_count == 0) {
      _analog_sensor->filter();
      return STRING_POT_READY;
    };
  };
  return STRING_POT_NO_SAMPLE;
}

bool StringPot::adc_in_range() {
  if (_analog_sensor->get_adc_value() > ((LinearTransform *)_transform)->get_src_max()) return false;
  if (_analog_sensor->get_adc_value() < ((LinearTransform *)_transform)->get_src_min()) return false;
  return true;
}


/* ========================================================
 *                      CalfSensor
 * ========================================================*/

CalfSensor::CalfSensor(FilteredAnalogSensor* analog_sensor, CalfLoadTransform* transform) {
  _analog_sensor = analog_sensor;
  _transform = transform;
}

float CalfSensor::read_load() {
  _analog_sensor->read_adc();
  return get_load();
}

unsigned int CalfSensor::read_adc() {
  return _analog_sensor->read_adc();
}

float CalfSensor::get_load() {
  return adc_value_to_load(_analog_sensor->get_adc_value());
}

unsigned int CalfSensor::get_adc_value() {
  return _analog_sensor->get_adc_value();
}

float CalfSensor::adc_value_to_load(unsigned int adc_value) {
  return _transform->src_to_dst(adc_value);
}

unsigned int CalfSensor::load_to_adc_value(float load) {
  return (unsigned int)(_transform->dst_to_src(load));
}

int CalfSensor::update() {
  // TODO same sample time/return as string pots?
  if (_sample_timer > STRING_POT_SAMPLE_TIME) {
    _analog_sensor->sample();
    _sample_count = (_sample_count + 1) % N_FILTER_SAMPLES;
    if (_sample_count == 0) {
      _analog_sensor->filter();
      return STRING_POT_READY;
    };
  };
  return STRING_POT_NO_SAMPLE;
}

/* ========================================================
 *                   PressureSensor
 * ========================================================*/
/*
PressureSensor::PressureSensor(int pin, ADC* adc, int adc_number, Transform* transform) : AnalogSensor(pin, adc, adc_number) {
  _transform = transform;
}

PressureSensor::PressureSensor(int pin, ADC* adc, int adc_number, unsigned int adc_min, unsigned int adc_max, float pressure_min, float pressure_max) : AnalogSensor(pin, adc, adc_number) {
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
*/

/* ========================================================
 *                   JoystickAxis
 * ========================================================*/
/*
JoystickAxis::JoystickAxis(int pin, ADC* adc, int adc_number, Transform* transform) : AnalogSensor(pin, adc, adc_number) {
  _transform = transform;
}


JoystickAxis::JoystickAxis(int pin, ADC* adc, int adc_number, unsigned int adc_min, unsigned int adc_deadband_min, unsigned int adc_deadband_max, unsigned int adc_max, float axis_min, float axis_mid, float axis_max) : AnalogSensor(pin, adc, adc_number) {
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
*/
