#include "pid.h"

PID::PID(float p, float i, float d) {
  _p = p;
  _i = i;
  _d = d;

  //_error_threshold = 1E10;  // default to large (off)
  _error_threshold = FOLLOWING_ERROR_THRESHOLD;  // default to large (off)
  _setpoint = 0;

  set_output_limits(0, 255);

  reset();
  _last_update = millis();
}

PID::PID(float p, float i, float d, float min_output, float max_output) : PID(p, i, d) {
  set_output_limits(min_output, max_output);
}

void PID::set_setpoint(float setpoint) {
  _setpoint = setpoint;
}

float PID::get_setpoint() {
  return _setpoint;
}

void PID::set_output_limits(float min_output, float max_output) {
  if (min_output > max_output) return;
  _min_output = min_output;
  _max_output = max_output;
}

float PID::get_output_min() {
  return _min_output;
}

float PID::get_output_max() {
  return _max_output;
}

void PID::set_error_threshold(float error_threshold) {
  _error_threshold = error_threshold;
};

float PID::get_error_threshold() {
  return _error_threshold;
};

bool PID::error_ok() {
  return abs(get_error()) < _error_threshold;
};

float PID::update(float input) {
  unsigned long t = millis();
  if (t == _last_update) return _output;  // TODO workaround
  float dt = ((unsigned long)(t - _last_update)) / 1000.;

  // compute error
  float previous_error = _error;
  _error = _setpoint - input;

  // compute i term
  _i_term += (_i * _error * dt);

  // limit
  if (_i_term < _min_output) _i_term = _min_output;
  if (_i_term > _max_output) _i_term = _max_output;

  // compute and limit output
  _output = _p * _error + _i_term + (_error - previous_error) * _d / dt;
  if (_output < _min_output) _output = _min_output;
  if (_output > _max_output) _output = _max_output;

  // store values
  _last_update = t;
  
  // return output
  return _output;
}

float PID::get_output() {
  return _output;
}

float PID::get_error() {
  return _error;
}

void PID::reset() {
  _i_term = 0;
  _error = 0;
  _output = 0;

  _last_update = millis();
}

void PID::reset_i() {
  _i_term = 0;
}

float PID::get_p() {
  return _p;
}

float PID::get_i() {
  return _i;
}

float PID::get_d() {
  return _d;
}

void PID::set_p(float p) {
  _p = p;
}

void PID::set_i(float i) {
  _i = i;
}

void PID::set_d(float d) {
  _d = d;
}
