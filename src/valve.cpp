#include "valve.h"

Valve::Valve(int extendPin, int retractPin, int enablePin, int disablePin, float frequency, int resolution) {
  // store state
  _pwm = 0;
  _direction = 0;

  // store pins
  _extendPin = extendPin;
  _retractPin = retractPin;
  _enablePin = enablePin;
  _disablePin = disablePin;

  // compute max by resolution
  set_pwm_limits(0, (1 << resolution) - 1, 0, (1 << resolution) - 1);

  // setup pins
  analogWriteFrequency(_extendPin, frequency);
  analogWriteFrequency(_retractPin, frequency);
  analogWriteResolution(resolution);
  pinMode(_enablePin, OUTPUT);
  pinMode(_disablePin, OUTPUT);

  // as a default, disable
  disable();
}


Valve::Valve(int extendPin, int retractPin, int enablePin, int disablePin, float frequency) : Valve(extendPin, retractPin, enablePin, disablePin, frequency, VALVE_PWM_RESOLUTION) {
}

Valve::Valve(int extendPin, int retractPin, int enablePin, int disablePin) : Valve(extendPin, retractPin, enablePin, disablePin, VALVE_PWM_FREQUENCY, VALVE_PWM_RESOLUTION) {
}

void Valve::stop() {
  _pwm = 0;
  analogWrite(_retractPin, 0);
  analogWrite(_extendPin, 0);
  _direction = VALVE_STOPPED;
}

void Valve::set_pwm(int pwm) {
  if (pwm > 0) {
    extend_pwm(pwm);
  } else {
    retract_pwm(-pwm);
  };
}

void Valve::extend_pwm(int pwm) {
  if (pwm <= _extend_pwm_min) return stop();
  if (pwm > _extend_pwm_max) pwm = _extend_pwm_max;
  _pwm = pwm;
  analogWrite(_retractPin, 0);
  analogWrite(_extendPin, pwm);
  _direction = VALVE_EXTENDING;
}

void Valve::retract_pwm(int pwm) {
  if (pwm <= _retract_pwm_min) return stop();
  if (pwm > _retract_pwm_max) pwm = _retract_pwm_max;
  _pwm = pwm;
  analogWrite(_extendPin, 0);
  analogWrite(_retractPin, pwm);
  _direction = VALVE_RETRACTING;
}

void Valve::set_pwm_limits(int extend_min, int extend_max, int retract_min, int retract_max) {
  if (extend_min > extend_max) return;
  if (retract_min > retract_max) return;
  set_extend_pwm_min(extend_min);
  set_extend_pwm_max(extend_max);
  set_retract_pwm_min(retract_min);
  set_retract_pwm_max(retract_max);
}

void Valve::set_extend_pwm_max(int max) {
  _extend_pwm_max = max;
}

int Valve::get_extend_pwm_max() {
  return _extend_pwm_max;
}

void Valve::set_extend_pwm_min(int min) {
  _extend_pwm_min = min;
}

int Valve::get_extend_pwm_min() {
  return _extend_pwm_min;
}

void Valve::set_retract_pwm_max(int max) {
  _retract_pwm_max = max;
}

int Valve::get_retract_pwm_max() {
  return _retract_pwm_max;
}

void Valve::set_retract_pwm_min(int min) {
  _retract_pwm_min = min;
}

int Valve::get_retract_pwm_min() {
  return _retract_pwm_min;
}


void Valve::set_ratio(float ratio) {
  if (ratio > 0) {
    extend_ratio(ratio);
  } else {
    retract_ratio(-ratio);
  };
}

void Valve::extend_ratio(float ratio) {
  extend_pwm(
    ratio * (_extend_pwm_max - _extend_pwm_min) + _extend_pwm_min);
}

void Valve::retract_ratio(float ratio) {
  retract_pwm(
    ratio * (_retract_pwm_max - _retract_pwm_min) + _retract_pwm_min);
}

void Valve::enable() {
  digitalWrite(_disablePin, LOW);
  digitalWrite(_enablePin, HIGH);
  _enabled = true;
}

void Valve::disable() {
  stop();
  digitalWrite(_enablePin, LOW);
  digitalWrite(_disablePin, HIGH);
  _enabled = false;
}

bool Valve::get_enabled() {
  return _enabled;
}

int Valve::get_pwm() {
  return _pwm;
}

int Valve::get_direction() {
  return _direction;
};
