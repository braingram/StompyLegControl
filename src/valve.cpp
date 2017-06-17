#include "valve.h"

Valve::Valve(int extendPin, int retractPin, int enablePin, float frequency, int resolution) {
  // store state
  _pwm = 0;
  _direction = 0;

  // store pins
  _extendPin = extendPin;
  _retractPin = retractPin;
  _enablePin = enablePin;

  // compute max by resolution
  _pwm_max = (1 << resolution) - 1;

  // setup pins
  analogWriteFrequency(_extendPin, frequency);
  analogWriteFrequency(_retractPin, frequency);
  analogWriteResolution(resolution);
  pinMode(_enablePin, OUTPUT);

  // as a default, disable
  disable();
}


Valve::Valve(int extendPin, int retractPin, int enablePin, float frequency) : Valve(extendPin, retractPin, enablePin, frequency, VALVE_PWM_RESOLUTION) {
}

Valve::Valve(int extendPin, int retractPin, int enablePin) : Valve(extendPin, retractPin, enablePin, VALVE_PWM_FREQUENCY, VALVE_PWM_RESOLUTION) {
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
  _pwm = pwm;
  if (pwm == 0) return stop();
  analogWrite(_retractPin, 0);
  analogWrite(_extendPin, pwm);
  _direction = VALVE_EXTENDING;
}

void Valve::retract_pwm(int pwm) {
  _pwm = pwm;
  if (pwm == 0) return stop();
  analogWrite(_extendPin, 0);
  analogWrite(_retractPin, pwm);
  _direction = VALVE_RETRACTING;
}

void Valve::set_ratio(float ratio) {
  if (ratio > 0) {
    extend_ratio(ratio);
  } else {
    retract_ratio(-ratio);
  };
}

void Valve::extend_ratio(float ratio) {
  extend_pwm(ratio * _pwm_max);
}

void Valve::retract_ratio(float ratio) {
  retract_pwm(ratio * _pwm_max);
}

void Valve::enable() {
  digitalWrite(_enablePin, HIGH);
  _enabled = true;
}

void Valve::disable() {
  stop();
  digitalWrite(_enablePin, LOW);
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
