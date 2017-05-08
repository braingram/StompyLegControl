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

void Valve::extend_pwm(int pwm) {
  _pwm = pwm;
  if (pwm == 0) {
    _direction = VALVE_STOPPED;
    return;
  }
  analogWrite(_retractPin, 0);
  analogWrite(_extendPin, pwm);
  _direction = VALVE_EXTENDING;
}

void Valve::retract_pwm(int pwm) {
  _pwm = pwm;
  if (pwm == 0) {
    _direction = VALVE_STOPPED;
    return;
  }
  analogWrite(_extendPin, 0);
  analogWrite(_retractPin, pwm);
  _direction = VALVE_RETRACTING;
}

void Valve::extend_ratio(float ratio) {
  extend_pwm(ratio * _pwm_max);
}

void Valve::retract_ratio(float ratio) {
  retract_pwm(ratio * _pwm_max);
}

void Valve::stop() {
  retract_pwm(0);
}

void Valve::enable() {
  digitalWrite(_enablePin, HIGH);
}

void Valve::disable() {
  digitalWrite(_enablePin, LOW);
  // TODO should this also stop?
}

int Valve::get_pwm() {
  return _pwm;
}

int Valve::get_direction() {
  return _direction;
};
