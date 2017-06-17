#include "joint.h"


Joint::Joint(Valve* valve, StringPot* pot, JointAngleTransform* angle_transform, PID* pid) {
  _valve = valve;
  _pot = pot;
  _angle_transform = angle_transform;
  _pid = pid;
  _pwm_min = 0;
  _pwm_max = 0;
  _min_pid_output = 0;
};

void Joint::set_pwm_limits(int pwm_min, int pwm_max) {
  _pwm_min = pwm_min;
  _pwm_max = pwm_max;
};

bool Joint::set_target_angle(float angle) {
  _target_angle = angle;
  _target_length = _angle_transform->angle_to_length(angle);
  _pid->set_setpoint(_pot->length_to_adc_value(_target_length));
};

bool Joint::set_target_length(float length) {
  _target_length = length;
  _target_angle = _angle_transform->length_to_angle(length);
  _pid->set_setpoint(_pot->length_to_adc_value(_target_length));
};

bool Joint::set_target_adc_value(unsigned int adc_value) {
  _pid->set_setpoint(adc_value);
  _target_length = _pot->adc_value_to_length(adc_value);
  _target_angle = _angle_transform->length_to_angle(_target_length);
};

float Joint::get_target_angle() {
  return _target_angle;
};

float Joint::get_target_length() {
  return _target_length;
};

unsigned int Joint::get_target_adc_value() {
  return _pid->get_setpoint();
};

float Joint::get_current_length() {
  return _pot->get_length();
};

float Joint::get_current_angle() {
  return _angle_transform->length_to_angle(get_current_length());
};

float Joint::get_pid_output() {
  return _pid_output;
};

void Joint::update() {
  _pot->read_adc();
  // if live, 
  if (!_valve->get_enabled()) {
    _pid->reset();
    return;
  };
  // TODO only update every n ms?
  _pid_output = _pid->update(_pot->get_adc_value());
  //int pwm = _pid_output;
  int pwm = 0;
  if (abs(_pid_output) < _min_pid_output) {
    pwm = 0;
  } else {
    if (_pid_output > 0) {
      //pwm = _pid_output;
      pwm = map(_pid_output, 0, _pid->get_output_max(), _pwm_min, _pwm_max);
      //pwm = _pid_output / _pid->get_output_max() * (_pwm_max - _pwm_min) + _pwm_min;
    } else {
      //pwm = _pid_output;
      pwm = -map(-_pid_output, 0, -_pid->get_output_min(), _pwm_min, _pwm_max);
      //pwm = -(_pid_output / _pid->get_output_min() * (_pwm_max - _pwm_min) + _pwm_min);
    };
  };
  //_pid_output = abs(_pid_output);
  _valve->set_pwm(pwm);
  /*
  // pid input 0->2 ** 12
  _pid->Compute();
  // ran = _pid.Compute();
  // if ran...
  // pid output -2**12->2**12
  // (optional) if output < T, output = 0
  // map output from -V->0->V to pwm -MAX_PWM->-MIN_PWM->MIN_PWM->MAX_PWM
  double pwm;
  if (_pid_output > 0) {
    pwm = map(_pid_output, 0, (1 >> 16), 0.3, 1.0);
  } else if (_pid_output < 0) {
    pwm = -map(-_pid_output, 0, (1 >> 16), 0.3, 1.0);
  } else {
    pwm = 0;
  };
  // set output
  _valve->set_pwm(pwm);
  */
};
