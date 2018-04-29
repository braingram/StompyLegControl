#include "joint.h"


Joint::Joint(Valve* valve, StringPot* pot, JointAngleTransform* angle_transform, PID* pid) {
  _valve = valve;
  _pot = pot;
  _angle_transform = angle_transform;
  _pid = pid;
  _pid_output = 0;
  _min_pid_output = 0;
  disable_pid();

  _pot->read_length();
  set_target_adc_value(_pot->get_adc_value());
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

void Joint::enable_pid() {
  _use_pid = true;
  // when enabling the pid, start with the setpoint at the current adc value
  _pot->read_adc();
  set_target_adc_value(_pot->get_adc_value());
};

void Joint::disable_pid() {
  _use_pid = false;
};

bool Joint::pid_enabled() {
  return _use_pid;
};

float Joint::get_pid_output() {
  return _pid_output;
};

void Joint::_update_pid() {
  // TODO only update every n ms?
  _pid_output = _pid->update(_pot->get_adc_value());
  int pwm = 0;
  if (abs(_pid_output) < _min_pid_output) {
    pwm = 0;
  } else {
    if (_pid_output > 0) {
      // TODO map this to 0-1? and use set ratio
      pwm = map(
        _pid_output, 0, _pid->get_output_max(),
        _valve->get_extend_pwm_min(), _valve->get_extend_pwm_max());
    } else {
      pwm = -map(
        -_pid_output, 0, -_pid->get_output_min(),
        _valve->get_retract_pwm_min(), _valve->get_retract_pwm_max());
    };
  };
  _valve->set_pwm(pwm);
};

void Joint::update() {
  // read adc
  int pot_ready = _pot->update();
  //_pot->read_adc();
  // if live, 
  if (!_valve->get_enabled()) {
    _pid->reset();
    return;
  };
  if (!_use_pid) {
    return;
  };
  if (pot_ready == STRING_POT_READY) {
    // every STRING_POT_SAMPLE_TIME * N_FILTER_SAMPLES ms (20 ms)
    _update_pid();
  } else {
    // update dither and (possibly) set the pwm
    if (_valve->update_dither()) _valve->set_pwm();
  };
};
