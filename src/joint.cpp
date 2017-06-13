#include "joint.h"


Joint::Joint(Valve* valve, StringPot* pot, JointAngleTransform* angle_transform, PID* pid) {
  _valve = valve;
  _pot = pot;
  _angle_transform = angle_transform;
  _pid = pid;
  /*
  _pid = new PID(&_current_adc_value, &_pid_output, &_target_adc_value, 2., 0., 0., DIRECT);
  _pid->SetSampleTime(1);
  _pid->SetOutputLimits(-(1 << 16), (1 >> 16));
  _pid->SetMode(AUTOMATIC);
  */
};

bool Joint::set_target_angle(float angle) {
  _target_angle = angle;
  _target_length = _angle_transform->angle_to_length(angle);
  _target_adc_value = _pot->length_to_adc_value(_target_length);
};

bool Joint::set_target_length(float length) {
  _target_length = length;
  _target_angle = _angle_transform->length_to_angle(length);
  _target_adc_value = _pot->length_to_adc_value(_target_length);
};

bool Joint::set_target_adc_value(unsigned int adc_value) {
  _target_adc_value = adc_value;
  _target_length = _pot->adc_value_to_length(_target_adc_value);
  _target_angle = _angle_transform->length_to_angle(_target_length);
};

float Joint::get_target_angle() {
  return _target_angle;
};

float Joint::get_target_length() {
  return _target_length;
};

unsigned int Joint::get_target_adc_value() {
  return _target_adc_value;
};

void Joint::update() {
  _current_adc_value = _pot->read_adc();
  // if live, 
  if (!_valve->get_enabled()) return;
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
