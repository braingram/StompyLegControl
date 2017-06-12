#include "leg.h"


Leg::Leg() {
  _estop = new EStop();

  _adc = new ADC();
  _adc->setAveraging(8, ADC_0);
  _adc->setResolution(16, ADC_0);
  _adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED, ADC_0);
  _adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED, ADC_0);
  
  _adc->setAveraging(8, ADC_1);
  _adc->setResolution(16, ADC_1);
  _adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED, ADC_1);
  _adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED, ADC_1);


  _hip_pot = new StringPot(
    HIP_SENSOR_PIN, _adc, ADC_0,
    5952, 46208,  // 372, 2888
    HIP_CYLINDER_MIN_LENGTH, HIP_CYLINDER_MAX_LENGTH);

  _thigh_pot = new StringPot(
    THIGH_SENSOR_PIN, _adc, ADC_1,
    2176, 58688,  // 136, 3668
    THIGH_CYLINDER_MIN_LENGTH, THIGH_CYLINDER_MAX_LENGTH);

  _knee_pot = new StringPot(
    KNEE_SENSOR_PIN, _adc, ADC_0,
    9472, 59776,  //592, 3736
    KNEE_CYLINDER_MIN_LENGTH, KNEE_CYLINDER_MAX_LENGTH);


  _hip_angle_transform = new JointAngleTransform(
    HIP_A, HIP_B, HIP_ZERO_ANGLE);

  _thigh_angle_transform = new JointAngleTransform(
    THIGH_A, THIGH_B, THIGH_ZERO_ANGLE);

  _knee_angle_transform = new JointAngleTransform(
    KNEE_A, KNEE_B, KNEE_ZERO_ANGLE);


  _hip_valve = new Valve(
      HIP_EXTEND_PIN, HIP_RETRACT_PIN, HIP_ENABLE_PIN);
  _thigh_valve = new Valve(
      THIGH_EXTEND_PIN, THIGH_RETRACT_PIN, THIGH_ENABLE_PIN);
  _knee_valve = new Valve(
      KNEE_EXTEND_PIN, KNEE_RETRACT_PIN, KNEE_ENABLE_PIN);


  _hip_joint = new Joint(_hip_valve, _hip_pot, _hip_angle_transform);
  _thigh_joint = new Joint(_thigh_valve, _thigh_pot, _thigh_angle_transform);
  _knee_joint = new Joint(_knee_valve, _knee_pot, _knee_angle_transform);

  _kinematics = new Kinematics(false);

  _defined = false;
};

void Leg::set_leg_info(byte side, byte order) {
  // TODO
  if (side == SIDE_LEFT) {
  } else if (side == SIDE_RIGHT) {
  } else {
    return;
  }
  if (order == ORDER_FRONT) {
  } else if (order == ORDER_MIDDLE) {
  } else if (order == ORDER_BACK) {
  } else {
    return;
  };
  _defined = true;
};

void Leg::update() {
  // TODO check if this leg has a loaded calibration

  // if estop is active, disable all valves
  if ((_estop->is_stopped()) | (! _defined)) {
    _hip_valve->disable();
    _thigh_valve->disable();
    _knee_valve->disable();
  };

  // TODO check that plan is still valid

  // TODO update all joints
  _hip_joint->update();
  _thigh_joint->update();
  _knee_joint->update();

  // TODO compute joint angles (at some interval?)

  // TODO compute foot xyz
};
