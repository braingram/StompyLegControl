#include "leg.h"


Leg::Leg() {
  estop = new EStop();

  adc = new ADC();
  adc->setAveraging(8, ADC_0);
  adc->setResolution(16, ADC_0);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED, ADC_0);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED, ADC_0);
  
  adc->setAveraging(8, ADC_1);
  adc->setResolution(16, ADC_1);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED, ADC_1);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED, ADC_1);


  hip_pot = new StringPot(
    HIP_SENSOR_PIN, adc, ADC_0,
    5952, 46208,  // 372, 2888
    HIP_CYLINDER_MIN_LENGTH, HIP_CYLINDER_MAX_LENGTH);

  thigh_pot = new StringPot(
    THIGH_SENSOR_PIN, adc, ADC_1,
    2176, 58688,  // 136, 3668
    THIGH_CYLINDER_MIN_LENGTH, THIGH_CYLINDER_MAX_LENGTH);

  knee_pot = new StringPot(
    KNEE_SENSOR_PIN, adc, ADC_0,
    9472, 59776,  //592, 3736
    KNEE_CYLINDER_MIN_LENGTH, KNEE_CYLINDER_MAX_LENGTH);


  hip_angle_transform = new JointAngleTransform(
    HIP_A, HIP_B, HIP_ZERO_ANGLE);

  thigh_angle_transform = new JointAngleTransform(
    THIGH_A, THIGH_B, THIGH_ZERO_ANGLE);

  knee_angle_transform = new JointAngleTransform(
    KNEE_A, KNEE_B, KNEE_ZERO_ANGLE);


  hip_pid = new PID(HIP_P, HIP_I, HIP_D, HIP_PID_MIN, HIP_PID_MAX);
  thigh_pid = new PID(THIGH_P, THIGH_I, THIGH_D, THIGH_PID_MIN, THIGH_PID_MAX);
  knee_pid = new PID(KNEE_P, KNEE_I, KNEE_D, KNEE_PID_MIN, KNEE_PID_MAX);


  hip_valve = new Valve(
      HIP_EXTEND_PIN, HIP_RETRACT_PIN, HIP_ENABLE_PIN);
  thigh_valve = new Valve(
      THIGH_EXTEND_PIN, THIGH_RETRACT_PIN, THIGH_ENABLE_PIN);
  knee_valve = new Valve(
      KNEE_EXTEND_PIN, KNEE_RETRACT_PIN, KNEE_ENABLE_PIN);


  hip_joint = new Joint(
      hip_valve, hip_pot, hip_angle_transform, hip_pid);
  thigh_joint = new Joint(
      thigh_valve, thigh_pot, thigh_angle_transform, thigh_pid);
  knee_joint = new Joint(
      knee_valve, knee_pot, knee_angle_transform, knee_pid);

  kinematics = new Kinematics(false);

  leg_number = LEG_NUMBER::UNDEFINED;
  // TODO read leg number from eeprom
};

void Leg::set_leg_number(LEG_NUMBER leg) {
  leg_number = leg;
  switch (leg_number) {
    case (LEG_NUMBER::MR):
    case (LEG_NUMBER::ML):
      hip_angle_transform->set_b(HIP_B_MIDDLE);
      hip_angle_transform->set_zero(HIP_ZERO_ANGLE_MIDDLE);
      break;
  };
};

void Leg::update() {
  // TODO check if this leg has a loaded calibration

  // if estop is active, disable all valves
  if ((estop->is_stopped()) | (leg_number == LEG_NUMBER::UNDEFINED)) {
    hip_valve->disable();
    thigh_valve->disable();
    knee_valve->disable();
  };

  // TODO check that plan is still valid

  // TODO update all joints
  hip_joint->update();
  thigh_joint->update();
  knee_joint->update();

  // TODO compute joint angles (at some interval?)

  // TODO compute foot xyz
};
