#include "leg.h"


Leg::Leg() {
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, LOW);
  estop = new EStop();

  adc = new ADC();
  adc->setAveraging(ADC_N_AVG, ADC_0);
  adc->setResolution(ADC_RES, ADC_0);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED, ADC_0);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED, ADC_0);
  
  adc->setAveraging(ADC_N_AVG, ADC_1);
  adc->setResolution(ADC_RES, ADC_1);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED, ADC_1);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED, ADC_1);


  hip_pot = new StringPot(
    HIP_SENSOR_PIN, adc, ADC_0,
    HIP_ADC_MIN, HIP_ADC_MAX,
    HIP_CYLINDER_MIN_LENGTH, HIP_CYLINDER_MAX_LENGTH);

  thigh_pot = new StringPot(
    THIGH_SENSOR_PIN, adc, ADC_1,
    THIGH_ADC_MIN, THIGH_ADC_MAX,
    THIGH_CYLINDER_MIN_LENGTH, THIGH_CYLINDER_MAX_LENGTH);

  knee_pot = new StringPot(
    KNEE_SENSOR_PIN, adc, ADC_0,
    KNEE_ADC_MIN, KNEE_ADC_MAX,
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

  hip_joint->set_pwm_limits(HIP_EXTEND_PWM_MIN, HIP_PWM_MAX);
  thigh_joint->set_pwm_limits(THIGH_EXTEND_PWM_MIN, KNEE_PWM_MAX);
  knee_joint->set_pwm_limits(THIGH_EXTEND_PWM_MIN, KNEE_PWM_MAX);

  kinematics = new Kinematics(false);

  leg_number = LEG_NUMBER::UNDEFINED;
  // TODO read leg number from eeprom

  target_position.x = hip_pot->get_adc_value();
  target_position.y = thigh_pot->get_adc_value();
  target_position.z = knee_pot->get_adc_value();

  current_plan.mode = PLAN_STOP_MODE;
  current_plan.frame = PLAN_SENSOR_FRAME;
  current_plan.active = true;
  next_plan.active = false;

  last_target_point_generation_time = 0;
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
    disable_valves();
  } else {
    enable_valves();
  };

  // check that plan is still valid
  if (next_plan.active) {
    long pdt = next_plan.start_time - millis();
    if (pdt <= 0) {
      current_plan = next_plan;
      next_plan.active = false;
 
      // make sure a new point gets generated TODO make configurable
      last_target_point_generation_time = millis() - 21;
    };
  };

  // check if a new target position should be generated
  // TODO make time configurable
  if (millis() - last_target_point_generation_time > 20) {
    // generate next target point TODO check output, stop on false
    // TODO make dt configurable
    follow_plan(current_plan, target_position, &target_position, 0.025); 
    last_target_point_generation_time = millis();

    // set joint targets
    switch (current_plan.frame) {
      case (PLAN_BODY_FRAME):  // TODO handle this in python
        // target is xyz in body coordinates
        break;
      case (PLAN_LEG_FRAME):
        // target position is xyz
        // convert to angles, then to sensor units
        JointAngle3D a;
        // TODO check return value, stop on false
        kinematics->xyz_to_angles(target_position, &a);
        hip_joint->set_target_angle(a.hip);
        thigh_joint->set_target_angle(a.thigh);
        knee_joint->set_target_angle(a.knee);
        break;
      case (PLAN_JOINT_FRAME):
        // TODO check angles, stop when out of range
        // target position are joint angles
        // convert to sensor units, set pid
        hip_joint->set_target_angle(target_position.x);
        thigh_joint->set_target_angle(target_position.y);
        knee_joint->set_target_angle(target_position.z);
        break;
      case (PLAN_SENSOR_FRAME):
        // target position is in sensor units, set pid targets
        hip_joint->set_target_adc_value(target_position.x);
        thigh_joint->set_target_adc_value(target_position.y);
        knee_joint->set_target_adc_value(target_position.z);
        break;
    };
  };

  // update all joints, this will read the sensors
  hip_joint->update();
  thigh_joint->update();
  knee_joint->update();
};

void Leg::compute_foot_position() {
  // compute joint angles
  joint_angles.hip = hip_joint->get_current_angle();
  joint_angles.thigh = thigh_joint->get_current_angle();
  joint_angles.knee = knee_joint->get_current_angle();

  // compute foot xyz
  kinematics->angles_to_xyz(joint_angles, &foot_position);
};

void Leg::set_next_plan(PlanStruct new_plan) {
  next_plan = new_plan;
  next_plan.active = true;
};

void Leg::disable_valves() {
  hip_valve->disable();
  thigh_valve->disable();
  knee_valve->disable();
  digitalWrite(STATUS_PIN, LOW);
};

void Leg::enable_valves() {
  hip_valve->enable();
  thigh_valve->enable();
  knee_valve->enable();
  digitalWrite(STATUS_PIN, HIGH);
};
