#include "leg.h"

// numbers from 170807 for MR
#define HIP_ADC_MIN 7073
#define HIP_ADC_MAX 52174
#define THIGH_ADC_MIN 3584
#define THIGH_ADC_MAX 59758
#define KNEE_ADC_MIN 9040
#define KNEE_ADC_MAX 59086

byte to_print = 'v';

Leg* leg = new Leg();

void setup(){
  Serial.begin(9600);
  // first set estop to default value
  leg->estop->set_estop(ESTOP_DEFAULT);
  // for now, set leg number in sketch
  leg->set_leg_number(LEG_NUMBER::MR);
  // disable pids, set pwms to 0
  leg->disable_pids();
  leg->hip_valve->set_ratio(0);
  leg->thigh_valve->set_ratio(0);
  leg->knee_valve->set_ratio(0);

  // override calibration
  leg->hip_pot->set_adc_range(HIP_ADC_MIN, HIP_ADC_MAX);
  leg->thigh_pot->set_adc_range(THIGH_ADC_MIN, THIGH_ADC_MAX);
  leg->knee_pot->set_adc_range(KNEE_ADC_MIN, KNEE_ADC_MAX);
};

void loop(){
  if (Serial.available()) {
    byte c = Serial.read();
    if ((c != '\r') && (c != '\n')) {
      to_print = c;
    };
  };
  leg->update();
  leg->compute_foot_position();
  float a, b, c;
  switch (to_print) {
    case 'v':
      a = leg->hip_pot->get_adc_value();
      b = leg->thigh_pot->get_adc_value();
      c = leg->knee_pot->get_adc_value();
      break;
    case 'l':
      a = leg->hip_pot->get_length();
      b = leg->thigh_pot->get_length();
      c = leg->knee_pot->get_length();
      break;
    case 'a':
      a = leg->joint_angles.hip;
      b = leg->joint_angles.thigh;
      c = leg->joint_angles.knee;
      break;
    case 'x':
      a = leg->foot_position.x;
      b = leg->foot_position.y;
      c = leg->foot_position.z;
      break;
    default:
      a = 0; b = 0; c = 0;
      break;
  };
  Serial.print(a); Serial.print(", ");
  Serial.print(b); Serial.print(", ");
  Serial.print(c); Serial.println();
  delay(100);
};
