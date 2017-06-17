#include "leg.h"


Leg* leg = new Leg();

elapsedMicros timer;
float time_sum = 0;
float time_n = 0;

char to_print = 's';
// v = adc
// s = string pots
// a = angle
// x = xyz
// t = timing

void setup() {
  Serial.begin(9600);
  leg->set_leg_number(LEG_NUMBER::FR);
  leg->disable_valves();
}

void loop() {
  timer = 0;
  leg->update();
  leg->compute_foot_position();
  unsigned long us = timer;
  time_sum += us; time_n += 1;
  

  switch(to_print) {
    case 't':
      Serial.print("us = "); Serial.print(us);
      Serial.print(", avg us = "); Serial.println(time_sum / time_n);
      break;
    case 'v':
      Serial.print(leg->hip_pot->get_adc_value()); Serial.print(", ");
      Serial.print(leg->thigh_pot->get_adc_value()); Serial.print(", ");
      Serial.print(leg->knee_pot->get_adc_value()); Serial.println();
      break;
    case 's':
      Serial.print(leg->hip_pot->get_length()); Serial.print(", ");
      Serial.print(leg->thigh_pot->get_length()); Serial.print(", ");
      Serial.print(leg->knee_pot->get_length()); Serial.println();
      break;
    case 'a':
      Serial.print(leg->joint_angles.hip); Serial.print(", ");
      Serial.print(leg->joint_angles.thigh); Serial.print(", ");
      Serial.print(leg->joint_angles.knee); Serial.println();
      break;
    case 'x':
      Serial.print(leg->foot_position.x); Serial.print(", ");
      Serial.print(leg->foot_position.y); Serial.print(", ");
      Serial.print(leg->foot_position.z); Serial.println();
      break;
  }

  if (Serial.available()) {
    char c = Serial.read();
    if ((c != '\r') & (c != '\n')) to_print = c;
  }
}
