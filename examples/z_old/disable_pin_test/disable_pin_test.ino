#include "leg.h"

void setup() {
  digitalWrite(DISABLE_PIN, HIGH);
  pinMode(DISABLE_PIN, OUTPUT);
  Serial.begin(9600);
  analogWrite(HIP_EXTEND_PIN, 0);
  analogWrite(HIP_RETRACT_PIN, 0);
  analogWrite(THIGH_EXTEND_PIN, 0);
  analogWrite(THIGH_RETRACT_PIN, 0);
  analogWrite(KNEE_EXTEND_PIN, 0);
  analogWrite(KNEE_RETRACT_PIN, 0);
}

void set_pwm(char joint, int value) {
  int epin = -1;
  int rpin = -1;
  switch (joint) {
    case 'h':
      epin = HIP_EXTEND_PIN;
      rpin = HIP_RETRACT_PIN;
      break;
    case 't':
      epin = HIP_EXTEND_PIN;
      rpin = HIP_RETRACT_PIN;
      break;
    case 'k':
      epin = HIP_EXTEND_PIN;
      rpin = HIP_RETRACT_PIN;
      break;
  }
  if (epin == -1) return;
  if (value > 0) {
    analogWrite(rpin, 0);
    analogWrite(epin, value);
  } else if (value < 0) {
    analogWrite(epin, 0);
    analogWrite(rpin, -value);
  } else {
    analogWrite(epin, 0);
    analogWrite(rpin, 0);
  }
  Serial.print(joint); Serial.print(" = "); Serial.println(value);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'd': {
        bool v = !digitalRead(DISABLE_PIN);
        digitalWrite(DISABLE_PIN, v);
        Serial.print("D = "); Serial.println(v);
        break; }
      case 'h':
      case 't':
      case 'k':
        set_pwm(c, Serial.parseInt());
        break;
    }
  }
}
