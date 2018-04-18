#include "leg.h"

elapsedMillis t;

Point3D pt;
Point3D target;
PlanStruct plan;

void setup() {
  pt.x = 82;
  pt.y = 0;
  pt.z = -32;

  plan.mode = PLAN_ARC_MODE;
  plan.frame = PLAN_LEG_FRAME;
  plan.linear.x = 0;
  plan.linear.y = 0;
  plan.linear.y = 0;
  plan.angular.x = 0.0;
  plan.angular.y = 0.0;
  plan.angular.z = 0.0;
  plan.speed = 1.0;

  prepare_plan(&plan);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r':
        pt.x = 82;
        pt.y = 0;
        pt.z = -32;
        break;
      case 't': {
        elapsedMicros ut;
        for (int i=0; i<100; i++)
          prepare_plan(&plan);
        unsigned long tt = ut;
        Serial.print("100 prepares took "); Serial.print(tt); Serial.println(" us");
      } break;
      case 'm':
        for (int i=0; i<4; i++) {
          for (int j=0; j<4; j++) {
            Serial.print(plan.t_matrix[i][j], 6); Serial.print(",");
          }
          Serial.println();
        }
        break;
    }
  }
  // put your main code here, to run repeatedly:
  if (t > 1000) {
    follow_plan(plan, pt, &target, 1.0);
    Serial.print(pt.x); Serial.print(",");
    Serial.print(pt.y); Serial.print(",");
    Serial.println(pt.z);
    pt.x = target.x;
    pt.y = target.y;
    pt.z = target.z;
    t = 0;
  }
}
