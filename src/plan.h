/*
	plan.h - Movement plan library.
	Created by Brett Graham, June 10, 2017.
	Released into the public domain -- so help you God.
*/

#ifndef PLAN_H
#define PLAN_H

#include "Arduino.h"
#include "point.h"

#define PLAN_STOP_MODE 0
#define PLAN_VELOCITY_MODE 1
#define PLAN_ARC_MODE 2
#define PLAN_TARGET_MODE 3
#define PLAN_MATRIX_MODE 4

#define PLAN_SENSOR_FRAME 0
#define PLAN_JOINT_FRAME 1
#define PLAN_LEG_FRAME 2
#define PLAN_BODY_FRAME 3


struct PlanStruct {
  byte mode;
  byte frame;
  Point3D linear;
  Angle3D angular;
  float speed;
  unsigned long start_time;
  bool active;
  float t_matrix[4][4];  // row column
};


void prepare_plan(PlanStruct *plan, float future_time);
bool follow_plan(PlanStruct plan, Point3D current, Point3D* target, float dt);
#endif
