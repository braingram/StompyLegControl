/*
	leg.h - Library leg class.
	Created by Brett Graham, June 11, 2017.
	Released into the public domain -- so help you God.
*/

#ifndef LEG_H
#define LEG_H

#include "Arduino.h"

#include <ADC.h>

#include "defaults.h"
#include "estop.h"
#include "kinematics.h"
#include "geometry.h"
#include "joint.h"
#include "plan.h"
#include "point.h"
#include "sensors.h"
#include "stompy_pins.h"
#include "transforms.h"
#include "valve.h"


/*
 * 1 estop/heartbeat
 * 1 adc
 * 3 string pots
 * 3 joint transforms [encapsulate into 3 joints?]
 * 1 angle3d
 * 1 point3d
 * 3 valves
 * calibration (settable)
 * name/side (make this settable)
 *
 * in constructor
 * - make adc
 * - make string pots
 * - make valves
 * - make joint transforms
 * - make angle3d, point3d
 * - make kinematics
 *
 * load calibration
 * - set side, name, limits
 *
 * update
 * - check estop
 * - update all joints
 */


enum class LEG_NUMBER : uint8_t {
  FL,
  ML,
  RL,
  RR,
  MR,
  FR,
  UNDEFINED,
};


class Leg {
  public:
    Leg();
    void set_leg_number(LEG_NUMBER leg);
    void update();

    LEG_NUMBER leg_number;
    EStop* estop;
    ADC* adc;

    StringPot* hip_pot;
    StringPot* thigh_pot;
    StringPot* knee_pot;

    Valve* hip_valve;
    Valve* thigh_valve;
    Valve* knee_valve;

    Joint* hip_joint;
    Joint* thigh_joint;
    Joint* knee_joint;

    JointAngleTransform* hip_angle_transform;
    JointAngleTransform* thigh_angle_transform;
    JointAngleTransform* knee_angle_transform;

    PID* hip_pid;
    PID* thigh_pid;
    PID* knee_pid;

    Kinematics* kinematics;

    JointAngle3D joint_angles;
    Point3D foot_position;

    Point3D target_position;
    unsigned long last_target_point_generation_time;
    PlanStruct current_plan;
    PlanStruct next_plan;

    void compute_foot_position();
    void set_next_plan(PlanStruct new_plan);

    void disable_valves();
    void enable_valves();
  private:
};

#endif
