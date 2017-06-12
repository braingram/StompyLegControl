/*
	leg.h - Library leg class.
	Created by Brett Graham, June 11, 2017.
	Released into the public domain -- so help you God.
*/

#ifndef LEG_H
#define LEG_H

#include "Arduino.h"

#include <ADC.h>

#include "estop.h"
#include "stompy_pins.h"
#include "geometry.h"
#include "sensors.h"
#include "transforms.h"
#include "point.h"
#include "kinematics.h"
#include "valve.h"
#include "joint.h"


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

class Leg {
  public:
    Leg();
    void set_leg_info(byte side, byte order);
    void update();

  private:
    // TODO make some of these public?
    EStop* _estop;
    ADC* _adc;
    StringPot* _hip_pot;
    StringPot* _thigh_pot;
    StringPot* _knee_pot;
    Valve* _hip_valve;
    Valve* _thigh_valve;
    Valve* _knee_valve;
    Joint* _hip_joint;
    Joint* _thigh_joint;
    Joint* _knee_joint;
    JointAngleTransform* _hip_angle_transform;
    JointAngleTransform* _thigh_angle_transform;
    JointAngleTransform* _knee_angle_transform;
    Kinematics* _kinematics;

    Angle3D joint_angles;
    Point3D foot_position;
};

#endif
