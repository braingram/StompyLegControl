/*
	joint.h - Leg joint class.
	Created by Brett Graham, June 11, 2017.
	Released into the public domain -- so help you God.
*/

#ifndef JOINT_H
#define JOINT_H

#include <PID_v1.h>
#include "Arduino.h"
#include "valve.h"
#include "sensors.h"
#include "transforms.h"


class Joint {
  public:
    Joint(Valve* valve, StringPot* pot, JointAngleTransform* angle_transform);

    bool set_target_angle(float angle);
    bool set_target_length(float length);
    bool set_target_adc_value(unsigned int);

    float get_target_angle();
    float get_target_length();
    unsigned int get_target_adc_value();

    void update();
  private:
    Valve* _valve;
    StringPot* _pot;
    JointAngleTransform* _angle_transform;
    PID* _pid;

    float _target_angle;
    float _target_length;
    double _target_adc_value;
    double _current_adc_value;
    double _pid_output;
};

#endif