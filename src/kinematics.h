/*
	kinematics.h - Library for leg kinematics.
	Created by Brett Graham, June 4, 2017.
	Released into the public domain -- so help you God.

  Angles [gazebo]
    hip angle
        looking down on robot + is CCW
        0 resting (straight out)
    thigh angle
        looking at side with leg pointed right, + tilts down
        0 resting
        angles always +
    knee angle
        looking at side with leg pointed right, - tilts up
        0 resting
        angles always -
  Positions [gazebo] in leg space
    x
        looking at side with leg pointed right + is out (right)
        never -
        minimum is... (depends on z)
    y
        looking down on leg pointed right + is up, - is down
        positive and negative
    z
        looking at side with leg pointed right, - is down
        0 is in line with hip joint, below this is -


  All hip string pots are at REAR
    - for right legs, increasing cylinder length should increase angle
    - for left legs, increasing cylinder length should decrease angle
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"

#include "point.h"


class Kinematics {
  public:
    // TODO
    // - different hip types
    // - different sides (right, left)
    // - input should be string pot lengths
    Kinematics(bool on_right_side);
    /*

    angles to xyz
    JointAngleTransforms: (zero angles?)
      hip_length_to_angle(length)
      thigh_length_to_angle(length)
      knee_length_to_angle(length)

      angles_to_xyz
      xyz_to_angles

    Sensor to angle?

    angle_to_sensor(joint, angle)
    sensor_to_angle(joint, sensor_reading)
    angle_to_rad(angle)
    angles_to_xyz(angles, xyz)
    angles_rad_to_xyz(angles_rad, xyz)
    xyz_to_angles(xyz, angles)
    xyz_to_sensors(xyz, sensors)
    sensors_to_xyz(sensors, xyz)
    */

    bool angles_to_xyz(
        float hip, float thigh, float knee,
        float* x, float* y, float* z);
    bool angles_to_xyz(JointAngle3D angles, Point3D* point);

    bool xyz_to_angles(
        float x, float y, float z,
        float *hip, float* thigh, float* knee);
    bool xyz_to_angles(Point3D point, JointAngle3D* angles);

  private:
    bool _on_right_side;
};

#endif
