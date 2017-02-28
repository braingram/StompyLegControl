/*
	legcontrol.h - Library for controlling a stompy leg.
	Created by Joel Greenwood, Febuary 26, 2017.
	Released into the public domain -- so help you God.
*/

#ifndef legControl_h
#define legControl_h

#include "Arduino.h"

class legControl
{
public:
	legControl();
	int angleToSensor(int joint, float angle);
	float sensorToAngle(int joint, int sensorReading);
};

#endif