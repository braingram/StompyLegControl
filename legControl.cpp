/*
	legcontrol.h - Library for controlling a stompy leg.
	Created by Joel Greenwood, Febuary 26, 2017.
	Released into the public domain -- so help you God.
*/

#include "Arduino.h"
#include "legControl.h"

legControl::legControl()
{
	//no setup here for now
}

int legControl::angleToSensor(int joint, float angle) 
{
	static int cylinderMinLength; 
	static int cylinderMaxLength;
	static float cylinderTravel;
	float currentCylinderLength;
	static float C1; //This is the length of one side of the triangle - not the cylinder
	static float C2; ; //This is the other side
	static float beta; //This is a constant angle between the changing angle calculated here (alpha) and the desired angle - e.g. theta1 for hip
	float alpha; //This is the angle that is changing when the cylinder length changes
	static float deadBugTheta; //This is the desired angle (e.g. theta2 plus beta and alpha) at deadbug (fully retracted and centered)
	static float sensorMax;
	static float sensorMin;
	float sensorGoal; //This is the final output from joint and angle input

	switch (joint) {
		case 0:
			//Serial.println("HIP");
			sensorMax = 722.00;
			sensorMin = 93.00;
			cylinderMinLength = 16;
			cylinderTravel = 8;
			//  cylinderMaxLength = cylinderMinLength + cylinderTravel;
			C1 = 6.83905;
			C2 = 19.62051;
			beta = 77.92503;
			deadBugTheta = -7.8707;
			break;
	        case 1:
			//Serial.println("THIGH");
			sensorMax = 917.00;
			sensorMin = 34.00;
			cylinderMinLength = 24;
			cylinderTravel = 14;
			//  cylinderMaxLength = cylinderMinLength + cylinderTravel;
			C1 = 10.21631;
			C2 = 33.43093;
			beta = 26.6594;
			deadBugTheta = 129.6249;
			break;
		case 2:
			//Serial.println("KNEE");
			sensorMax = 934.00;
			sensorMin = 148.00;
			cylinderMinLength = 20;
			cylinderTravel = 12;
			//  cylinderMaxLength = cylinderMinLength + cylinderTravel;
			C1 = 25.6021;
			C2 = 7.4386;
			beta = 35.8658;
			deadBugTheta = 194.1805;
			break;
	}
      
	//calculate alpha (the internal angle opposite the cylinder in the cylinder triangle
	if (joint == 0) { 
	        //The hip is inverted logic because the sensor is at minimum when the joint is at minimum angle 
	        //The other joints are at their maximum angle whne the sensor is fully retracted                
	        alpha = angle - deadBugTheta + beta;
	}
	else {
		alpha = deadBugTheta - beta - angle;
	}
	float alphaR = (alpha * 71) / 4068;
	//Serial.print("alpha: ");
	//Serial.println(alpha);

	//calculate cylinder length from alpha
	float cylinderGoalLength = sqrt((sq(C2) + sq(C1) - (2*C1*C2*cos(alphaR))));
	float pistonGoal = cylinderGoalLength - cylinderMinLength;
	float sensorUnitsPerInch = (sensorMax - sensorMin) / (cylinderTravel);
	//Serial.print("sensor units per inch: ");
	//Serial.println(sensorUnitsPerInch);
	sensorGoal = (pistonGoal * sensorUnitsPerInch) + sensorMin;

	return sensorGoal;
}

float legControl::sensorToAngle(int joint, int sensorReading) 
{
	static int cylinderMinLength; 
	static int cylinderMaxLength;
	static float cylinderTravel;
	float currentCylinderLength;
	static float C1; //This is the length of one side of the triangle - not the cylinder
	static float C2; ; //This is the other side
	static float beta; //This is a constant angle between the changing angle calculated here (alpha) and the desired angle - e.g. theta1 for hip
	float alpha; //This is the angle that is changing when the cylinder length changes
	static float deadBugTheta; //This is the desired angle (e.g. theta2 plus beta and alpha) at deadbug (fully retracted and centered)
	float theta; //This is the final ouput angle 
	static int sensorMax;
	static int sensorMin;

	switch (joint) {
        case 0:
		//Serial.println("HIP");
		sensorMax = 722;
		sensorMin = 93;
		cylinderMinLength = 16;
		cylinderTravel = 8;
		C1 = 6.83905;
		C2 = 19.62051;
		beta = 77.92503;
		deadBugTheta = -7.8707;
		break;
        case 1:
		//Serial.println("THIGH");
		sensorMax = 917;
		sensorMin = 34;
		cylinderMinLength = 24;
		cylinderTravel = 14;
		C1 = 10.21631;
		C2 = 33.43093;
		beta = 26.6594;
		deadBugTheta = 129.6249;
		break;
        case 2:
		//Serial.println("KNEE");
		sensorMax = 934;
		sensorMin = 148;
		cylinderMinLength = 20;
		cylinderTravel = 12;
		C1 = 25.6021;
		C2 = 7.4386;
		beta = 35.8658;
		deadBugTheta = 194.1805;
		break;
	}
	float sensorUnitsPerInch = (sensorMax - sensorMin) / (cylinderTravel);
	//      Serial.print("sensor units per inch: ");
	//      Serial.println(sensorUnitsPerInch);
	int sensorReading_sensorMin = sensorReading - sensorMin;
	//      Serial.print("sensorReading_sensorMin: ");
	//      Serial.println(sensorReading_sensorMin);
	currentCylinderLength = ((sensorReading - sensorMin) / sensorUnitsPerInch) + cylinderMinLength;
	//      Serial.print("currentCylindarLength: ");
	//      Serial.println(currentCylinderLength);
	float alphaR = acos((sq(C1) + sq(C2) - sq(currentCylinderLength))/(2*C1*C2));
	alpha = (alphaR * 4068) / 71;
	if (joint == 0) { 
		//The hip is inverted logic because the sensor is at minimum when the joint is at minimum angle 
		//The other joints are at their maximum angle whne the sensor is fully retracted                
		theta = deadBugTheta - beta + alpha;
	}
	else {
		theta = deadBugTheta - beta - alpha;
	}
	//      Serial.print("alpha: ");
	//      Serial.println(alpha);
	//      Serial.print("theta in void: ");
	//      Serial.println(theta);
	// float thetaR = (theta * 71) / 4068;
	return theta;
}

void legControl::anglesToRad(float angles[], float anglesRad[]) {
	for(int i = 0; i < 3; i ++) {
		anglesRad[i] = ((angles[i] * 71)/4068);
	}
}

void legControl::anglesToXYZ(float angles[], float xyz[]) {
	float anglesRad[3];
	for(int i = 0; i < 3; i ++) {
		anglesRad[i] = ((angles[i] * 71)/4068);
	}
	const float pi = 3.141593;
	const int L1 = 11; //leg link lengths hip, thigh, and knee
	const int L2 = 54;
	const int L3 = 72; //72 inches is from knee joint to ankle joint
	Serial.println();
	Serial.println("entered xyz calculation");
	
	xyz[0] = cos(anglesRad[0]) * (L1 + L2*cos(anglesRad[1]) + L3*cos(anglesRad[1] + anglesRad[2] - pi));
	xyz[1] = xyz[0] * tan(anglesRad[0]);
	xyz[2] = (L2 * sin(anglesRad[1])) + (L3 * sin(anglesRad[1] + anglesRad[2] - pi));
}

void legControl::anglesRadToXYZ(float anglesRad[], float xyz[]) {

	const float pi = 3.141593;
	const int L1 = 11; //leg link lengths hip, thigh, and knee
	const int L2 = 54;
	const int L3 = 72; //72 inches is from knee joint to ankle joint
	Serial.println();
	Serial.println("entered xyz calculation");
	
	xyz[0] = cos(anglesRad[0]) * (L1 + L2*cos(anglesRad[1]) + L3*cos(anglesRad[1] + anglesRad[2] - pi));
	xyz[1] = xyz[0] * tan(anglesRad[0]);
	xyz[2] = (L2 * sin(anglesRad[1])) + (L3 * sin(anglesRad[1] + anglesRad[2] - pi));
}