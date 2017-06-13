
#include <math.h>
#include "geometry.h"
#include "kinematics.h"

Kinematics::Kinematics(bool on_right_side) {
  _on_right_side = on_right_side;
}

bool Kinematics::angles_to_xyz(float hip, float thigh, float knee, float* x, float* y, float* z) {
  // TODO range check
  *x = HIP_LENGTH;
  *y = 0;
  *z = 0;

  float a = THIGH_REST_ANGLE - thigh;
  *x += THIGH_LENGTH * cos(a);
  *z += THIGH_LENGTH * sin(a);

  a = KNEE_REST_ANGLE - knee - thigh;
  *x += KNEE_LENGTH * cos(a);
  *z += KNEE_LENGTH * sin(a);

  *y = *x * sin(hip);
  *x *= cos(hip);
  return true;
};

bool Kinematics::angles_to_xyz(Angle3D angles, Point3D* point) {
  return angles_to_xyz(angles.hip, angles.thigh, angles.knee, &(point->x), &(point->y), &(point->z));
};

bool Kinematics::xyz_to_angles(float x, float y, float z, float* hip, float* thigh, float* knee) {
  if (x <= HIP_LENGTH) return false;
  float l = sqrt(x * x + y * y);

  *hip = atan2(y, x);

  float L = sqrt(z * z + (l - HIP_LENGTH) * (l - HIP_LENGTH));

  float a1 = acos(-z / L);
  //float a2 = acos(
  //    (KNEE_LENGTH * KNEE_LENGTH - THIGH_LENGTH * THIGH_LENGTH - L * L) /
  //    (-2 * THIGH_LENGTH * L));
  float a2 = acos(
      (KNEE_LENGTH_SQUARED - THIGH_LENGTH_SQUARED - L * L) /
      (-2 * THIGH_LENGTH * L));

  float alpha = (a1 + a2);

  //float beta = acos(
  //    (L * L - KNEE_LENGTH_SQUARED - THIGH_LENGTH_SQUARED) /
  //    (-2 * KNEE_LENGTH * THIGH_LENGTH));
  float beta = acos(
      (L * L - KNEE_LENGTH_SQUARED - THIGH_LENGTH_SQUARED) /
      (KNEE_THIGH_MIN_2));

  *thigh = THIGH_REST_ANGLE - (alpha - PI / 2.);
  //float base_beta = PI - THIGH_REST_ANGLE + KNEE_REST_ANGLE;
  *knee = BASE_BETA - beta;
  // TODO check ranges, return false if out-of-bounds
  return true;
};

bool Kinematics::xyz_to_angles(Point3D point, Angle3D* angles) {
  return xyz_to_angles(point.x, point.y, point.z, &(angles->hip), &(angles->thigh), &(angles->knee));
};

/*
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
			sensorMax = hip_sensorMax;
			sensorMin = hip_sensorMin;
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
			sensorMax = thigh_sensorMax;
			sensorMin = thigh_sensorMin;
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
			sensorMax = knee_sensorMax;
			sensorMin = knee_sensorMin;
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
	        //The other joints are at their maximum angle when the sensor is fully retracted                
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

float legControl::sensorToAngle(int joint, float sensorReading) 
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
		sensorMax = hip_sensorMax; // 722 in 10 bit -- 2888 in 12 bit
		sensorMin = hip_sensorMin; // 93 in 10 bit -- 372 in 12 bit
		cylinderMinLength = 16;
		cylinderTravel = 8;
		C1 = 6.83905;
		C2 = 19.62051;
		beta = 77.92503;
		deadBugTheta = -7.8707;
		break;
        case 1:
		//Serial.println("THIGH");
		sensorMax = thigh_sensorMax; //917 in 10 bit -- 3668 in 12 bit
		sensorMin = thigh_sensorMin; //34 in 10 bit -- 136 in 12 bit
		cylinderMinLength = 24;
		cylinderTravel = 14;
		C1 = 10.21631;
		C2 = 33.43093;
		beta = 26.6594;
		deadBugTheta = 129.6249;
		break;
        case 2:
		//Serial.println("KNEE");
		sensorMax = knee_sensorMax; //934 in 10bit -- 3736 in 12 bit
		sensorMin = knee_sensorMin; //148 in 10bit -- 592 in 12 bit
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
	float sensorReading_sensorMin = sensorReading - sensorMin;
	//      Serial.print("sensorReading_sensorMin: ");
	//      Serial.println(sensorReading_sensorMin);
	currentCylinderLength = ((sensorReading - sensorMin) / sensorUnitsPerInch) + cylinderMinLength;
	//      Serial.print("currentCylindarLength: ");
	//      Serial.println(currentCylinderLength);
	float alphaR = acos((sq(C1) + sq(C2) - sq(currentCylinderLength))/(2*C1*C2));
	alpha = (alphaR * 4068) / 71;
	if (joint == 0) { 
		//The hip is inverted logic because the sensor is at minimum when the joint is at minimum angle 
		//The other joints are at their maximum angle when the sensor is fully retracted                
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

float legControl::angleToRad(float angle) {
	return (angle * 71) / 4068;
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
	//Serial.println();
	//Serial.println("entered xyz calculation");
	
	xyz[0] = cos(anglesRad[0]) * (L1 + L2*cos(anglesRad[1]) + L3*cos(anglesRad[1] + anglesRad[2] - pi));
	xyz[1] = xyz[0] * tan(anglesRad[0]);
	xyz[2] = (L2 * sin(anglesRad[1])) + (L3 * sin(anglesRad[1] + anglesRad[2] - pi));
}

void legControl::anglesRadToXYZ(float anglesRad[], float xyz[]) {

	const float pi = 3.141593;
	const int L1 = 11; //leg link lengths hip, thigh, and knee
	const int L2 = 54;
	const int L3 = 72; //72 inches is from knee joint to ankle joint
	//Serial.println();
	//Serial.println("entered xyz calculation");
	
	xyz[0] = cos(anglesRad[0]) * (L1 + L2*cos(anglesRad[1]) + L3*cos(anglesRad[1] + anglesRad[2] - pi));
	xyz[1] = xyz[0] * tan(anglesRad[0]);
	xyz[2] = (L2 * sin(anglesRad[1])) + (L3 * sin(anglesRad[1] + anglesRad[2] - pi));
}

void legControl::xyzToAngles(float xyz[], float angles[]) {
	
	//Serial.println();
	//Serial.print("angle zero from xyzToAngles");
	//Serial.println(angles[1]);


	const float pi = 3.141593;
	const int L1 = 11; //leg link lengths hip, thigh, and knee
	const int L2 = 54;
	const int L3 = 72; //72 inches is from knee joint to ankle joint

	//theta1
		float theta1R = atan(xyz[1]/xyz[0]);
		//convert to degrees
		float theta1 = (theta1R * 4068) / 71;
		//Serial.print("theta1 = ");
		//Serial.println(theta1);
	//End theta1

	//theta2
		float r;
		float x1;
		if (theta1R == 0) {
			x1 = (xyz[0] - L1);
		}
		else {
			x1 = (xyz[1]/sin(theta1R)) - L1;
		} 
		x1 = abs(x1);    
		float beta = atan(xyz[2]/x1);
		if (xyz[0] == L1) {
			beta = -(pi/2);
		}
		else if (xyz[0] < L1) {
			if (xyz[2] == 0) {
				r = x1;
		        }
		        else {
				r = xyz[2]/sin(beta);
		        }
			r = abs(r);
			float gama = asin(x1/r);
			beta = -(gama + pi/2);
			}
		else {
			beta = atan(xyz[2]/x1);
		}
		if (xyz[2] == 0) {
			r = x1;
		}
		else {
			r = xyz[2]/sin(beta);
		}
		r = abs(r); 
		float theta2R = beta + acos((sq(L2) + sq(r) - sq(L3))/(2*L2*r));
		float theta2 = (theta2R * 4068) / 71;
		//Serial.print("theta2 = ");
		//Serial.println(theta2);	
	//End theta2	

	//theta3
		float theta3R = acos((sq(L3) + sq(L2) - sq(r)) / (2*L3*L2));
		float theta3 = (theta3R * 4068) / 71;
		//Serial.print("theta3 = ");
		//Serial.print(theta3);
	//End theta3

	angles[0] = theta1;
	angles[1] = theta2;
	angles[2] = theta3;
}

void legControl::xyzToSensors(float xyz[], float sensorGoals[]) {
	float angles_tmp[3];
	legControl::xyzToAngles(xyz, angles_tmp);
	for (int i = 0; i < 3; i ++) {
		sensorGoals[i] = legControl::angleToSensor(i, angles_tmp[i]);
		//Serial.print(sensorGoals[i]);
		//Serial.print('\t');
	}
	//Serial.println();
}

void legControl::sensors_to_xyz(float sensors[], float xyz[]) {
	float tmpAngles[3];
	for (int i = 0; i < 3; i ++) {
		tmpAngles[i] = legControl::sensorToAngle(i, sensors[i]);
	}
	legControl::anglesToXYZ(tmpAngles, xyz);
}
*/
