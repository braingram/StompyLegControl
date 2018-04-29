#ifndef GEOMETRY_H
#define GEOMETRY_H

// ----------- Hip cylinder --------
#define HIP_CYLINDER_MIN_LENGTH 16.0
#define HIP_CYLINDER_MAX_LENGTH_MIDDLE 24.0
#define HIP_CYLINDER_MAX_LENGTH 23.153837
#define HIP_CYLINDER_TRAVEL_MIDDLE 8.0
#define HIP_CYLINDER_TRAVEL 7.153837

// ----------- Thigh cylinder --------
#define THIGH_CYLINDER_MIN_LENGTH 24.0
#define THIGH_CYLINDER_MAX_LENGTH 38.0
#define THIGH_CYLINDER_TRAVEL 14.0

// ----------- Knee cylinder --------
#define KNEE_CYLINDER_MIN_LENGTH 20.0
#define KNEE_CYLINDER_MAX_LENGTH 32.0
#define KNEE_CYLINDER_TRAVEL 12.0

// ----- leg kinematics triangles -----
// A is short static side
// B is long static side
// C is cylinder side
// cos(C) = (a * a + b * b - c * c) / (2 * a * b)
// c * c = a * a + b * b - (2 * a * b) * cos(C)
#define HIP_A 6.83905
#define HIP_B_MIDDLE 19.62051 // middle legs
#define HIP_B 19.16327 // front/back legs
#define THIGH_A 10.21631
#define THIGH_B 33.43093
#define KNEE_A 7.4386
#define KNEE_B 25.6021

// ----- 'zero' angles in radians -----
// hip 0 is with cylinder at half travel
// when front/rear hip cylinder is at 19.80315 hip is at zero
//#define HIP_ZERO_ANGLE 1.4228032681870073
// when front/rear hip cylinder is at 19.8173 hip is at zero
#define HIP_ZERO_ANGLE 1.4895289069969002
// when middle hip cylinder is at 20.314961 hip is at zero
//#define HIP_ZERO_ANGLE_MIDDLE 1.4997925172727815
// when middle hip cylinder is at 20.30 hip is at zero
#define HIP_ZERO_ANGLE_MIDDLE 1.4975224344157239
// thigh 0 is with cylinder fully retracted
#define THIGH_ZERO_ANGLE 0.33189216561617446
// knee 0 is with cylinder fully extended
#define KNEE_ZERO_ANGLE 2.541326289554743  // TODO check this

#define HIP_REST_ANGLE 0.0
#define THIGH_REST_ANGLE 1.4663392033212757
#define KNEE_REST_ANGLE -1.4485440219526171
// BASE_BETA = PI - THIGH_REST_ANGLE - KNEE_REST_ANGLE
#define BASE_BETA 0.22670942831590035

#define HIP_MIN_ANGLE -0.56583574349656163
#define HIP_MAX_ANGLE 0.56583574349656163 
#define HIP_MIN_ANGLE_MIDDLE -0.64071036840711837
#define HIP_MAX_ANGLE_MIDDLE 0.64071036840711837
#define THIGH_MIN_ANGLE 0.0
#define THIGH_MAX_ANGLE 1.5708
#define KNEE_MIN_ANGLE -2.3736
#define KNEE_MAX_ANGLE 0.0

// these are in meters
//#define HIP_LENGTH 0.279
//#define THIGH_LENGTH 1.3714754828286215
//#define KNEE_LENGTH 1.8286481345518608 
// link lengths in inches?
#define HIP_LENGTH 11.0
#define THIGH_LENGTH 54.0
#define THIGH_LENGTH_SQUARED 2916.0
#define KNEE_LENGTH 72.0 
#define KNEE_LENGTH_SQUARED 5184.0 
#define KNEE_THIGH_MIN_2 -7776  // KNEE_LENGTH * THIGH_LENGTH * -2

//#define PI 3.141592653589793

#define CALF_LINK_LENGTH 8.0  // a
#define CALF_TO_SHOCK_LENGTH 18.0  // b
#define CALF_SHOCK_BASE_LENGTH 18.0
// TODO get these for all legs
// assuming full range of sensor 0-65535 and 2000 lb load
#define CALF_SENSOR_SLOPE -0.0000083891708147599049
#define CALF_SENSOR_OFFSET 1.3386050049460962  // radians(~77)
#define CALF_INCHES_TO_LBS 600.0

#endif
