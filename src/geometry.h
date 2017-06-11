#ifndef GEOMETRY_H
#define GEOMETRY_H

// ----------- Hip cylinder --------
#define HIP_CYLINDER_MIN_LENGTH 16.0
#define HIP_CYLINDER_MAX_LENGTH 24.0
#define HIP_CYILNDER_TRAVEL 8.0

// ----------- Thigh cylinder --------
#define THIGH_CYLINDER_MIN_LENGTH 24.0
#define THIGH_CYLINDER_MAX_LENGTH 38.0
#define THIGH_CYILNDER_TRAVEL 14.0

// ----------- Knee cylinder --------
#define KNEE_CYLINDER_MIN_LENGTH 20.0
#define KNEE_CYLINDER_MAX_LENGTH 32.0
#define KNEE_CYILNDER_TRAVEL 12.0

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
#define HIP_ZERO_ANGLE 1.5173462055263942
#define HIP_ZERO_ANGLE_MIDDLE 1.4522610715150865
// thigh 0 is with cylinder fully retracted
#define THIGH_ZERO_ANGLE 0.33189216561617446
// knee 0 is with cylinder fully extended
#define KNEE_ZERO_ANGLE 2.541326289554743  // TODO check this

#define HIP_REST_ANGLE 0.0
#define THIGH_REST_ANGLE 1.4663392033212757
#define KNEE_REST_ANGLE -1.4485440219526171

// these are in meters
//#define HIP_LENGTH 0.279
//#define THIGH_LENGTH 1.3714754828286215
//#define KNEE_LENGTH 1.8286481345518608 
// link lengths in inches?
#define HIP_LENGTH 11.0
#define THIGH_LENGTH 54.0
#define KNEE_LENGTH 72.0 

#define PI 3.141592653589793

#endif
