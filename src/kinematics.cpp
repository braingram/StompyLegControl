
#include <math.h>
#include "geometry.h"
#include "kinematics.h"

Kinematics::Kinematics(LEG_NUMBER leg_number) {
  // make default joints
  joints[HIP_INDEX].min_angle = HIP_MIN_ANGLE;
  joints[HIP_INDEX].max_angle = HIP_MAX_ANGLE;
  joints[HIP_INDEX].rest_angle = HIP_REST_ANGLE;
  joints[HIP_INDEX].length = HIP_LENGTH;

  joints[THIGH_INDEX].min_angle = THIGH_MIN_ANGLE;
  joints[THIGH_INDEX].max_angle = THIGH_MAX_ANGLE;
  joints[THIGH_INDEX].rest_angle = THIGH_REST_ANGLE;
  joints[THIGH_INDEX].length = THIGH_LENGTH;

  joints[KNEE_INDEX].min_angle = KNEE_MIN_ANGLE;
  joints[KNEE_INDEX].max_angle = KNEE_MAX_ANGLE;
  joints[KNEE_INDEX].rest_angle = KNEE_REST_ANGLE;
  joints[KNEE_INDEX].length = KNEE_LENGTH;

  set_leg_number(leg_number);
  recompute();
}

void Kinematics::recompute() {
    _knee_length_squared = joints[KNEE_INDEX].length * joints[KNEE_INDEX].length;
    _thigh_length_squared = joints[THIGH_INDEX].length * joints[THIGH_INDEX].length;
    _knee_thigh_min_2 = joints[KNEE_INDEX].length * joints[THIGH_INDEX].length * -2.0;
    _base_beta = PI - joints[THIGH_INDEX].rest_angle + joints[KNEE_INDEX].rest_angle;
}

void Kinematics::set_leg_number(LEG_NUMBER leg_number) {
  _leg_number = leg_number;
  if ((leg_number == LEG_NUMBER::ML) || 
      (leg_number == LEG_NUMBER::MR)) {
    joints[HIP_INDEX].min_angle = HIP_MIN_ANGLE_MIDDLE;
    joints[HIP_INDEX].max_angle = HIP_MAX_ANGLE_MIDDLE;
  } else {
    joints[HIP_INDEX].min_angle = HIP_MIN_ANGLE;
    joints[HIP_INDEX].max_angle = HIP_MAX_ANGLE;
  };
  recompute();
}

bool Kinematics::angles_in_limits(float hip, float thigh, float knee) {
  if (hip < joints[HIP_INDEX].min_angle) return false;
  if (hip > joints[HIP_INDEX].max_angle) return false;
  if (thigh < joints[THIGH_INDEX].min_angle) return false;
  if (thigh > joints[THIGH_INDEX].max_angle) return false;
  if (knee < joints[KNEE_INDEX].min_angle) return false;
  if (knee > joints[KNEE_INDEX].max_angle) return false;
  return true;
}

bool Kinematics::angles_to_xyz(float hip, float thigh, float knee, float* x, float* y, float* z) {
  // range check
  if (!angles_in_limits(hip, thigh, knee)) return false;
  *x = joints[HIP_INDEX].length;
  *y = 0;
  *z = 0;

  float a = joints[THIGH_INDEX].rest_angle - thigh;
  *x += joints[THIGH_INDEX].length * cosf(a);
  *z += joints[THIGH_INDEX].length * sinf(a);

  a = joints[KNEE_INDEX].rest_angle - knee - thigh;
  *x += joints[KNEE_INDEX].length * cosf(a);
  *z += joints[KNEE_INDEX].length * sinf(a);

  *y = *x * sinf(hip);
  *x *= cosf(hip);
  return true;
};

bool Kinematics::angles_to_xyz(JointAngle3D angles, Point3D* point) {
  return angles_to_xyz(angles.hip, angles.thigh, angles.knee, &(point->x), &(point->y), &(point->z));
};

bool Kinematics::xyz_to_angles(float x, float y, float z, float* hip, float* thigh, float* knee) {
  if (x <= joints[HIP_INDEX].length) return false;
  //float l = sqrtf(x * x + y * y);
  float l = hypotf(x, y);

  *hip = atan2f(y, x);

  //float L = sqrtf(z * z + (l - HIP_LENGTH) * (l - HIP_LENGTH));
  float L = hypotf(z, l - joints[HIP_INDEX].length);

  float a1 = acosf(-z / L);
  float a2 = acosf(
      (_knee_length_squared - _thigh_length_squared - L * L) /
      (-2 * joints[THIGH_INDEX].length * L));

  float alpha = (a1 + a2);

  float beta = acosf(
      (L * L - _knee_length_squared - _thigh_length_squared) /
      (_knee_thigh_min_2));

  *thigh = joints[THIGH_INDEX].rest_angle - (alpha - PI / 2.);
  //float base_beta = PI - THIGH_REST_ANGLE + KNEE_REST_ANGLE;
  *knee = _base_beta - beta;
  // check ranges, return false if out-of-bounds
  return angles_in_limits(*hip, *thigh, *knee);
};

bool Kinematics::xyz_to_angles(Point3D point, JointAngle3D* angles) {
  return xyz_to_angles(point.x, point.y, point.z, &(angles->hip), &(angles->thigh), &(angles->knee));
};
