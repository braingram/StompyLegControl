#include <math.h>
#include "plan.h"

void prepare_plan(PlanStruct *plan) {
  if (plan->mode == PLAN_ARC_MODE) {
    // build transformation matrix
    float cx = cos(plan->angular.x * plan->speed);
    float sx = sin(plan->angular.x * plan->speed);
    float cy = cos(plan->angular.y * plan->speed);
    float sy = sin(plan->angular.y * plan->speed);
    float cz = cos(plan->angular.z * plan->speed);
    float sz = sin(plan->angular.z * plan->speed);
    plan->t_matrix[0][0] = cz * cy;
    plan->t_matrix[0][1] = cz*sx*sy-sz*cx;
    plan->t_matrix[0][2] = cx*sy*cz+sx*sz;
    plan->t_matrix[0][3] = (
      -plan->linear.x*cz*cy +
      -plan->linear.y*(cz*sx*sy-sz*cx) +
      -plan->linear.z*(cx*sy*cz+sx*sz) +
      plan->linear.x);
    plan->t_matrix[1][0] = sz*cy;
    plan->t_matrix[1][1] = sx*sy*sz+cx*cz;
    plan->t_matrix[1][2] = cx*sy*sz-sx*cz;
    plan->t_matrix[1][3] = (
      -plan->linear.x*sz*cy +
      -plan->linear.y*(sx*sy*sz+cx*cz) +
      -plan->linear.z*(cx*sy*sz-sx*cz) +
      plan->linear.y);
    plan->t_matrix[2][0] = -sy;
    plan->t_matrix[2][1] = sx*cy;
    plan->t_matrix[2][2] = cx*cy;
    plan->t_matrix[2][3] = (
      -plan->linear.x*-sy +
      -plan->linear.y*sx*cy +
      -plan->linear.z*cx*cy +
      plan->linear.z);
  };
};

bool follow_plan(PlanStruct plan, Point3D current, Point3D* target, float dt) {
  if (plan.mode == PLAN_STOP_MODE) {
    target->x = current.x;
    target->y = current.y;
    target->z = current.z;
    return true;
  }
  if (plan.mode == PLAN_VELOCITY_MODE) {
    float ss = plan.speed * dt;
    target->x = current.x + plan.linear.x * ss;
    target->y = current.y + plan.linear.y * ss;
    target->z = current.z + plan.linear.z * ss;
    return true;
  }
  if (plan.mode == PLAN_TARGET_MODE) {
    // compute delta, move in direction of target
    // (target - current) * ss
    float ss = plan.speed * dt;
    target->x = (plan.linear.x - current.x);
    target->y = (plan.linear.y - current.y);
    target->z = (plan.linear.z - current.z);
    // compute magnitude
    float mag = sqrt(
        target->x * target->x +
        target->y * target->y +
        target->z * target->z);
    // if mag is large, scale down
    if (mag > 1.0) {
      target->x /= mag;
      target->y /= mag;
      target->z /= mag;
    }
    // if length < 1, don't scale
    // if mag very small, hold?
    target->x = current.x + target->x * ss;
    target->y = current.y + target->y * ss;
    target->z = current.z + target->z * ss;
    return true;
  }
  if (plan.mode == PLAN_ARC_MODE) {
    target->x = (
        current.x * plan.t_matrix[0][0] +
        current.y * plan.t_matrix[0][1] +
        current.z * plan.t_matrix[0][2] +
        plan.t_matrix[0][3]);
    target->y = (
        current.x * plan.t_matrix[1][0] +
        current.y * plan.t_matrix[1][1] +
        current.z * plan.t_matrix[1][2] +
        plan.t_matrix[1][3]);
    target->z = (
        current.x * plan.t_matrix[2][0] +
        current.y * plan.t_matrix[2][1] +
        current.z * plan.t_matrix[2][2] +
        plan.t_matrix[2][3]);
    return true;
  }
  return false;
}
