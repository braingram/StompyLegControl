#include <math.h>
#include "plan.h"

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
  /*
    case (PLAN_ARC_MODE):
      // TODO not implemented
      return false;
    case (PLAN_TARGET_MODE):
      // TODO not implemented
      return false;
  }
  */
  return false;
}
