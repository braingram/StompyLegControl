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
