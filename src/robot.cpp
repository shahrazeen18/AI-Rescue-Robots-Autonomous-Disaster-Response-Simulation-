#include "robot.h"

namespace rr {

bool Robot::transitionAllowed(int from, int to) {
  switch (from) {
    case RS_AVAILABLE:
      return to == RS_TRAVELLING || to == RS_PERFORMING || to == RS_RETURNING ||
             to == RS_RECHARGING || to == RS_UNAVAILABLE;
    case RS_TRAVELLING:
      return to == RS_PERFORMING || to == RS_AVAILABLE || to == RS_UNAVAILABLE;
    case RS_PERFORMING:
      return to == RS_AVAILABLE || to == RS_UNAVAILABLE;
    case RS_RETURNING:
      return to == RS_RECHARGING || to == RS_AVAILABLE || to == RS_UNAVAILABLE;
    case RS_RECHARGING:
      return to == RS_AVAILABLE || to == RS_TRAVELLING || to == RS_PERFORMING ||
             to == RS_UNAVAILABLE;
    case RS_UNAVAILABLE:
      return to == RS_AVAILABLE;
    default:
      return false;
  }
}

bool Robot::transition(int to) {
  if (!transitionAllowed(state, to)) return false;
  state = to;
  return true;
}

}  // namespace rr
