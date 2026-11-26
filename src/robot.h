// robot.h - a rescue robot: identity, capability, state machine, resources.
//
// Allowed state transitions (enforced by Robot::transition):
//   AVAILABLE  -> TRAVELLING | PERFORMING | RETURNING | RECHARGING | UNAVAILABLE
//   TRAVELLING -> PERFORMING | AVAILABLE | UNAVAILABLE
//   PERFORMING -> AVAILABLE  | UNAVAILABLE
//   RETURNING  -> RECHARGING | AVAILABLE | UNAVAILABLE
//   RECHARGING -> AVAILABLE  | TRAVELLING | PERFORMING | UNAVAILABLE
//   UNAVAILABLE-> AVAILABLE
#ifndef RR_ROBOT_H
#define RR_ROBOT_H

#include "types.h"
#include "vector.h"

namespace rr {

struct Robot {
  int id;
  int location;  // internal index of the last node reached
  int capability;
  int maxBattery;
  int battery;
  int state;  // RobotState

  int request;  // internal request index being served, or -1
  Vector<int> path;  // connection indices still to traverse (travel order)
  int pathPos;       // next entry of path to traverse
  int edgeRemaining; // >0 while part-way along a connection
  int nextNode;      // node the current connection leads to
  int workLeft;      // work steps left while PERFORMING
  int returnTarget;  // charging station (internal index) while RETURNING
  bool batteryBlocked;  // scratch flag set by dispatch (see simulation.cpp)

  // statistics
  int busySteps;    // steps spent TRAVELLING or PERFORMING
  i64 distance;     // total cost units travelled
  i64 batteryUsed;  // battery consumed (travel + work)
  i64 batteryRecharged;
  int tasksDone;

  Robot()
      : id(0), location(0), capability(CAP_GENERAL), maxBattery(1), battery(1),
        state(RS_AVAILABLE), request(-1), pathPos(0), edgeRemaining(0),
        nextNode(-1), workLeft(0), returnTarget(-1), batteryBlocked(false), busySteps(0),
        distance(0), batteryUsed(0), batteryRecharged(0), tasksDone(0) {}

  static bool transitionAllowed(int from, int to);
  bool transition(int to);

  // True when the robot may be given a new request right now.
  bool eligibleForDispatch() const {
    return (state == RS_AVAILABLE || state == RS_RECHARGING) &&
           edgeRemaining == 0 && request == -1;
  }
};

}  
#endif
