// rescue_request.h - a rescue request and its lifecycle.
// COMPLETED and FAILED are terminal. A request has at most one robot at a
// time (robot index), and only IN_PROGRESS -> COMPLETED completes it, so it
// can never be completed twice.
#ifndef RR_RESCUE_REQUEST_H
#define RR_RESCUE_REQUEST_H

#include "types.h"

namespace rr {

struct Request {
  int id;
  int reportTime;
  int location;  // internal location index
  int category;  // Category
  int urgency;   // 1..5
  int duration;  // work steps required
  int state;     // RequestState
  int robot;     // internal robot index currently holding it, or -1
  int reassignments;
  int firstAssignTime;  // -1 until first assigned
  int startTime;        // step of first work unit of the successful attempt
  int doneTime;         // step of the last work unit (COMPLETED) or failure
  bool everStarted;
  int firstStartTime;   // first time any robot began work (for response time)

  Request()
      : id(0), reportTime(0), location(0), category(0), urgency(1),
        duration(1), state(QS_NOT_REPORTED), robot(-1), reassignments(0),
        firstAssignTime(-1), startTime(-1), doneTime(-1), everStarted(false),
        firstStartTime(-1) {}

  static bool transitionAllowed(int from, int to);
  // Performs the transition if legal and returns true; otherwise leaves the
  // state untouched and returns false.
  bool transition(int to);
  bool terminal() const { return state == QS_COMPLETED || state == QS_FAILED; }
};

}  

#endif
