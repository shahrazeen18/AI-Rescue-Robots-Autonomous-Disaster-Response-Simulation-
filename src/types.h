// types.h - shared enumerations and name<->enum conversion.
#ifndef RR_TYPES_H
#define RR_TYPES_H

namespace rr {

typedef long long i64;

// Request categories. Numeric values 0..3 are shared with Capability so a
// specialist robot serves a request iff the numbers match.
enum Category { CAT_MEDICAL = 0, CAT_FIRE = 1, CAT_SUPPLY = 2, CAT_SEARCH = 3 };
const int NUM_CATEGORIES = 4;

enum Capability {
  CAP_MEDICAL = 0,
  CAP_FIRE = 1,
  CAP_SUPPLY = 2,
  CAP_SEARCH = 3,
  CAP_GENERAL = 4
};

enum LocType {
  LOC_JUNCTION,
  LOC_BUILDING,
  LOC_HOSPITAL,
  LOC_DEPOT,
  LOC_CHARGING,
  LOC_COMMAND
};

enum RobotState {
  RS_AVAILABLE,
  RS_TRAVELLING,
  RS_PERFORMING,
  RS_RETURNING,
  RS_RECHARGING,
  RS_UNAVAILABLE
};
const int NUM_ROBOT_STATES = 6;

enum RequestState {
  QS_NOT_REPORTED,
  QS_PENDING,
  QS_ASSIGNED,
  QS_IN_PROGRESS,
  QS_COMPLETED,
  QS_FAILED
};
const int NUM_REQUEST_STATES = 6;

enum EventType {
  EV_BLOCK,
  EV_UNBLOCK,
  EV_STATION_DOWN,
  EV_STATION_UP,
  EV_SET_URGENCY,
  EV_ROBOT_OFFLINE,
  EV_ROBOT_ONLINE
};

// Parsers return -1 when the text is not a valid name.
int parseCategory(const char* s);
int parseCapability(const char* s);
int parseLocType(const char* s);
int parseEventType(const char* s);

const char* categoryName(int c);
const char* capabilityName(int c);
const char* locTypeName(int t);
const char* robotStateName(int s);
const char* requestStateName(int s);
const char* eventTypeName(int e);

inline bool canServe(int capability, int category) {
  return capability == CAP_GENERAL || capability == category;
}

}  

#endif
