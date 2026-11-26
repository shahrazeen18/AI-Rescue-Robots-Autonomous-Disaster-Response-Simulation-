#include "types.h"

#include <cstring>

using namespace std;

namespace rr {

namespace {

const char* const kCategoryNames[] = {"MEDICAL", "FIRE", "SUPPLY", "SEARCH"};
const char* const kCapabilityNames[] = {"MEDICAL", "FIRE", "SUPPLY", "SEARCH",
                                        "GENERAL"};
const char* const kLocTypeNames[] = {"JUNCTION", "BUILDING", "HOSPITAL",
                                     "DEPOT",    "CHARGING", "COMMAND"};
const char* const kRobotStateNames[] = {"AVAILABLE",  "TRAVELLING",
                                        "PERFORMING", "RETURNING",
                                        "RECHARGING", "UNAVAILABLE"};
const char* const kRequestStateNames[] = {"NOT_REPORTED", "PENDING",
                                          "ASSIGNED",     "IN_PROGRESS",
                                          "COMPLETED",    "FAILED"};
const char* const kEventTypeNames[] = {"BLOCK",        "UNBLOCK",
                                       "STATION_DOWN", "STATION_UP",
                                       "SET_URGENCY",  "ROBOT_OFFLINE",
                                       "ROBOT_ONLINE"};

int lookup(const char* const* names, int n, const char* s) {
  for (int i = 0; i < n; ++i) {
    if (strcmp(names[i], s) == 0) return i;
  }
  return -1;
}

}  // namespace

int parseCategory(const char* s) { return lookup(kCategoryNames, 4, s); }
int parseCapability(const char* s) { return lookup(kCapabilityNames, 5, s); }
int parseLocType(const char* s) { return lookup(kLocTypeNames, 6, s); }
int parseEventType(const char* s) { return lookup(kEventTypeNames, 7, s); }

const char* categoryName(int c) { return kCategoryNames[c]; }
const char* capabilityName(int c) { return kCapabilityNames[c]; }
const char* locTypeName(int t) { return kLocTypeNames[t]; }
const char* robotStateName(int s) { return kRobotStateNames[s]; }
const char* requestStateName(int s) { return kRequestStateNames[s]; }
const char* eventTypeName(int e) { return kEventTypeNames[e]; }

}  
