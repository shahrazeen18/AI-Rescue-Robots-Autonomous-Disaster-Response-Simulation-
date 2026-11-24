// scenario.h - scenario file loader and validator.
//
// loadScenario() reads the text format described in docs/format (see the
// README) into a Scenario. It never crashes on malformed input: every problem
// is recorded (with its line number) in a LoadReport, parsing continues so
// that several problems can be reported at once, and the scenario is only
// usable when report.ok() is true.
#ifndef RR_SCENARIO_H
#define RR_SCENARIO_H

#include <cstdio>

#include "environment.h"
#include "rescue_request.h"
#include "robot.h"
#include "types.h"
#include "vector.h"

namespace rr {

struct Params {
  char name[64];
  int maxSteps;
  int speed;               // cost units a robot moves per time step
  int batteryPerCostUnit;  // battery used per cost unit travelled
  int batteryPerWorkStep;  // battery used per step of rescue work
  int rechargePerStep;     // battery gained per step at an UP station
  // Optional extensions (absent in scenario_provided.txt -> defaults):
  int stopWhenDone;        // 1: stop early once every request is resolved
  int maxReassign;         // request FAILS after more releases than this
  int lowBatteryPct;       // idle robots below this % go to recharge
  int agingSteps;          // 0 = off; else +1 effective urgency per this many
                           // steps a request has waited (prevents starvation)
  int routeStrategy;       // 0 auto (default), 1 per-request, 2 per-robot search
  Params()
      : maxSteps(0), speed(1), batteryPerCostUnit(1), batteryPerWorkStep(2),
        rechargePerStep(5), stopWhenDone(1), maxReassign(5),
        lowBatteryPct(50), agingSteps(0), routeStrategy(0) {
    name[0] = '\0';
  }
};

struct ScenarioEvent {
  int time;
  int type;    // EventType
  int target;  // internal index: connection / location / robot / request
  int value;   // new urgency for SET_URGENCY
  ScenarioEvent() : time(0), type(0), target(0), value(0) {}
};

struct Scenario {
  Params params;
  Environment env;
  Vector<Robot> robots;
  Vector<Request> requests;
  Vector<ScenarioEvent> events;  // stably sorted by time
};

struct Message {
  int line;  // 0 when not tied to a line
  char text[200];
  Message() : line(0) { text[0] = '\0'; }
};

struct LoadReport {
  Vector<Message> errors;
  Vector<Message> warnings;
  bool ok() const { return errors.empty(); }
};

// Returns true iff the file was read and is valid (report.ok()).
bool loadScenario(const char* path, Scenario& out, LoadReport& report);

void printLoadReport(FILE* f, const LoadReport& report);

}  // namespace rr

#endif
