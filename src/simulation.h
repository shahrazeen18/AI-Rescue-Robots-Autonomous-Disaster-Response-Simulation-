// simulation.h - the discrete-time rescue-coordination engine.
//
// ONE TIME STEP (t = 0, 1, ..., max_steps-1) is processed in this fixed order:
//   1. EVENTS    scenario events with time <= t, in file order
//   2. REPORTS   requests with report_time <= t become PENDING, in file order
//   3. DISPATCH  pending requests, highest priority first, are matched to idle
//                robots (priority = urgency desc, report_time asc, id asc)
//   4. ROBOTS    every robot, in ascending robot id, advances one step
//                (move / work / recharge / go-to-charger)
//   5. CHECK     termination test
// One step = one unit of simulated time; a robot moves `speed` cost units and
// does one unit of rescue work per step. See README for the full rule set.
#ifndef RR_SIMULATION_H
#define RR_SIMULATION_H

#include <cstdio>

#include "logger.h"
#include "scenario.h"
#include "types.h"
#include "vector.h"

namespace rr {

struct Stats {
  int steps;
  int totalRequests;
  int completed, failed, pending, assigned, inProgress, notReported;
  int assignments;
  int reassignments;
  int reroutes;
  int conflicts;         // requests with >= 2 feasible candidate robots
  int contestedRobots;   // assigned robot was also compatible with another pending request
  int rejectedActions;
  int envChanges;
  int responseCount;
  i64 sumResponse;
  int maxResponse;
  i64 totalDistance;
  i64 batteryTravel, batteryWork, batteryRecharged;
  double utilization;  // productive robot-steps / (robots * steps)
  double execSeconds;  // wall-clock time spent inside step()
  i64 routeSearches;   // shortest-path searches run (deterministic work count)
  i64 nodesSettled;    // nodes settled across all searches
  i64 edgesScanned;    // adjacency entries examined across all searches
};

class Simulation {
 public:
  // Copies the scenario. `log` may be null (no record is written).
  Simulation(const Scenario& scenario, Logger* log);

  bool finished() const;
  const char* terminationReason() const;
  int time() const { return time_; }  // next step to be executed

  // Executes one step. Returns false if the simulation had already finished.
  bool step();
  // Executes up to n steps (stops early when finished).
  void runSteps(int n);
  void run() { runSteps(params_.maxSteps + 1); }
  // Writes the final-statistics records to the log (idempotent).
  void finish();

  void computeStats(Stats& out) const;

  // Reports.
  void printEnvironment(FILE* f) const { env_.print(f); }
  void printRobots(FILE* f) const;
  void printRequests(FILE* f) const;
  void printStats(FILE* f) const;

  // Consistency checks over the whole state; returns false and fills msg with
  // the first violation found.
  bool checkInvariants(char* msg, int cap) const;

  const Environment& environment() const { return env_; }
  const Vector<Robot>& robots() const { return robots_; }
  const Vector<Request>& requests() const { return requests_; }
  const Params& params() const { return params_; }

 private:
  // step phases 
  void applyEvents();
  void applyEvent(const ScenarioEvent& e);
  void reportRequests();
  void dispatch();
  void updateRobots();

  //  dispatch helpers
  // Route search from an idle robot's location, cached while the robot stays
  // where it is and no road/station has changed (env_.version()).
  struct RouteCache {
    Vector<i64> dist;
    Vector<int> prev;
    int loc;
    int version;
    RouteCache() : loc(-1), version(-1) {}
  };
  const RouteCache& routeFrom(int ri);
  // Battery reserve for a job at `loc`: cost from there to the nearest UP
  // charging station (0 if none is reachable). One cached multi-source search.
  i64 reserveAt(int loc);
  bool batteryFeasible(const Robot& r, i64 routeCost, i64 reserve,
                       int duration) const;
  void assign(int ri, int qi, i64 cost, bool contested);

  // robot behaviour 
  void travelStep(int ri);
  void workStep(int ri);
  void returnStep(int ri);
  void rechargeStep(int ri);
  void idleBehaviour(int ri);
  void settleEdge(int ri);
  void beginEdge(int ri, int edge);
  int moveUnit(int ri);  // 1 = reached node, 0 = moved, -1 = no battery
  void arriveAtRequest(int ri);
  void beginWork(int ri, int startStep);
  bool rerouteTravelling(int ri);
  bool planReturn(int ri, i64& cost);
  void releaseRequest(int ri, const char* reason);
  void failRequest(int qi, const char* reason);

  //  guarded state changes (reject + record illegal transitions) 
  bool setRobotState(int ri, int to, const char* reason);
  bool setRequestState(int qi, int to, const char* reason);

  Params params_;
  Environment env_;
  Vector<Robot> robots_;
  Vector<Request> requests_;
  Vector<ScenarioEvent> events_;
  Logger* log_;

  int time_;
  int eventCursor_;
  int reportCursor_;
  Vector<int> reqOrder_;    // request indices stably sorted by report time
  Vector<int> robotOrder_;  // robot indices sorted by id
  bool capable_[NUM_CATEGORIES];
  int terminalCount_;
  bool finishedLogged_;

  // scratch space reused every step (avoids repeated allocation)
  Vector<i64> dist_;
  Vector<int> prev_;
  Vector<int> eligible_;
  Vector<char> mask_;     // target nodes for the early-exit route search
  Vector<int> marked_;    // nodes currently set in mask_ (to clear again)
  bool robotRooted_;      // dispatch strategy of the current pass (see dispatch())
  Vector<RouteCache> routeCache_;  // one per robot (filled lazily)
  Vector<i64> stationDist_;        // distance to nearest UP station
  int stationDistVersion_;

  Stats stats_;
};

}  

#endif
