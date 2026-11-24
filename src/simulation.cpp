#include "simulation.h"

#include <chrono>
#include <cstring>

#include "min_heap.h"
#include "sort_util.h"

using namespace std;

namespace rr {

namespace {

struct ReportOrderLess {
  const Vector<Request>* q;
  bool operator()(int a, int b) const {
    return (*q)[a].reportTime < (*q)[b].reportTime;
  }
};

struct RobotIdLess {
  const Vector<Robot>* r;
  bool operator()(int a, int b) const { return (*r)[a].id < (*r)[b].id; }
};

// Dispatch priority: effective urgency (high first), then waiting time
// (earliest report first), then request id. A strict total order ->
// deterministic. Effective urgency = urgency, plus (with aging_steps > 0) one
// level per aging_steps steps the request has already waited.
struct PendingLess {
  const Vector<Request>* q;
  int now;
  int aging;
  int eff(const Request& r) const {
    return aging > 0 ? r.urgency + (now - r.reportTime) / aging : r.urgency;
  }
  bool operator()(int a, int b) const {
    const Request& x = (*q)[a];
    const Request& y = (*q)[b];
    int ex = eff(x), ey = eff(y);
    if (ex != ey) return ex > ey;
    if (x.reportTime != y.reportTime) return x.reportTime < y.reportTime;
    return x.id < y.id;
  }
};

}  // namespace

Simulation::Simulation(const Scenario& s, Logger* log)
    : params_(s.params),
      env_(s.env),
      robots_(s.robots),
      requests_(s.requests),
      events_(s.events),
      log_(log),
      time_(0),
      eventCursor_(0),
      reportCursor_(0),
      terminalCount_(0),
      finishedLogged_(false),
      stationDistVersion_(-1) {
  robotRooted_ = false;
  memset(&stats_, 0, sizeof stats_);
  routeCache_.resize(robots_.size());
  for (int c = 0; c < NUM_CATEGORIES; ++c) {
    capable_[c] = false;
    for (int i = 0; i < robots_.size(); ++i) {
      if (canServe(robots_[i].capability, c)) capable_[c] = true;
    }
  }
  for (int i = 0; i < requests_.size(); ++i) reqOrder_.pushBack(i);
  ReportOrderLess ro;
  ro.q = &requests_;
  stableSort(reqOrder_, ro);
  for (int i = 0; i < robots_.size(); ++i) robotOrder_.pushBack(i);
  RobotIdLess rl;
  rl.r = &robots_;
  stableSort(robotOrder_, rl);
  if (log_) {
    log_->log(0, "SIM_START", -1, -1, -1, params_.maxSteps,
              "scenario=%s locations=%d connections=%d robots=%d requests=%d "
              "events=%d",
              params_.name, env_.numLocations(), env_.numConnections(),
              robots_.size(), requests_.size(), events_.size());
  }
}

bool Simulation::finished() const {
  if (time_ >= params_.maxSteps) return true;
  return params_.stopWhenDone != 0 && requests_.size() > 0 &&
         terminalCount_ == requests_.size();
}

const char* Simulation::terminationReason() const {
  if (time_ >= params_.maxSteps) return "max_steps reached";
  if (finished()) return "all requests resolved";
  return "running";
}

bool Simulation::step() {
  if (finished()) return false;
  chrono::steady_clock::time_point t0 = chrono::steady_clock::now();
  applyEvents();
  reportRequests();
  dispatch();
  updateRobots();
  ++time_;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  stats_.execSeconds += chrono::duration<double>(t1 - t0).count();
  return true;
}

void Simulation::runSteps(int n) {
  for (int i = 0; i < n; ++i) {
    if (!step()) break;
  }
  if (finished()) finish();
}

// ---------------------------------------------------------------------------
// guarded state changes
// ---------------------------------------------------------------------------

bool Simulation::setRobotState(int ri, int to, const char* reason) {
  Robot& r = robots_[ri];
  int from = r.state;
  if (!r.transition(to)) {
    ++stats_.rejectedActions;
    if (log_) {
      log_->log(time_, "REJECTED", r.id, -1, -1, 0,
                "illegal robot transition %s->%s (%s)", robotStateName(from),
                robotStateName(to), reason);
    }
    return false;
  }
  if (log_) {
    log_->log(time_, "ROBOT_STATE", r.id, -1, -1, r.battery, "%s->%s %s",
              robotStateName(from), robotStateName(to), reason);
  }
  return true;
}

bool Simulation::setRequestState(int qi, int to, const char* reason) {
  Request& q = requests_[qi];
  int from = q.state;
  if (!q.transition(to)) {
    ++stats_.rejectedActions;
    if (log_) {
      log_->log(time_, "REJECTED", -1, q.id, -1, 0,
                "illegal request transition %s->%s (%s)",
                requestStateName(from), requestStateName(to), reason);
    }
    return false;
  }
  if (to == QS_COMPLETED || to == QS_FAILED) ++terminalCount_;
  if (log_) {
    log_->log(time_, "REQUEST_STATE", -1, q.id, -1, q.urgency, "%s->%s %s",
              requestStateName(from), requestStateName(to), reason);
  }
  return true;
}

void Simulation::failRequest(int qi, const char* reason) {
  Request& q = requests_[qi];
  if (!setRequestState(qi, QS_FAILED, reason)) return;
  q.robot = -1;
  q.doneTime = time_;
  if (log_) log_->log(time_, "REQUEST_FAILED", -1, q.id, -1, 0, "%s", reason);
}

void Simulation::releaseRequest(int ri, const char* reason) {
  Robot& r = robots_[ri];
  int qi = r.request;
  if (qi < 0) return;
  Request& q = requests_[qi];
  r.request = -1;
  r.path.clear();
  r.pathPos = 0;
  r.workLeft = 0;
  if (!setRequestState(qi, QS_PENDING, reason)) return;
  q.robot = -1;
  ++q.reassignments;
  ++stats_.reassignments;
  if (log_) {
    log_->log(time_, "REASSIGN", r.id, q.id, -1, q.reassignments,
              "request released: %s", reason);
  }
  if (q.reassignments > params_.maxReassign) {
    failRequest(qi, "too many reassignments");
  }
}

// ---------------------------------------------------------------------------
// events and reports
// ---------------------------------------------------------------------------

void Simulation::applyEvents() {
  while (eventCursor_ < events_.size() &&
         events_[eventCursor_].time <= time_) {
    applyEvent(events_[eventCursor_]);
    ++eventCursor_;
  }
}

void Simulation::applyEvent(const ScenarioEvent& e) {
  switch (e.type) {
    case EV_BLOCK:
    case EV_UNBLOCK: {
      const Connection& c = env_.connection(e.target);
      bool block = e.type == EV_BLOCK;
      bool changed = env_.setBlocked(e.target, block);
      if (changed) ++stats_.envChanges;
      int affected = 0;
      if (changed && block) {
        // Robots whose remaining route uses this connection will notice when
        // they reach the node in front of it (see travelStep / returnStep).
        for (int i = 0; i < robots_.size(); ++i) {
          const Robot& r = robots_[i];
          if (r.state != RS_TRAVELLING && r.state != RS_RETURNING) continue;
          for (int k = r.pathPos; k < r.path.size(); ++k) {
            if (r.path[k] == e.target) {
              ++affected;
              break;
            }
          }
        }
      }
      if (log_) {
        log_->log(time_, block ? "ENV_BLOCK" : "ENV_UNBLOCK", -1, -1, c.id,
                  affected, "%d-%d %s routes_affected=%d",
                  env_.location(c.a).id, env_.location(c.b).id,
                  changed ? "changed" : "no-op", affected);
      }
      break;
    }
    case EV_STATION_DOWN:
    case EV_STATION_UP: {
      bool up = e.type == EV_STATION_UP;
      bool changed = env_.setStationUp(e.target, up);
      if (changed) ++stats_.envChanges;
      if (log_) {
        log_->log(time_, up ? "STATION_UP" : "STATION_DOWN", -1, -1,
                  env_.location(e.target).id, up ? 1 : 0, "%s",
                  changed ? "changed" : "no-op");
      }
      break;
    }
    case EV_SET_URGENCY: {
      Request& q = requests_[e.target];
      int old = q.urgency;
      if (q.terminal()) {
        if (log_) {
          log_->log(time_, "URGENCY_CHANGE", -1, q.id, -1, e.value,
                    "ignored: request already %s", requestStateName(q.state));
        }
      } else {
        q.urgency = e.value;
        ++stats_.envChanges;
        if (log_) {
          log_->log(time_, "URGENCY_CHANGE", -1, q.id, -1, e.value,
                    "urgency %d->%d", old, e.value);
        }
      }
      break;
    }
    case EV_ROBOT_OFFLINE: {
      Robot& r = robots_[e.target];
      if (r.state == RS_UNAVAILABLE) {
        if (log_) log_->log(time_, "ROBOT_OFFLINE", r.id, -1, -1, 0, "no-op");
        break;
      }
      ++stats_.envChanges;
      if (r.request >= 0) releaseRequest(e.target, "robot went offline");
      r.path.clear();
      r.pathPos = 0;
      r.workLeft = 0;
      setRobotState(e.target, RS_UNAVAILABLE, "offline");
      if (log_) log_->log(time_, "ROBOT_OFFLINE", r.id, -1, -1, 1, "changed");
      break;
    }
    case EV_ROBOT_ONLINE: {
      Robot& r = robots_[e.target];
      if (r.state != RS_UNAVAILABLE) {
        if (log_) log_->log(time_, "ROBOT_ONLINE", r.id, -1, -1, 0, "no-op");
        break;
      }
      ++stats_.envChanges;
      setRobotState(e.target, RS_AVAILABLE, "online");
      if (log_) log_->log(time_, "ROBOT_ONLINE", r.id, -1, -1, 1, "changed");
      break;
    }
    default:
      break;
  }
}

void Simulation::reportRequests() {
  while (reportCursor_ < reqOrder_.size() &&
         requests_[reqOrder_[reportCursor_]].reportTime <= time_) {
    int qi = reqOrder_[reportCursor_++];
    Request& q = requests_[qi];
    if (!setRequestState(qi, QS_PENDING, "reported")) continue;
    if (log_) {
      log_->log(time_, "REQUEST_REPORTED", -1, q.id, env_.location(q.location).id,
                q.urgency, "%s duration=%d", categoryName(q.category),
                q.duration);
    }
    if (!capable_[q.category]) {
      failRequest(qi, "no robot in the fleet can serve this category");
    }
  }
}

// ---------------------------------------------------------------------------
// dispatch
// ---------------------------------------------------------------------------

const Simulation::RouteCache& Simulation::routeFrom(int ri) {
  RouteCache& c = routeCache_[ri];
  const Robot& r = robots_[ri];
  if (c.loc != r.location || c.version != env_.version()) {
    env_.shortestPaths(r.location, c.dist, c.prev);
    c.loc = r.location;
    c.version = env_.version();
  }
  return c;
}

i64 Simulation::reserveAt(int loc) {
  if (stationDistVersion_ != env_.version()) {
    env_.distanceToNearestStation(stationDist_);
    stationDistVersion_ = env_.version();
  }
  i64 d = stationDist_[loc];
  return d >= Environment::INF ? 0 : d;
}

bool Simulation::batteryFeasible(const Robot& r, i64 routeCost, i64 reserve,
                                 int duration) const {
  i64 need = (routeCost + reserve) * params_.batteryPerCostUnit +
             static_cast<i64>(duration) * params_.batteryPerWorkStep;
  return r.battery >= need;
}

void Simulation::dispatch() {
  eligible_.clear();
  for (int k = 0; k < robotOrder_.size(); ++k) {
    int ri = robotOrder_[k];
    robots_[ri].batteryBlocked = false;
    if (robots_[ri].eligibleForDispatch()) eligible_.pushBack(ri);
  }
  if (eligible_.empty()) return;

  PendingLess less;
  less.q = &requests_;
  less.now = time_;
  less.aging = params_.agingSteps;
  MinHeap<int, PendingLess> heap(less);
  for (int i = 0; i < requests_.size(); ++i) {
    if (requests_[i].state == QS_PENDING) heap.push(i);
  }

  // Route-search strategy for this pass (a deterministic function of the
  // state, chosen only for speed; both strategies find minimum-cost routes):
  //  * more requests waiting than robots idle -> one search per IDLE ROBOT,
  //    cached while the robot stays put and the roads do not change, so the
  //    work is bounded by the number of robots, not of requests (overload);
  //  * otherwise -> one early-exit search per REQUEST rooted at the request
  //    (cheap when few requests meet many idle robots).
  robotRooted_ = params_.routeStrategy == 2 ||
                 (params_.routeStrategy == 0 && heap.size() > eligible_.size());

  while (!heap.empty()) {
    int qi = heap.top();
    heap.pop();
    const Request& q = requests_[qi];

    // Skip the (relatively expensive) route search if no idle robot can
    // possibly take this request.
    bool anyFree = false;
    for (int k = 0; k < eligible_.size(); ++k) {
      const Robot& r = robots_[eligible_[k]];
      if (r.eligibleForDispatch() && canServe(r.capability, q.category)) {
        anyFree = true;
        break;
      }
    }
    if (!anyFree) continue;

    // The battery reserve (job location -> nearest UP charger) comes from one
    // cached multi-source search shared by both strategies.
    i64 reserve = reserveAt(q.location);
    if (!robotRooted_) {
      // One Dijkstra rooted at the request (roads are undirected, so dist_[v]
      // is also the cost from robot location v to the request); it stops once
      // every idle capable robot has been settled.
      if (mask_.size() != env_.numLocations()) mask_.assign(env_.numLocations(), 0);
      marked_.clear();
      for (int k = 0; k < eligible_.size(); ++k) {
        const Robot& r = robots_[eligible_[k]];
        if (r.eligibleForDispatch() && canServe(r.capability, q.category) &&
            !mask_[r.location]) {
          mask_[r.location] = 1;
          marked_.pushBack(r.location);
        }
      }
      env_.shortestPaths(q.location, dist_, prev_, &mask_, marked_.size(), false);
      for (int k = 0; k < marked_.size(); ++k) mask_[marked_[k]] = 0;
    }

    int best = -1;
    i64 bestCost = 0;
    int cands = 0;
    for (int k = 0; k < eligible_.size(); ++k) {
      int ri = eligible_[k];
      Robot& r = robots_[ri];
      if (!r.eligibleForDispatch() || !canServe(r.capability, q.category)) {
        continue;
      }
      i64 d = robotRooted_ ? routeFrom(ri).dist[q.location] : dist_[r.location];
      if (d >= Environment::INF) continue;  // route blocked / unreachable
      if (!batteryFeasible(r, d, reserve, q.duration)) {
        r.batteryBlocked = true;
        continue;
      }
      ++cands;
      // Preference: cheapest route, then specialist over GENERAL, then id.
      bool better = false;
      if (best < 0) {
        better = true;
      } else if (d != bestCost) {
        better = d < bestCost;
      } else {
        bool g = r.capability == CAP_GENERAL;
        bool bg = robots_[best].capability == CAP_GENERAL;
        if (g != bg) {
          better = !g;
        } else {
          better = r.id < robots_[best].id;
        }
      }
      if (better) {
        best = ri;
        bestCost = d;
      }
    }
    if (best < 0) continue;

    bool contested = false;
    for (int j = 0; j < requests_.size() && !contested; ++j) {
      if (j != qi && requests_[j].state == QS_PENDING &&
          canServe(robots_[best].capability, requests_[j].category)) {
        contested = true;
      }
    }
    if (cands >= 2) {
      ++stats_.conflicts;
      if (log_) {
        log_->log(time_, "CONFLICT", robots_[best].id, q.id, -1, cands,
                  "%d candidate robots; chose cheapest route", cands);
      }
    }
    assign(best, qi, bestCost, contested);
  }
}

void Simulation::assign(int ri, int qi, i64 cost, bool contested) {
  Robot& r = robots_[ri];
  Request& q = requests_[qi];
  if (!setRequestState(qi, QS_ASSIGNED, "assigned")) return;
  q.robot = ri;
  if (q.firstAssignTime < 0) q.firstAssignTime = time_;
  r.request = qi;
  if (robotRooted_) {
    env_.pathFromRoot(routeFrom(ri).prev, q.location, r.path);
  } else {
    env_.pathToRoot(prev_, r.location, r.path);
  }
  r.pathPos = 0;
  ++stats_.assignments;
  if (contested) ++stats_.contestedRobots;
  if (log_) {
    log_->log(time_, "ASSIGN", r.id, q.id, env_.location(q.location).id, cost,
              "route_edges=%d battery=%d", r.path.size(), r.battery);
  }
  if (r.path.empty()) {
    setRobotState(ri, RS_PERFORMING, "assigned (already on site)");
    beginWork(ri, time_);
  } else {
    setRobotState(ri, RS_TRAVELLING, "assigned");
  }
}

// ---------------------------------------------------------------------------
// robot behaviour
// ---------------------------------------------------------------------------

void Simulation::updateRobots() {
  for (int k = 0; k < robotOrder_.size(); ++k) {
    int ri = robotOrder_[k];
    Robot& r = robots_[ri];
    switch (r.state) {
      case RS_UNAVAILABLE:
        break;
      case RS_AVAILABLE:
        if (r.edgeRemaining > 0) {
          settleEdge(ri);
        } else {
          idleBehaviour(ri);
        }
        break;
      case RS_RECHARGING:
        rechargeStep(ri);
        break;
      case RS_TRAVELLING:
        travelStep(ri);
        break;
      case RS_PERFORMING:
        workStep(ri);
        break;
      case RS_RETURNING:
        returnStep(ri);
        break;
      default:
        break;
    }
  }
}

void Simulation::beginEdge(int ri, int edge) {
  Robot& r = robots_[ri];
  const Connection& c = env_.connection(edge);
  r.nextNode = env_.otherEnd(edge, r.location);
  r.edgeRemaining = c.cost;
  if (log_) {
    int qid = r.request >= 0 ? requests_[r.request].id : -1;
    log_->log(time_, "MOVE", r.id, qid, c.id, r.battery, "%d->%d cost=%d",
              env_.location(r.location).id, env_.location(r.nextNode).id,
              c.cost);
  }
}

int Simulation::moveUnit(int ri) {
  Robot& r = robots_[ri];
  int mv = params_.speed < r.edgeRemaining ? params_.speed : r.edgeRemaining;
  i64 need = static_cast<i64>(mv) * params_.batteryPerCostUnit;
  if (r.battery < need) {
    ++stats_.rejectedActions;
    if (log_) {
      log_->log(time_, "REJECTED", r.id, -1, -1, r.battery,
                "not enough battery to move");
    }
    return -1;
  }
  r.battery -= static_cast<int>(need);
  r.batteryUsed += need;
  stats_.batteryTravel += need;
  r.distance += mv;
  stats_.totalDistance += mv;
  r.edgeRemaining -= mv;
  if (r.edgeRemaining == 0) {
    r.location = r.nextNode;
    r.nextNode = -1;
    return 1;
  }
  return 0;
}

void Simulation::settleEdge(int ri) { moveUnit(ri); }

bool Simulation::rerouteTravelling(int ri) {
  Robot& r = robots_[ri];
  Request& q = requests_[r.request];
  int blockedEdge = r.path[r.pathPos];
  if (mask_.size() != env_.numLocations()) mask_.assign(env_.numLocations(), 0);
  mask_[q.location] = 1;  // search from the robot, stop once the job is reached
  env_.shortestPaths(r.location, dist_, prev_, &mask_, 1, false);
  mask_[q.location] = 0;
  i64 d = dist_[q.location];
  if (d >= Environment::INF) {
    releaseRequest(ri, "route blocked and no alternative route");
    setRobotState(ri, RS_AVAILABLE, "route lost");
    return false;
  }
  if (!batteryFeasible(r, d, reserveAt(q.location), q.duration)) {
    releaseRequest(ri, "insufficient battery for alternative route");
    setRobotState(ri, RS_AVAILABLE, "route lost");
    return false;
  }
  env_.pathFromRoot(prev_, q.location, r.path);
  r.pathPos = 0;
  ++stats_.reroutes;
  if (log_) {
    log_->log(time_, "REROUTE", r.id, q.id, env_.connection(blockedEdge).id, d,
              "connection blocked; new route edges=%d", r.path.size());
  }
  return true;
}

void Simulation::travelStep(int ri) {
  Robot& r = robots_[ri];
  ++r.busySteps;
  if (r.edgeRemaining == 0) {
    if (r.pathPos >= r.path.size()) {
      arriveAtRequest(ri);
      return;
    }
    if (env_.isBlocked(r.path[r.pathPos])) {
      if (!rerouteTravelling(ri)) return;
      if (r.path.empty()) {
        arriveAtRequest(ri);
        return;
      }
    }
    beginEdge(ri, r.path[r.pathPos]);
  }
  int res = moveUnit(ri);
  if (res == 1) {
    ++r.pathPos;
    if (r.pathPos >= r.path.size()) arriveAtRequest(ri);
  } else if (res < 0 && r.edgeRemaining == 0) {
    releaseRequest(ri, "insufficient battery");
    setRobotState(ri, RS_AVAILABLE, "stuck");
  }
}

void Simulation::arriveAtRequest(int ri) {
  Robot& r = robots_[ri];
  Request& q = requests_[r.request];
  if (log_) {
    log_->log(time_, "ARRIVE", r.id, q.id, env_.location(q.location).id,
              r.battery, "at request location");
  }
  r.path.clear();
  r.pathPos = 0;
  setRobotState(ri, RS_PERFORMING, "arrived");
  beginWork(ri, time_ + 1);
}

void Simulation::beginWork(int ri, int startStep) {
  Robot& r = robots_[ri];
  Request& q = requests_[r.request];
  if (!setRequestState(r.request, QS_IN_PROGRESS, "work started")) return;
  q.startTime = startStep;
  if (!q.everStarted) {
    q.everStarted = true;
    q.firstStartTime = startStep;
  }
  r.workLeft = q.duration;
  if (log_) {
    log_->log(time_, "WORK_START", r.id, q.id, -1, startStep, "duration=%d",
              q.duration);
  }
}

void Simulation::workStep(int ri) {
  Robot& r = robots_[ri];
  ++r.busySteps;
  int qi = r.request;
  if (r.battery < params_.batteryPerWorkStep) {
    ++stats_.rejectedActions;
    if (log_) {
      log_->log(time_, "REJECTED", r.id, requests_[qi].id, -1, r.battery,
                "not enough battery to work");
    }
    releaseRequest(ri, "insufficient battery to continue work");
    setRobotState(ri, RS_AVAILABLE, "battery");
    return;
  }
  r.battery -= params_.batteryPerWorkStep;
  r.batteryUsed += params_.batteryPerWorkStep;
  stats_.batteryWork += params_.batteryPerWorkStep;
  --r.workLeft;
  if (r.workLeft > 0) return;

  Request& q = requests_[qi];
  if (!setRequestState(qi, QS_COMPLETED, "work finished")) return;
  q.doneTime = time_;
  r.request = -1;
  r.path.clear();
  r.pathPos = 0;
  ++r.tasksDone;
  if (log_) {
    log_->log(time_, "WORK_DONE", r.id, q.id, -1, time_ - q.reportTime,
              "completed; turnaround=%d battery=%d", time_ - q.reportTime,
              r.battery);
  }
  setRobotState(ri, RS_AVAILABLE, "task complete");
}

void Simulation::rechargeStep(int ri) {
  Robot& r = robots_[ri];
  if (!env_.stationUp(r.location)) {
    setRobotState(ri, RS_AVAILABLE, "charging station down");
    return;
  }
  int room = r.maxBattery - r.battery;
  int add = params_.rechargePerStep < room ? params_.rechargePerStep : room;
  r.battery += add;
  r.batteryRecharged += add;
  stats_.batteryRecharged += add;
  if (r.battery >= r.maxBattery) {
    setRobotState(ri, RS_AVAILABLE, "fully charged");
  }
}

// Chooses the nearest UP charging station reachable over open roads and sets
// the robot's path to it. Fails if none is reachable or the battery would not
// last. On success `cost` holds the route cost (0 = already at a station).
bool Simulation::planReturn(int ri, i64& cost) {
  Robot& r = robots_[ri];
  env_.shortestPaths(r.location, dist_, prev_, nullptr, 0, true);
  int best = -1;
  i64 bd = Environment::INF;
  const Vector<int>& st = env_.stations();
  for (int i = 0; i < st.size(); ++i) {
    if (!env_.stationUp(st[i])) continue;
    if (dist_[st[i]] < bd) {  // strict: lowest location index wins ties
      bd = dist_[st[i]];
      best = st[i];
    }
  }
  if (best < 0 || bd >= Environment::INF) return false;
  if (bd * params_.batteryPerCostUnit > r.battery) return false;
  env_.pathFromRoot(prev_, best, r.path);
  r.pathPos = 0;
  r.returnTarget = best;
  cost = bd;
  return true;
}

void Simulation::idleBehaviour(int ri) {
  Robot& r = robots_[ri];
  if (env_.stationUp(r.location)) {
    if (r.battery < r.maxBattery) setRobotState(ri, RS_RECHARGING, "idle at station");
    return;
  }
  if (r.battery >= r.maxBattery) return;
  bool low = static_cast<i64>(r.battery) * 100 <
             static_cast<i64>(r.maxBattery) * params_.lowBatteryPct;
  if (!low && !r.batteryBlocked) return;
  i64 cost = 0;
  if (!planReturn(ri, cost)) return;
  if (setRobotState(ri, RS_RETURNING,
                    low ? "low battery" : "battery too low for pending request") &&
      log_) {
    log_->log(time_, "RETURN_TO_CHARGER", r.id, -1,
              env_.location(r.returnTarget).id, cost, "route_edges=%d",
              r.path.size());
  }
}

void Simulation::returnStep(int ri) {
  Robot& r = robots_[ri];
  if (r.edgeRemaining == 0) {
    if (env_.stationUp(r.location)) {
      r.path.clear();
      r.pathPos = 0;
      setRobotState(ri, RS_RECHARGING, "reached charging station");
      return;
    }
    bool replan = r.pathPos >= r.path.size() ||
                  !env_.stationUp(r.returnTarget) ||
                  env_.isBlocked(r.path[r.pathPos]);
    if (replan) {
      i64 cost = 0;
      if (!planReturn(ri, cost) || r.path.empty()) {
        r.path.clear();
        r.pathPos = 0;
        setRobotState(ri, RS_AVAILABLE, "no reachable charging station");
        return;
      }
      if (log_) {
        log_->log(time_, "REROUTE", r.id, -1, env_.location(r.returnTarget).id,
                  cost, "return route replanned edges=%d", r.path.size());
      }
      ++stats_.reroutes;
    }
    beginEdge(ri, r.path[r.pathPos]);
  }
  int res = moveUnit(ri);
  if (res == 1) {
    ++r.pathPos;
  } else if (res < 0 && r.edgeRemaining == 0) {
    r.path.clear();
    r.pathPos = 0;
    setRobotState(ri, RS_AVAILABLE, "stuck");
  }
}

// ---------------------------------------------------------------------------
// statistics, reports, invariants
// ---------------------------------------------------------------------------

void Simulation::computeStats(Stats& out) const {
  out = stats_;
  out.routeSearches = env_.searchesRun();
  out.nodesSettled = env_.nodesSettled();
  out.edgesScanned = env_.edgesScanned();
  out.steps = time_;
  out.totalRequests = requests_.size();
  out.completed = out.failed = out.pending = out.assigned = 0;
  out.inProgress = out.notReported = 0;
  out.responseCount = 0;
  out.sumResponse = 0;
  out.maxResponse = 0;
  for (int i = 0; i < requests_.size(); ++i) {
    const Request& q = requests_[i];
    switch (q.state) {
      case QS_COMPLETED: ++out.completed; break;
      case QS_FAILED: ++out.failed; break;
      case QS_PENDING: ++out.pending; break;
      case QS_ASSIGNED: ++out.assigned; break;
      case QS_IN_PROGRESS: ++out.inProgress; break;
      default: ++out.notReported; break;
    }
    if (q.everStarted) {
      int resp = q.firstStartTime - q.reportTime;
      ++out.responseCount;
      out.sumResponse += resp;
      if (resp > out.maxResponse) out.maxResponse = resp;
    }
  }
  i64 busy = 0;
  for (int i = 0; i < robots_.size(); ++i) busy += robots_[i].busySteps;
  i64 denom = static_cast<i64>(robots_.size()) * time_;
  out.utilization = denom > 0 ? static_cast<double>(busy) / denom : 0.0;
}

void Simulation::finish() {
  if (finishedLogged_) return;
  finishedLogged_ = true;
  if (!log_) return;
  Stats s;
  computeStats(s);
  log_->log(time_, "SIM_END", -1, -1, -1, time_, "%s", terminationReason());
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.completed, "completed");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.failed, "failed");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.pending, "pending");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.assigned, "assigned");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.inProgress, "in_progress");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.notReported, "not_reported");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.assignments, "assignments");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.reassignments, "reassignments");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.reroutes, "reroutes");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.conflicts, "conflicts");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.rejectedActions, "rejected_actions");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.totalDistance, "total_distance");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.batteryTravel, "battery_travel");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.batteryWork, "battery_work");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.batteryRecharged, "battery_recharged");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.maxResponse, "max_response_time");
  log_->log(time_, "FINAL_STAT", -1, -1, -1,
            s.responseCount ? s.sumResponse * 1000 / s.responseCount : 0,
            "avg_response_time_x1000");
  log_->log(time_, "FINAL_STAT", -1, -1, -1, s.routeSearches, "route_searches");
  for (int k = 0; k < robotOrder_.size(); ++k) {
    const Robot& r = robots_[robotOrder_[k]];
    log_->log(time_, "FINAL_ROBOT", r.id, -1, -1, r.tasksDone,
              "state=%s battery=%d/%d busy_steps=%d distance=%lld "
              "battery_used=%lld battery_recharged=%lld",
              robotStateName(r.state), r.battery, r.maxBattery, r.busySteps,
              r.distance, r.batteryUsed, r.batteryRecharged);
  }
}

void Simulation::printRobots(FILE* f) const {
  fprintf(f, "Robots at t=%d:\n", time_);
  fprintf(f, "  %-4s %-8s %-11s %-9s %-8s %-8s %s\n", "id", "cap", "state",
               "battery", "at", "request", "notes");
  for (int k = 0; k < robotOrder_.size(); ++k) {
    const Robot& r = robots_[robotOrder_[k]];
    char bat[24];
    snprintf(bat, sizeof bat, "%d/%d", r.battery, r.maxBattery);
    char req[16];
    if (r.request >= 0) {
      snprintf(req, sizeof req, "#%d", requests_[r.request].id);
    } else {
      snprintf(req, sizeof req, "-");
    }
    fprintf(f, "  %-4d %-8s %-11s %-9s %-8d %-8s", r.id,
                 capabilityName(r.capability), robotStateName(r.state), bat,
                 env_.location(r.location).id, req);
    if (r.edgeRemaining > 0) {
      fprintf(f, " on road to %d (%d left)", env_.location(r.nextNode).id,
                   r.edgeRemaining);
    }
    fprintf(f, "\n");
  }
}

void Simulation::printRequests(FILE* f) const {
  Stats s;
  computeStats(s);
  fprintf(f,
               "Requests at t=%d: total %d | not reported %d | pending %d | "
               "assigned %d | in progress %d | completed %d | failed %d\n",
               time_, s.totalRequests, s.notReported, s.pending, s.assigned,
               s.inProgress, s.completed, s.failed);
  const int groups[] = {QS_PENDING, QS_ASSIGNED, QS_IN_PROGRESS, QS_COMPLETED,
                        QS_FAILED};
  for (int g = 0; g < 5; ++g) {
    int count = 0;
    for (int i = 0; i < requests_.size(); ++i) {
      if (requests_[i].state == groups[g]) ++count;
    }
    if (count == 0) continue;
    fprintf(f, "  %s (%d):\n", requestStateName(groups[g]), count);
    for (int i = 0; i < requests_.size(); ++i) {
      const Request& q = requests_[i];
      if (q.state != groups[g]) continue;
      fprintf(f, "    #%-4d %-8s urgency %d at %-4d reported %-4d", q.id,
                   categoryName(q.category), q.urgency,
                   env_.location(q.location).id, q.reportTime);
      if (q.robot >= 0) fprintf(f, " robot %d", robots_[q.robot].id);
      if (q.state == QS_COMPLETED) fprintf(f, " done at %d", q.doneTime);
      if (q.reassignments > 0) fprintf(f, " (reassigned %dx)", q.reassignments);
      fprintf(f, "\n");
    }
  }
}

void Simulation::printStats(FILE* f) const {
  Stats s;
  computeStats(s);
  fprintf(f, "=== Final statistics: %s ===\n", params_.name);
  fprintf(f, "Termination            : %s (simulated %d steps)\n",
               terminationReason(), s.steps);
  fprintf(f, "Requests               : %d total\n", s.totalRequests);
  fprintf(f, "  completed            : %d\n", s.completed);
  fprintf(f, "  failed               : %d\n", s.failed);
  fprintf(f, "  remaining            : %d (pending %d, assigned %d, "
                  "in progress %d, not yet reported %d)\n",
               s.pending + s.assigned + s.inProgress + s.notReported, s.pending,
               s.assigned, s.inProgress, s.notReported);
  if (s.responseCount > 0) {
    fprintf(f, "Response time (report -> work start): avg %.2f, max %d "
                    "(over %d requests)\n",
                 static_cast<double>(s.sumResponse) / s.responseCount,
                 s.maxResponse, s.responseCount);
  } else {
    fprintf(f, "Response time          : n/a (no request started)\n");
  }
  {
    int cnt[6] = {0, 0, 0, 0, 0, 0}, unresolved[6] = {0, 0, 0, 0, 0, 0};
    i64 sum[6] = {0, 0, 0, 0, 0, 0};
    int mx[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < requests_.size(); ++i) {
      const Request& q = requests_[i];
      int u = q.urgency;
      if (q.everStarted) {
        int resp = q.firstStartTime - q.reportTime;
        ++cnt[u];
        sum[u] += resp;
        if (resp > mx[u]) mx[u] = resp;
      }
      if (q.state != QS_COMPLETED && q.state != QS_FAILED) ++unresolved[u];
    }
    fprintf(f, "Response by urgency    :");
    for (int u = 5; u >= 1; --u) {
      if (cnt[u] > 0) {
        fprintf(f, " U%d avg %.1f max %d (n=%d);", u,
                     static_cast<double>(sum[u]) / cnt[u], mx[u], cnt[u]);
      } else {
        fprintf(f, " U%d n/a;", u);
      }
    }
    fprintf(f, "\n");
    fprintf(f, "Unresolved by urgency  : U5 %d, U4 %d, U3 %d, U2 %d, U1 %d\n",
                 unresolved[5], unresolved[4], unresolved[3], unresolved[2],
                 unresolved[1]);
  }
  fprintf(f, "Robot utilization      : %.1f%% (steps travelling or working)\n",
               s.utilization * 100.0);
  fprintf(f, "Distance travelled     : %lld cost units\n", s.totalDistance);
  fprintf(f, "Battery                : %lld travel + %lld work used, "
                  "%lld recharged\n",
               s.batteryTravel, s.batteryWork, s.batteryRecharged);
  fprintf(f, "Assignments            : %d (reassignments %d, reroutes %d)\n",
               s.assignments, s.reassignments, s.reroutes);
  fprintf(f, "Conflicts              : %d requests with 2+ candidate robots, "
                  "%d assignments of a contested robot\n",
               s.conflicts, s.contestedRobots);
  fprintf(f, "Environment changes    : %d applied\n", s.envChanges);
  fprintf(f, "Rejected actions       : %d\n", s.rejectedActions);
  fprintf(f, "Route searches         : %lld (%lld nodes settled, %lld edges scanned)\n",
               s.routeSearches, s.nodesSettled, s.edgesScanned);
  fprintf(f, "Execution time         : %.6f s\n", s.execSeconds);
  fprintf(f, "Per-robot summary:\n");
  fprintf(f, "  %-4s %-8s %-6s %-7s %-9s %-10s %-9s %s\n", "id", "cap",
               "tasks", "busy%", "distance", "batt-used", "recharged",
               "final-battery");
  for (int k = 0; k < robotOrder_.size(); ++k) {
    const Robot& r = robots_[robotOrder_[k]];
    double busy = time_ > 0 ? 100.0 * r.busySteps / time_ : 0.0;
    fprintf(f, "  %-4d %-8s %-6d %-7.1f %-9lld %-10lld %-9lld %d/%d\n",
                 r.id, capabilityName(r.capability), r.tasksDone, busy,
                 r.distance, r.batteryUsed, r.batteryRecharged, r.battery,
                 r.maxBattery);
  }
}

bool Simulation::checkInvariants(char* msg, int cap) const {
#define RR_FAIL(...)                    \
  do {                                  \
    snprintf(msg, cap, __VA_ARGS__); \
    return false;                       \
  } while (0)
  int terminal = 0;
  for (int i = 0; i < robots_.size(); ++i) {
    const Robot& r = robots_[i];
    if (r.battery < 0 || r.battery > r.maxBattery) {
      RR_FAIL("robot %d battery %d outside [0,%d]", r.id, r.battery, r.maxBattery);
    }
    if (r.edgeRemaining < 0) RR_FAIL("robot %d negative edgeRemaining", r.id);
    if (r.state == RS_TRAVELLING || r.state == RS_PERFORMING) {
      if (r.request < 0) RR_FAIL("robot %d is %s without a request", r.id, robotStateName(r.state));
      const Request& q = requests_[r.request];
      if (q.robot != i) RR_FAIL("robot %d / request %d link is not mutual", r.id, q.id);
      int want = r.state == RS_TRAVELLING ? QS_ASSIGNED : QS_IN_PROGRESS;
      if (q.state != want) {
        RR_FAIL("robot %d is %s but request %d is %s", r.id, robotStateName(r.state),
                q.id, requestStateName(q.state));
      }
    } else if (r.request >= 0) {
      RR_FAIL("robot %d is %s but holds request %d", r.id, robotStateName(r.state),
              requests_[r.request].id);
    }
    if (r.state == RS_RECHARGING && !env_.isStation(r.location)) {
      RR_FAIL("robot %d recharging away from a station", r.id);
    }
  }
  for (int i = 0; i < requests_.size(); ++i) {
    const Request& q = requests_[i];
    if (q.terminal()) ++terminal;
    if (q.state == QS_ASSIGNED || q.state == QS_IN_PROGRESS) {
      if (q.robot < 0 || robots_[q.robot].request != i) {
        RR_FAIL("request %d is %s but no robot holds it", q.id, requestStateName(q.state));
      }
    } else if ((q.state == QS_PENDING || q.state == QS_NOT_REPORTED) && q.robot != -1) {
      RR_FAIL("request %d is %s but has robot set", q.id, requestStateName(q.state));
    }
  }
  if (terminal != terminalCount_) {
    RR_FAIL("terminal counter %d differs from actual %d", terminalCount_, terminal);
  }
#undef RR_FAIL
  msg[0] = '\0';
  return true;
}

}  // namespace rr
