// test_main.cpp - self-contained unit / regression tests (no test framework,
// no STL). Run from the project root: ./test_runner
#include <cstdio>
#include <cstring>

#include "environment.h"
#include "int_map.h"
#include "logger.h"
#include "min_heap.h"
#include "rescue_request.h"
#include "robot.h"
#include "scenario.h"
#include "simulation.h"
#include "sort_util.h"
#include "vector.h"

using namespace rr;

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    ++g_checks;                                                        \
    if (!(cond)) {                                                     \
      ++g_failures;                                                    \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
    }                                                                  \
  } while (0)

static unsigned long long g_rng = 88172645463325252ULL;
static int rnd(int n) {
  g_rng = g_rng * 6364136223846793005ULL + 1442695040888963407ULL;
  return static_cast<int>((g_rng >> 33) % static_cast<unsigned long long>(n));
}

// ---------------------------------------------------------------- data structures

static void testVector() {
  Vector<int> v;
  for (int i = 0; i < 1000; ++i) v.pushBack(i * 3);
  CHECK(v.size() == 1000 && v[999] == 2997 && v.back() == 2997);
  Vector<int> c(v);
  c[0] = -1;
  CHECK(v[0] == 0 && c[0] == -1);  // deep copy
  v.pushBack(v[5]);                // aliasing push during growth
  CHECK(v[1000] == 15);
  v.resize(10);
  CHECK(v.size() == 10);
  v.resize(12);
  CHECK(v[10] == 0 && v[11] == 0);
  v.reverse();
  CHECK(v[11] == 0 && v[0] == 0 && v[2] == 27);
  Vector<Vector<int> > nested;
  nested.resize(5);
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j <= i; ++j) nested[i].pushBack(j);
  CHECK(nested[4].size() == 5 && nested[0].size() == 1);
  Vector<Vector<int> > moved(static_cast<Vector<Vector<int> >&&>(nested));
  CHECK(moved.size() == 5 && nested.size() == 0);
}

static void testIntMap() {
  IntMap m;
  for (int i = 0; i < 20000; ++i) CHECK(m.insert(i * 7 - 5000, i));
  CHECK(m.size() == 20000);
  bool ok = true;
  for (int i = 0; i < 20000; ++i) ok = ok && m.get(i * 7 - 5000) == i;
  CHECK(ok);
  CHECK(!m.insert(0 * 7 - 5000, 99));  // duplicate rejected
  CHECK(m.get(0 * 7 - 5000) == 0);     // value unchanged
  CHECK(m.get(123456789) == -1 && !m.contains(123456789));
}

struct IntLess {
  bool operator()(int a, int b) const { return a < b; }
};

static void testHeap() {
  MinHeap<int, IntLess> h;
  Vector<int> ref;
  for (int i = 0; i < 5000; ++i) {
    int x = rnd(1000);
    h.push(x);
    ref.pushBack(x);
  }
  stableSort(ref, IntLess());
  bool ok = true;
  for (int i = 0; i < ref.size(); ++i) {
    ok = ok && h.top() == ref[i];
    h.pop();
  }
  CHECK(ok && h.empty());
}

struct Pair {
  int key, order;
  Pair() : key(0), order(0) {}
};
struct PairLess {
  bool operator()(const Pair& a, const Pair& b) const { return a.key < b.key; }
};

static void testStableSort() {
  const int sizes[] = {0, 1, 2, 3, 17, 64, 1000};
  for (unsigned si = 0; si < sizeof sizes / sizeof sizes[0]; ++si) {
    int n = sizes[si];
    Vector<Pair> v;
    for (int i = 0; i < n; ++i) {
      Pair p;
      p.key = rnd(10);
      p.order = i;
      v.pushBack(p);
    }
    stableSort(v, PairLess());
    bool ok = true;
    for (int i = 1; i < v.size(); ++i) {
      if (v[i - 1].key > v[i].key) ok = false;
      if (v[i - 1].key == v[i].key && v[i - 1].order > v[i].order) ok = false;
    }
    CHECK(ok && v.size() == n);
  }
}

// ---------------------------------------------------------------- routing

// Brute force reference: Floyd-Warshall over OPEN edges.
static void testDijkstraAgainstFloydWarshall() {
  for (int trial = 0; trial < 60; ++trial) {
    int n = 3 + rnd(35);
    Environment env;
    for (int i = 0; i < n; ++i) env.addLocation(i + 1, "L", LOC_JUNCTION);
    int m = n - 1 + rnd(2 * n);
    for (int e = 0; e < m; ++e) {
      int a = rnd(n), b = rnd(n);
      if (a == b) b = (a + 1) % n;
      env.addConnection(e + 1, a, b, 1 + rnd(9), rnd(4) == 0);
    }
    env.finalize();
    Vector<i64> fw;
    fw.assign(n * n, Environment::INF);
    for (int i = 0; i < n; ++i) fw[i * n + i] = 0;
    for (int e = 0; e < env.numConnections(); ++e) {
      const Connection& c = env.connection(e);
      if (c.blocked) continue;
      if (c.cost < fw[c.a * n + c.b]) fw[c.a * n + c.b] = fw[c.b * n + c.a] = c.cost;
    }
    for (int k = 0; k < n; ++k)
      for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
          if (fw[i * n + k] < Environment::INF && fw[k * n + j] < Environment::INF &&
              fw[i * n + k] + fw[k * n + j] < fw[i * n + j])
            fw[i * n + j] = fw[i * n + k] + fw[k * n + j];
    bool ok = true;
    Vector<i64> dist;
    Vector<int> prev, path;
    for (int s = 0; s < n; ++s) {
      env.shortestPaths(s, dist, prev);
      for (int t = 0; t < n; ++t) {
        if (dist[t] != fw[s * n + t]) ok = false;
        if (dist[t] < Environment::INF) {
          env.pathFromRoot(prev, t, path);  // root -> t
          i64 sum = 0;
          int at = s;
          for (int k = 0; k < path.size(); ++k) {
            if (env.isBlocked(path[k])) ok = false;
            sum += env.connection(path[k]).cost;
            at = env.otherEnd(path[k], at);
          }
          if (sum != dist[t] || at != t) ok = false;
        }
      }
    }
    CHECK(ok);
  }
}

static void testBlockChangesRoutes() {
  Environment env;
  for (int i = 1; i <= 3; ++i) env.addLocation(i, "L", LOC_JUNCTION);
  env.addConnection(1, 0, 1, 1, false);
  env.addConnection(2, 1, 2, 1, false);
  env.addConnection(3, 0, 2, 5, false);
  env.finalize();
  Vector<i64> d;
  Vector<int> p;
  env.shortestPaths(0, d, p);
  CHECK(d[2] == 2);
  CHECK(env.setBlocked(1, true));
  CHECK(!env.setBlocked(1, true));  // no change reported
  env.shortestPaths(0, d, p);
  CHECK(d[2] == 5);
  env.setBlocked(2, true);
  env.shortestPaths(0, d, p);
  CHECK(d[2] == Environment::INF);
}

// The early-exit search must give exactly the full-search answer for every
// target node (distance AND route) and for the nearest UP station.
static void testEarlyExitDijkstra() {
  for (int trial = 0; trial < 80; ++trial) {
    int n = 5 + rnd(40);
    Environment env;
    for (int i = 0; i < n; ++i) env.addLocation(i + 1, "L", rnd(4) == 0 ? LOC_CHARGING : LOC_JUNCTION);
    int m = n - 1 + rnd(2 * n);
    for (int e = 0; e < m; ++e) {
      int a = rnd(n), b = rnd(n);
      if (a == b) b = (a + 1) % n;
      env.addConnection(e + 1, a, b, 1 + rnd(9), rnd(4) == 0);
    }
    env.finalize();
    for (int i = 0; i < n; ++i)
      if (env.isStation(i) && rnd(3) == 0) env.setStationUp(i, false);
    bool ok = true;
    Vector<i64> fd, ed;
    Vector<int> fp, ep, fpath, epath;
    for (int s = 0; s < n; ++s) {
      env.shortestPaths(s, fd, fp);
      Vector<char> mask;
      mask.assign(n, 0);
      int count = 0;
      for (int k = 0; k < 1 + rnd(4); ++k) {
        int t = rnd(n);
        if (!mask[t]) {
          mask[t] = 1;
          ++count;
        }
      }
      bool wantStation = rnd(2) == 0;
      env.shortestPaths(s, ed, ep, &mask, count, wantStation);
      for (int t = 0; t < n; ++t) {
        if (!mask[t]) continue;
        if (ed[t] != fd[t]) ok = false;
        if (fd[t] < Environment::INF) {
          env.pathToRoot(fp, t, fpath);
          env.pathToRoot(ep, t, epath);
          if (fpath.size() != epath.size()) ok = false;
          else
            for (int k = 0; k < fpath.size(); ++k)
              if (fpath[k] != epath[k]) ok = false;
        }
      }
      if (wantStation) {
        i64 fb = Environment::INF, eb = Environment::INF;
        for (int i = 0; i < n; ++i) {
          if (!env.stationUp(i)) continue;
          if (fd[i] < fb) fb = fd[i];
          if (ed[i] < eb) eb = ed[i];
        }
        if (fb != eb) ok = false;
      }
    }
    CHECK(ok);
  }
}

// ---------------------------------------------------------------- state machines

static void testStateMachines() {
  CHECK(!Request::transitionAllowed(QS_COMPLETED, QS_COMPLETED));
  CHECK(!Request::transitionAllowed(QS_COMPLETED, QS_PENDING));
  CHECK(!Request::transitionAllowed(QS_PENDING, QS_COMPLETED));
  CHECK(!Request::transitionAllowed(QS_PENDING, QS_IN_PROGRESS));
  CHECK(Request::transitionAllowed(QS_ASSIGNED, QS_PENDING));
  CHECK(!Request::transitionAllowed(QS_FAILED, QS_ASSIGNED));
  Request q;
  q.state = QS_PENDING;
  CHECK(q.transition(QS_ASSIGNED) && q.transition(QS_IN_PROGRESS) && q.transition(QS_COMPLETED));
  CHECK(!q.transition(QS_COMPLETED) && q.state == QS_COMPLETED);
  CHECK(!Robot::transitionAllowed(RS_UNAVAILABLE, RS_TRAVELLING));
  CHECK(!Robot::transitionAllowed(RS_PERFORMING, RS_TRAVELLING));
  CHECK(Robot::transitionAllowed(RS_RECHARGING, RS_TRAVELLING));
  CHECK(canServe(CAP_GENERAL, CAT_FIRE) && canServe(CAP_FIRE, CAT_FIRE) &&
        !canServe(CAP_MEDICAL, CAT_FIRE));
}

// ---------------------------------------------------------------- loader

static bool load(const char* path, Scenario& s, LoadReport& r) {
  return loadScenario(path, s, r);
}

static void testLoader() {
  Scenario s;
  LoadReport r;
  CHECK(load("data/scenario_provided.txt", s, r));
  CHECK(s.env.numLocations() == 56 && s.env.numConnections() == 117);
  CHECK(s.env.countBlocked() == 7);
  CHECK(s.robots.size() == 12 && s.requests.size() == 60 && s.events.size() == 20);
  CHECK(s.params.maxSteps == 250 && s.params.rechargePerStep == 5);
  CHECK(r.errors.empty());
  // events are sorted by time (stable)
  bool sorted = true;
  for (int i = 1; i < s.events.size(); ++i)
    if (s.events[i - 1].time > s.events[i].time) sorted = false;
  CHECK(sorted);
  // location 7 (index 6) has all three connections blocked at the start
  int open = 0;
  for (int k = 0; k < s.env.degree(6); ++k)
    if (!s.env.isBlocked(s.env.adjacentEdge(6, k))) ++open;
  CHECK(s.env.degree(6) == 3 && open == 0);

  Scenario v;
  LoadReport vr;
  CHECK(load("tests/valid_base.txt", v, vr));

  // Each deliberately broken file must be rejected, without crashing.
  const char* bad[] = {"count_too_few", "count_too_many", "missing_end", "bad_category",
                       "dup_location", "bad_endpoint", "bad_urgency", "battery_over_max",
                       "unknown_keyword", "event_bad_ref", "nonnumeric", "missing_max_steps",
                       "self_loop", "zero_cost", "station_down_non_station", "bad_status",
                       "dup_request", "robot_bad_start", "content_after_end",
                       "too_many_fields", "truncated", "empty_file"};
  for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; ++i) {
    char path[128];
    std::snprintf(path, sizeof path, "tests/invalid/%s.txt", bad[i]);
    Scenario bs;
    LoadReport br;
    bool ok = load(path, bs, br);
    if (ok || br.errors.empty()) std::printf("  not rejected: %s\n", path);
    CHECK(!ok && !br.errors.empty());
  }
  Scenario ms;
  LoadReport mr;
  CHECK(!load("tests/invalid/does_not_exist.txt", ms, mr));
}

// ---------------------------------------------------------------- simulation

struct RunResult {
  Stats stats;
  bool invariantsOk;
  int rejected;
};

static bool runScenario(const char* path, const char* logPath, RunResult& out,
                        Scenario* keep = nullptr, Simulation** simOut = nullptr) {
  Scenario s;
  LoadReport r;
  if (!loadScenario(path, s, r)) return false;
  Logger lg;
  if (logPath) lg.open(logPath);
  Simulation sim(s, logPath ? &lg : nullptr);
  out.invariantsOk = true;
  char msg[256];
  while (sim.step()) {
    if (!sim.checkInvariants(msg, sizeof msg)) {
      out.invariantsOk = false;
      std::printf("  invariant: %s (t=%d)\n", msg, sim.time());
      break;
    }
  }
  sim.finish();
  sim.computeStats(out.stats);
  out.rejected = out.stats.rejectedActions;
  (void)keep;
  (void)simOut;
  return true;
}

static bool sameFile(const char* a, const char* b) {
  FILE* fa = std::fopen(a, "rb");
  FILE* fb = std::fopen(b, "rb");
  bool same = fa && fb;
  while (same) {
    int ca = std::fgetc(fa), cb = std::fgetc(fb);
    if (ca != cb) same = false;
    if (ca == EOF) break;
  }
  if (fa) std::fclose(fa);
  if (fb) std::fclose(fb);
  return same;
}

static void testSimulations() {
  RunResult r;
  CHECK(runScenario("data/scenario_provided.txt", "output/test_a.csv", r));
  CHECK(r.invariantsOk && r.rejected == 0);
  CHECK(r.stats.completed == 60 && r.stats.failed == 0 && r.stats.pending == 0);
  CHECK(r.stats.reassignments >= 1);  // robots went offline holding requests

  // Determinism: identical input -> byte-identical record.
  RunResult r2;
  CHECK(runScenario("data/scenario_provided.txt", "output/test_b.csv", r2));
  CHECK(sameFile("output/test_a.csv", "output/test_b.csv"));
  CHECK(r2.stats.completed == r.stats.completed &&
        r2.stats.totalDistance == r.stats.totalDistance);

  RunResult t;
  CHECK(runScenario("data/scenario_tiny.txt", nullptr, t));
  CHECK(t.invariantsOk && t.rejected == 0 && t.stats.completed == 6);

  RunResult rr;
  CHECK(runScenario("data/scenario_reroute.txt", nullptr, rr));
  CHECK(rr.invariantsOk && rr.stats.reroutes == 1 && rr.stats.completed == 3);

  RunResult c;
  CHECK(runScenario("data/scenario_conflict.txt", nullptr, c));
  CHECK(c.invariantsOk && c.stats.failed == 1 && c.stats.completed == 5);
  CHECK(c.stats.conflicts >= 1);

  RunResult b;
  CHECK(runScenario("data/scenario_battery.txt", nullptr, b));
  CHECK(b.invariantsOk && b.rejected == 0 && b.stats.completed == 2);
  CHECK(b.stats.reassignments == 1);

  RunResult lv1;
  CHECK(runScenario("data/scenario_level1_easy.txt", nullptr, lv1));
  CHECK(lv1.invariantsOk && lv1.rejected == 0 && lv1.stats.completed == 60);

  RunResult lv3;
  CHECK(runScenario("data/scenario_level3_hard.txt", nullptr, lv3));
  CHECK(lv3.invariantsOk && lv3.rejected == 0);
  CHECK(lv3.stats.completed + lv3.stats.failed + lv3.stats.pending +
            lv3.stats.assigned + lv3.stats.inProgress + lv3.stats.notReported ==
        lv3.stats.totalRequests);
  CHECK(lv3.stats.pending + lv3.stats.assigned + lv3.stats.inProgress > 0);  // overloaded
  CHECK(lv3.stats.reassignments >= 1);

  RunResult m;
  CHECK(runScenario("data/scenario_medium.txt", nullptr, m));
  CHECK(m.invariantsOk && m.rejected == 0);
  RunResult st;
  CHECK(runScenario("data/scenario_stress.txt", nullptr, st));
  CHECK(st.invariantsOk && st.rejected == 0 && st.stats.steps == 600);
  CHECK(st.stats.completed + st.stats.failed + st.stats.pending +
            st.stats.assigned + st.stats.inProgress + st.stats.notReported ==
        st.stats.totalRequests);
}

// The urgency event must change the dispatch order (request 3 before 4).
static void testUrgencyEventAffectsDecision() {
  Scenario s;
  LoadReport r;
  CHECK(loadScenario("data/scenario_conflict.txt", s, r));
  Simulation sim(s, nullptr);
  while (sim.step()) {
  }
  int start3 = -1, start4 = -1;
  for (int i = 0; i < sim.requests().size(); ++i) {
    const Request& q = sim.requests()[i];
    if (q.id == 3) start3 = q.firstStartTime;
    if (q.id == 4) start4 = q.firstStartTime;
  }
  CHECK(start3 >= 0 && start4 >= 0 && start3 < start4);
}

// With aging the old low-urgency request is served earlier than without.
static int firstStartOfRequest(const char* path, int requestId) {
  Scenario s;
  LoadReport r;
  if (!loadScenario(path, s, r)) return -2;
  Simulation sim(s, nullptr);
  while (sim.step()) {
  }
  for (int i = 0; i < sim.requests().size(); ++i)
    if (sim.requests()[i].id == requestId) return sim.requests()[i].firstStartTime;
  return -3;
}

static void testAging() {
  int on = firstStartOfRequest("data/scenario_aging.txt", 1);
  int off = firstStartOfRequest("tests/aging_off.txt", 1);
  CHECK(on > 0 && off > 0);
  CHECK(on < off);
  CHECK(on <= 45 && off >= 60);
}

// Forcing either route-search strategy must give the same outcome as auto.
static void testRouteStrategiesAgree() {
  const char* files[] = {"data/scenario_provided.txt", "data/scenario_level1_easy.txt",
                         "data/scenario_level3_hard.txt", "data/scenario_stress.txt",
                         "data/scenario_conflict.txt", "data/scenario_battery.txt"};
  for (unsigned f = 0; f < sizeof files / sizeof files[0]; ++f) {
    Stats st[3];
    for (int mode = 0; mode < 3; ++mode) {
      Scenario s;
      LoadReport r;
      CHECK(loadScenario(files[f], s, r));
      s.params.routeStrategy = mode;
      Simulation sim(s, nullptr);
      char msg[256];
      bool ok = true;
      while (sim.step())
        if (!sim.checkInvariants(msg, sizeof msg)) ok = false;
      CHECK(ok);
      sim.computeStats(st[mode]);
    }
    for (int mode = 1; mode < 3; ++mode) {
      CHECK(st[mode].completed == st[0].completed);
      CHECK(st[mode].failed == st[0].failed);
      CHECK(st[mode].assignments == st[0].assignments);
      CHECK(st[mode].totalDistance == st[0].totalDistance);
      CHECK(st[mode].reassignments == st[0].reassignments);
      CHECK(st[mode].rejectedActions == 0);
    }
  }
}

// Work counters: deterministic and non-trivial.
static void testCounters() {
  Scenario s;
  LoadReport r;
  CHECK(loadScenario("data/scenario_provided.txt", s, r));
  Simulation sim(s, nullptr);
  while (sim.step()) {
  }
  Stats st;
  sim.computeStats(st);
  CHECK(st.routeSearches > 0 && st.nodesSettled >= st.routeSearches);
  CHECK(st.edgesScanned >= st.nodesSettled);
  Scenario s2;
  LoadReport r2;
  CHECK(loadScenario("data/scenario_provided.txt", s2, r2));
  Simulation sim2(s2, nullptr);
  while (sim2.step()) {
  }
  Stats st2;
  sim2.computeStats(st2);
  CHECK(st2.routeSearches == st.routeSearches && st2.nodesSettled == st.nodesSettled);
}

int main() {
  std::printf("data structures\n");
  testVector();
  testIntMap();
  testHeap();
  testStableSort();
  std::printf("routing\n");
  testDijkstraAgainstFloydWarshall();
  testBlockChangesRoutes();
  testEarlyExitDijkstra();
  std::printf("state machines\n");
  testStateMachines();
  std::printf("loader / validator\n");
  testLoader();
  std::printf("simulation\n");
  testSimulations();
  testUrgencyEventAffectsDecision();
  testAging();
  testRouteStrategiesAgree();
  testCounters();
  std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
