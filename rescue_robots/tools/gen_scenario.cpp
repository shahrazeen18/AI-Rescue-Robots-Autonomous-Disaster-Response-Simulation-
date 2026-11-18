// gen_scenario.cpp - deterministic scenario generator (own LCG, no STL).
//
// Usage: gen_scenario <name> <locations> <connections> <robots> <requests>
//                     <max_steps> <seed> [normal|easy|hard]   > scenario.txt
// Profiles: normal (default, as used for medium/stress/scale), easy (few
// disruptions, generous batteries, short jobs), hard (requests arrive in a
// burst, many roads blocked for long periods, two locations cut off at the
// start, frequent station/robot outages, small batteries, longer jobs).
// The generated road network is always connected (random spanning tree plus
// extra random roads); about 6% of the extra roads start BLOCKED; events are
// BLOCK/UNBLOCK pairs, STATION_DOWN/UP pairs, ROBOT_OFFLINE/ONLINE pairs and
// SET_URGENCY changes spread over the run.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "sort_util.h"
#include "vector.h"

using rr::Vector;

namespace {

unsigned long long g_state = 1;

unsigned long long nextRaw() {
  g_state = g_state * 6364136223846793005ULL + 1442695040888963407ULL;
  return g_state >> 33;
}
int rnd(int n) { return static_cast<int>(nextRaw() % static_cast<unsigned long long>(n)); }
int rndRange(int lo, int hi) { return lo + rnd(hi - lo + 1); }

struct Ev {
  int time;
  char text[64];
};
struct EvLess {
  bool operator()(const Ev& a, const Ev& b) const { return a.time < b.time; }
};

const char* const kLocTypes[] = {"JUNCTION", "BUILDING", "HOSPITAL",
                                 "DEPOT",    "CHARGING", "COMMAND"};
const char* const kCaps[] = {"MEDICAL", "FIRE", "SUPPLY", "SEARCH", "GENERAL"};
const char* const kCats[] = {"MEDICAL", "FIRE", "SUPPLY", "SEARCH"};

}  // namespace

int main(int argc, char** argv) {
  if (argc != 8 && argc != 9) {
    std::fprintf(stderr,
                 "usage: %s <name> <locations> <connections> <robots> "
                 "<requests> <max_steps> <seed> [normal|easy|hard]\n",
                 argv[0]);
    return 2;
  }
  int profile = 0;  // 0 normal, 1 easy, 2 hard
  if (argc == 9) {
    if (std::strcmp(argv[8], "normal") == 0) profile = 0;
    else if (std::strcmp(argv[8], "easy") == 0) profile = 1;
    else if (std::strcmp(argv[8], "hard") == 0) profile = 2;
    else {
      std::fprintf(stderr, "unknown profile '%s'\n", argv[8]);
      return 2;
    }
  }
  const bool easy = profile == 1, hard = profile == 2;
  const int blockPct = easy ? 3 : hard ? 12 : 6;
  const int reqWindowPct = easy ? 50 : hard ? 30 : 70;
  const int durLo = easy ? 2 : hard ? 4 : 2, durHi = easy ? 5 : hard ? 12 : 8;
  const int evLenLo = hard ? 20 : 10, evLenHi = hard ? 90 : 60;
  const int batLo = easy ? 120 : hard ? 60 : 90, batHi = easy ? 150 : hard ? 100 : 150;
  const char* name = argv[1];
  int L = std::atoi(argv[2]), C = std::atoi(argv[3]), R = std::atoi(argv[4]);
  int Q = std::atoi(argv[5]), T = std::atoi(argv[6]);
  g_state = static_cast<unsigned long long>(std::atoll(argv[7])) * 2654435761ULL + 12345ULL;
  if (L < 10 || C < L - 1 || R < 1 || Q < 1 || T < 20) {
    std::fprintf(stderr, "sizes too small (need L>=10, C>=L-1, R>=1, Q>=1, T>=20)\n");
    return 2;
  }

  std::printf("# Generated scenario (tools/gen_scenario, seed %s)\n", argv[7]);
  std::printf("SCENARIO %s\n\nPARAM max_steps %d\nPARAM speed 1\n"
              "PARAM battery_per_cost_unit 1\nPARAM battery_per_work_step 2\n"
              "PARAM recharge_per_step 5\nPARAM stop_when_done 0\n", name, T);
  if (hard) std::printf("PARAM max_reassign 3\nPARAM aging_steps 40\n");
  std::printf("\n");

  // --- locations: fixed counts of special types, then shuffled ---
  Vector<int> types;
  int nCharging = L / 12 > 4 ? L / 12 : 4;
  int nHospital = L / 20 > 3 ? L / 20 : 3;
  int nDepot = L / 20 > 3 ? L / 20 : 3;
  types.pushBack(5);  // COMMAND
  for (int i = 0; i < nCharging; ++i) types.pushBack(4);
  for (int i = 0; i < nHospital; ++i) types.pushBack(2);
  for (int i = 0; i < nDepot; ++i) types.pushBack(3);
  int nBuilding = (L - types.size()) * 6 / 10;
  for (int i = 0; i < nBuilding; ++i) types.pushBack(1);
  while (types.size() < L) types.pushBack(0);
  for (int i = L - 1; i > 0; --i) {  // Fisher-Yates
    int j = rnd(i + 1);
    int t = types[i];
    types[i] = types[j];
    types[j] = t;
  }
  Vector<int> stations;
  std::printf("LOCATIONS %d\n# <id> <name> <type>\n", L);
  for (int i = 0; i < L; ++i) {
    std::printf("%d Loc_%d %s\n", i + 1, i + 1, kLocTypes[types[i]]);
    if (types[i] == 4) stations.pushBack(i + 1);
  }

  // --- connections ---
  Vector<int> ea, eb, ec;
  Vector<char> blocked;
  for (int v = 2; v <= L; ++v) {  // random spanning tree: always connected
    ea.pushBack(v);
    eb.pushBack(rnd(v - 1) + 1);
    ec.pushBack(rndRange(1, 9));
    blocked.pushBack(0);
  }
  while (ea.size() < C) {
    int a = rnd(L) + 1, b = rnd(L) + 1;
    if (a == b) continue;
    ea.pushBack(a);
    eb.pushBack(b);
    ec.pushBack(rndRange(1, 9));
    blocked.pushBack(rnd(100) < blockPct ? 1 : 0);
  }
  // hard profile: two locations are cut off at the start (all roads blocked)
  Vector<int> cutEdges;
  if (hard) {
    for (int k = 0; k < 2; ++k) {
      int v = rnd(L) + 1;
      for (int e = 0; e < ea.size(); ++e) {
        if ((ea[e] == v || eb[e] == v) && !blocked[e]) {
          blocked[e] = 1;
          cutEdges.pushBack(e + 1);
        }
      }
    }
  }
  std::printf("\nCONNECTIONS %d\n# <id> <a> <b> <cost> <OPEN|BLOCKED>\n", ea.size());
  for (int e = 0; e < ea.size(); ++e) {
    std::printf("%d %d %d %d %s\n", e + 1, ea[e], eb[e], ec[e],
                blocked[e] ? "BLOCKED" : "OPEN");
  }

  // --- robots ---
  std::printf("\nROBOTS %d\n# <id> <start> <capability> <max_battery> <initial_battery>\n", R);
  for (int r = 0; r < R; ++r) {
    int maxB = rndRange(batLo, batHi);
    int initB = hard ? maxB * (35 + rnd(26)) / 100
                     : (rnd(100) < 70 ? maxB : maxB * 4 / 5);
    std::printf("%d %d %s %d %d\n", r + 1, rndRange(1, L), kCaps[r % 5], maxB, initB);
  }

  // --- requests (non-decreasing report time) ---
  Vector<int> times;
  for (int q = 0; q < Q; ++q) times.pushBack(rnd(T * reqWindowPct / 100));
  struct IntLess {
    bool operator()(int a, int b) const { return a < b; }
  };
  rr::stableSort(times, IntLess());
  std::printf("\nREQUESTS %d\n# <id> <time> <location> <category> <urgency> <duration>\n", Q);
  Vector<int> reqTime;
  for (int q = 0; q < Q; ++q) {
    reqTime.pushBack(times[q]);
    std::printf("%d %d %d %s %d %d\n", q + 1, times[q], rndRange(1, L),
                kCats[rnd(4)], rndRange(1, 5), rndRange(durLo, durHi));
  }

  // --- events ---
  Vector<Ev> evs;
  int pairs = easy ? (T / 60 > 2 ? T / 60 : 2) : hard ? (T / 10 > 8 ? T / 10 : 8)
                                                      : (T / 25 > 4 ? T / 25 : 4);
  for (int k = 0; k < cutEdges.size(); ++k) {  // restore the cut-off locations
    Ev a;
    a.time = rndRange(T / 5, T / 2);
    std::snprintf(a.text, sizeof a.text, "%d UNBLOCK %d", a.time, cutEdges[k]);
    evs.pushBack(a);
  }
  for (int k = 0; k < pairs; ++k) {
    int kind = rnd(4);
    int t0 = rndRange(5, T * 8 / 10);
    int t1 = t0 + rndRange(evLenLo, evLenHi);
    if (t1 >= T) t1 = T - 1;
    Ev a, b;
    if (kind == 0 || kind == 1) {
      int e = rnd(ea.size()) + 1;
      a.time = t0;
      std::snprintf(a.text, sizeof a.text, "%d BLOCK %d", t0, e);
      b.time = t1;
      std::snprintf(b.text, sizeof b.text, "%d UNBLOCK %d", t1, e);
    } else if (kind == 2 && !stations.empty()) {
      int s = stations[rnd(stations.size())];
      a.time = t0;
      std::snprintf(a.text, sizeof a.text, "%d STATION_DOWN %d", t0, s);
      b.time = t1;
      std::snprintf(b.text, sizeof b.text, "%d STATION_UP %d", t1, s);
    } else {
      int r = rnd(R) + 1;
      a.time = t0;
      std::snprintf(a.text, sizeof a.text, "%d ROBOT_OFFLINE %d", t0, r);
      b.time = t1;
      std::snprintf(b.text, sizeof b.text, "%d ROBOT_ONLINE %d", t1, r);
    }
    evs.pushBack(a);
    evs.pushBack(b);
  }
  for (int k = 0; k < pairs / 2 + 1; ++k) {
    int q = rnd(Q);
    int t = reqTime[q] + rndRange(0, 15);
    if (t >= T) t = T - 1;
    Ev a;
    a.time = t;
    std::snprintf(a.text, sizeof a.text, "%d SET_URGENCY %d %d", t, q + 1, rndRange(1, 5));
    evs.pushBack(a);
  }
  rr::stableSort(evs, EvLess());
  std::printf("\nEVENTS %d\n# <time> <type> <target> [value]\n", evs.size());
  for (int i = 0; i < evs.size(); ++i) std::printf("%s\n", evs[i].text);
  std::printf("\nEND\n");
  return 0;
}
