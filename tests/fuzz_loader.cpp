// fuzz_loader.cpp - mutation fuzzer for the scenario loader and the simulator.
//
// Takes a valid scenario file, applies random mutations (delete / duplicate /
// swap lines, replace tokens with garbage or extreme numbers, truncate, flip
// bytes, insert very long lines) and feeds each mutant to loadScenario().
// If the loader accepts a mutant, the simulation is run for up to 400 steps
// with the invariant checker after every step. The program must never crash,
// hang or violate an invariant (build it with -fsanitize=address,undefined to
// also catch memory errors and undefined behaviour).
//
// Usage: fuzz_loader <seed-scenario> <iterations> <rng-seed> [work-file]
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "logger.h"
#include "scenario.h"
#include "simulation.h"
#include "vector.h"

using namespace std;

using namespace rr;

static unsigned long long g_rng = 1;
static int rnd(int n) {
  g_rng = g_rng * 6364136223846793005ULL + 1442695040888963407ULL;
  return static_cast<int>((g_rng >> 33) % static_cast<unsigned long long>(n));
}

typedef Vector<char> Bytes;

static bool readFile(const char* path, Bytes& out) {
  FILE* f = fopen(path, "rb");
  if (!f) return false;
  char buf[4096];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) {
    for (size_t i = 0; i < n; ++i) out.pushBack(buf[i]);
  }
  fclose(f);
  return true;
}

static void writeFile(const char* path, const Bytes& b) {
  FILE* f = fopen(path, "wb");
  if (!f) return;
  if (b.size() > 0) fwrite(&b[0], 1, static_cast<size_t>(b.size()), f);
  fclose(f);
}

// Line boundaries: starts[i] .. starts[i+1]-1 (including the newline).
static void lineStarts(const Bytes& b, Vector<int>& starts) {
  starts.clear();
  starts.pushBack(0);
  for (int i = 0; i < b.size(); ++i) {
    if (b[i] == '\n') starts.pushBack(i + 1);
  }
  if (starts.back() != b.size()) starts.pushBack(b.size());
}

static void appendRange(const Bytes& src, int lo, int hi, Bytes& dst) {
  for (int i = lo; i < hi; ++i) dst.pushBack(src[i]);
}

static void appendStr(const char* s, Bytes& dst) {
  for (; *s; ++s) dst.pushBack(*s);
}

static const char* const kWords[] = {
    "OPEN", "BLOCKED", "MEDICAL", "FIRE", "SUPPLY", "SEARCH", "GENERAL",
    "JUNCTION", "CHARGING", "COMMAND", "END", "PARAM", "LOCATIONS",
    "CONNECTIONS", "ROBOTS", "REQUESTS", "EVENTS", "SCENARIO", "#", "abc",
    "BLOCK", "UNBLOCK", "STATION_DOWN", "SET_URGENCY", "-", "--5", "0x10",
    "1e9", "+3", ""};
static const char* const kNumbers[] = {
    "-1", "0", "1", "2", "5", "6", "100", "999999", "1000000", "1000001",
    "2147483647", "2147483648", "-2147483648", "9999999999", "99999999999",
    "4294967296", "1000000000"};

// Replaces the n-th whitespace-separated token of a line (or appends one).
static void mutateToken(const Bytes& src, int lo, int hi, Bytes& dst) {
  int tokens = 0;
  bool in = false;
  for (int i = lo; i < hi; ++i) {
    bool ws = src[i] == ' ' || src[i] == '\t' || src[i] == '\n' || src[i] == '\r';
    if (!ws && !in) ++tokens;
    in = !ws;
  }
  if (tokens == 0) {
    appendRange(src, lo, hi, dst);
    return;
  }
  int target = rnd(tokens), cur = -1;
  in = false;
  const char* repl = rnd(2) ? kNumbers[rnd(17)] : kWords[rnd(30)];
  for (int i = lo; i < hi; ++i) {
    bool ws = src[i] == ' ' || src[i] == '\t' || src[i] == '\n' || src[i] == '\r';
    if (!ws && !in) {
      ++cur;
      if (cur == target) appendStr(repl, dst);
    }
    in = !ws;
    if (cur != target || ws) dst.pushBack(src[i]);
  }
}

static void mutate(const Bytes& src, Bytes& dst) {
  dst.clear();
  Vector<int> st;
  lineStarts(src, st);
  int lines = st.size() - 1;
  if (lines <= 0) {
    appendRange(src, 0, src.size(), dst);
    return;
  }
  int op = rnd(9);
  int ln = rnd(lines);
  switch (op) {
    case 0:  // delete a line
      for (int i = 0; i < lines; ++i)
        if (i != ln) appendRange(src, st[i], st[i + 1], dst);
      break;
    case 1:  // duplicate a line
      for (int i = 0; i < lines; ++i) {
        appendRange(src, st[i], st[i + 1], dst);
        if (i == ln) appendRange(src, st[i], st[i + 1], dst);
      }
      break;
    case 2: {  // swap two lines
      int other = rnd(lines);
      for (int i = 0; i < lines; ++i) {
        int j = i == ln ? other : (i == other ? ln : i);
        appendRange(src, st[j], st[j + 1], dst);
      }
      break;
    }
    case 3:
    case 4:
    case 5:  // replace a token (most common mutation)
      for (int i = 0; i < lines; ++i) {
        if (i == ln) mutateToken(src, st[i], st[i + 1], dst);
        else appendRange(src, st[i], st[i + 1], dst);
      }
      break;
    case 6:  // truncate
      appendRange(src, 0, rnd(src.size() + 1), dst);
      break;
    case 7: {  // flip a byte
      appendRange(src, 0, src.size(), dst);
      if (dst.size() > 0) dst[rnd(dst.size())] = static_cast<char>(rnd(256));
      break;
    }
    default:  // insert an over-long line
      for (int i = 0; i < lines; ++i) {
        if (i == ln) {
          for (int k = 0; k < 1500 + rnd(1500); ++k) dst.pushBack('x');
          dst.pushBack('\n');
        }
        appendRange(src, st[i], st[i + 1], dst);
      }
      break;
  }
}

int main(int argc, char** argv) {
  if (argc < 4) {
    fprintf(stderr, "usage: %s <seed-scenario> <iterations> <rng-seed> [work-file]\n", argv[0]);
    return 2;
  }
  Bytes seed;
  if (!readFile(argv[1], seed)) {
    fprintf(stderr, "cannot read %s\n", argv[1]);
    return 2;
  }
  int iters = atoi(argv[2]);
  g_rng = static_cast<unsigned long long>(atoll(argv[3])) * 2654435761ULL + 99ULL;
  const char* work = argc > 4 ? argv[4] : "fuzz_work.txt";

  int accepted = 0, rejected = 0, simulated = 0;
  Bytes cur, next;
  for (int it = 0; it < iters; ++it) {
    cur.clear();
    appendRange(seed, 0, seed.size(), cur);
    int nmut = 1 + rnd(4);
    for (int m = 0; m < nmut; ++m) {
      mutate(cur, next);
      cur = next;
    }
    writeFile(work, cur);
    Scenario s;
    LoadReport r;
    bool ok = loadScenario(work, s, r);
    if (!ok) {
      if (r.errors.empty()) {
        printf("BUG: loader returned false without errors (iteration %d)\n", it);
        return 1;
      }
      ++rejected;
      continue;
    }
    ++accepted;
    Simulation sim(s, nullptr);
    char msg[256];
    bool violated = false;
    for (int step = 0; step < 400 && sim.step(); ++step) {
      if (!sim.checkInvariants(msg, sizeof msg)) {
        printf("BUG: invariant violated at iteration %d, t=%d: %s\n", it, sim.time(), msg);
        violated = true;
        break;
      }
    }
    if (violated) {
      printf("  failing mutant kept in %s\n", work);
      return 1;
    }
    ++simulated;
  }
  printf("fuzz: %d mutants, %d rejected by validator, %d accepted and simulated cleanly\n",
              iters, rejected, simulated);
  (void)accepted;
  return 0;
}
