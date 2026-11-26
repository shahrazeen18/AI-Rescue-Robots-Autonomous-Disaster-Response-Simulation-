#include "scenario.h"

#include <cctype>
#include <cstdarg>
#include <cstring>

#include "int_map.h"
#include "sort_util.h"

using namespace std;

namespace rr {

namespace {

const int LINE_CAP = 1024;
const int MAX_TOKENS = 8;
const long long ID_MAX = 1000000000LL;
const int MAX_MESSAGES = 100;

enum Section { SEC_LOC, SEC_CONN, SEC_ROB, SEC_REQ, SEC_EVT, NUM_SECTIONS };
const char* const kSectionNames[NUM_SECTIONS] = {
    "LOCATIONS", "CONNECTIONS", "ROBOTS", "REQUESTS", "EVENTS"};

struct RawConn {
  int line, id, a, b, cost;
  bool blocked;
};
struct RawRobot {
  int line, id, start, cap, maxBat, initBat;
};
struct RawReq {
  int line, id, time, loc, cat, urg, dur;
};
struct RawEvent {
  int line, time, type, a, b;
};

// Splits a line in place on spaces/tabs. Returns the token count, or
// MAX_TOKENS + 1 if there are more tokens than fit.
int tokenize(char* line, char** tok) {
  int n = 0;
  char* p = line;
  for (;;) {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (*p == '\0') break;
    if (n == MAX_TOKENS) return MAX_TOKENS + 1;
    tok[n++] = p;
    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
    if (*p) *p++ = '\0';
  }
  return n;
}

// Strict decimal integer: optional '-', 1..10 digits, nothing else.
bool parseInt(const char* s, long long& out) {
  bool neg = false;
  if (*s == '-') {
    neg = true;
    ++s;
  }
  int digits = 0;
  long long v = 0;
  while (*s >= '0' && *s <= '9') {
    v = v * 10 + (*s - '0');
    ++s;
    if (++digits > 10) return false;
  }
  if (digits == 0 || *s != '\0') return false;
  out = neg ? -v : v;
  return true;
}

bool looksNumeric(const char* s) {
  return (*s >= '0' && *s <= '9') || (*s == '-' && s[1] >= '0' && s[1] <= '9');
}

bool isKeyword(const char* s) {
  if (strcmp(s, "SCENARIO") == 0 || strcmp(s, "PARAM") == 0 ||
      strcmp(s, "END") == 0) {
    return true;
  }
  for (int i = 0; i < NUM_SECTIONS; ++i) {
    if (strcmp(s, kSectionNames[i]) == 0) return true;
  }
  return false;
}

bool readLine(FILE* f, char* buf, int cap, bool& tooLong) {
  if (!fgets(buf, cap, f)) return false;
  tooLong = false;
  size_t n = strlen(buf);
  if (n == static_cast<size_t>(cap - 1) && buf[n - 1] != '\n') {
    tooLong = true;
    int c;
    while ((c = fgetc(f)) != EOF && c != '\n') {
    }
  }
  return true;
}

class Loader {
 public:
  Loader(Scenario& s, LoadReport& r) : s_(s), r_(r) {
    for (int i = 0; i < NUM_SECTIONS; ++i) seen_[i] = false;
    gotMaxSteps_ = false;
  }

  void parse(FILE* f);
  void build();

 private:
  void error(int line, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
  void warning(int line, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
  void add(Vector<Message>& v, int line, const char* fmt, va_list ap);

  bool intField(int line, const char* what, const char* tok, long long lo,
                long long hi, long long& out);
  bool fieldCount(int line, const char* sec, int n, int expected);

  void parseParam(int line, char** t, int n);
  void parseRecord(int sec, int line, char** t, int n);
  void parseLocation(int line, char** t, int n);
  void parseConnection(int line, char** t, int n);
  void parseRobot(int line, char** t, int n);
  void parseRequest(int line, char** t, int n);
  void parseEvent(int line, char** t, int n);

  Scenario& s_;
  LoadReport& r_;
  bool seen_[NUM_SECTIONS];
  bool gotMaxSteps_;
  Vector<RawConn> conns_;
  Vector<RawRobot> robots_;
  Vector<RawReq> reqs_;
  Vector<RawEvent> events_;
};

void Loader::add(Vector<Message>& v, int line, const char* fmt, va_list ap) {
  if (v.size() > MAX_MESSAGES) return;
  Message m;
  m.line = line;
  if (v.size() == MAX_MESSAGES) {
    snprintf(m.text, sizeof m.text, "further messages suppressed");
  } else {
    vsnprintf(m.text, sizeof m.text, fmt, ap);
  }
  v.pushBack(m);
}

void Loader::error(int line, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  add(r_.errors, line, fmt, ap);
  va_end(ap);
}

void Loader::warning(int line, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  add(r_.warnings, line, fmt, ap);
  va_end(ap);
}

bool Loader::intField(int line, const char* what, const char* tok,
                      long long lo, long long hi, long long& out) {
  long long v;
  if (!parseInt(tok, v)) {
    error(line, "%s '%s' is not a valid integer", what, tok);
    return false;
  }
  if (v < lo || v > hi) {
    error(line, "%s %lld is out of range [%lld, %lld]", what, v, lo, hi);
    return false;
  }
  out = v;
  return true;
}

bool Loader::fieldCount(int line, const char* sec, int n, int expected) {
  if (n != expected) {
    error(line, "%s record needs %d fields but has %s%d", sec, expected,
          n > MAX_TOKENS ? "more than " : "", n > MAX_TOKENS ? MAX_TOKENS : n);
    return false;
  }
  return true;
}

void Loader::parseParam(int line, char** t, int n) {
  if (n != 3) {
    error(line, "PARAM needs a key and a value");
    return;
  }
  const char* key = t[1];
  struct Spec {
    const char* name;
    int* slot;
    long long lo, hi;
  };
  Params& p = s_.params;
  Spec specs[] = {
      {"max_steps", &p.maxSteps, 1, 10000000},
      {"speed", &p.speed, 1, 1000000},
      {"battery_per_cost_unit", &p.batteryPerCostUnit, 0, 1000000},
      {"battery_per_work_step", &p.batteryPerWorkStep, 0, 1000000},
      {"recharge_per_step", &p.rechargePerStep, 0, 1000000},
      {"stop_when_done", &p.stopWhenDone, 0, 1},
      {"max_reassign", &p.maxReassign, 0, 1000},
      {"low_battery_pct", &p.lowBatteryPct, 0, 100},
      {"aging_steps", &p.agingSteps, 0, 1000000},
      {"route_strategy", &p.routeStrategy, 0, 2},
  };
  for (unsigned i = 0; i < sizeof specs / sizeof specs[0]; ++i) {
    if (strcmp(key, specs[i].name) == 0) {
      long long v;
      if (intField(line, key, t[2], specs[i].lo, specs[i].hi, v)) {
        *specs[i].slot = static_cast<int>(v);
        if (i == 0) gotMaxSteps_ = true;
      }
      return;
    }
  }
  warning(line, "unknown PARAM '%s' ignored", key);
}

void Loader::parseLocation(int line, char** t, int n) {
  if (!fieldCount(line, "LOCATIONS", n, 3)) return;
  long long id;
  bool ok = intField(line, "location id", t[0], 0, ID_MAX, id);
  if (strlen(t[1]) >= static_cast<size_t>(NAME_LEN)) {
    error(line, "location name '%s' is longer than %d characters", t[1],
          NAME_LEN - 1);
    ok = false;
  }
  int type = parseLocType(t[2]);
  if (type < 0) {
    error(line, "unknown location type '%s'", t[2]);
    ok = false;
  }
  if (!ok) return;
  if (s_.env.addLocation(static_cast<int>(id), t[1], type) < 0) {
    error(line, "duplicate location id %lld", id);
  }
}

void Loader::parseConnection(int line, char** t, int n) {
  if (!fieldCount(line, "CONNECTIONS", n, 5)) return;
  long long id, a, b, cost;
  bool ok = intField(line, "connection id", t[0], 0, ID_MAX, id);
  ok = intField(line, "connection endpoint", t[1], 0, ID_MAX, a) && ok;
  ok = intField(line, "connection endpoint", t[2], 0, ID_MAX, b) && ok;
  ok = intField(line, "connection cost", t[3], 1, 1000000, cost) && ok;
  bool blocked = false;
  if (strcmp(t[4], "OPEN") == 0) {
    blocked = false;
  } else if (strcmp(t[4], "BLOCKED") == 0) {
    blocked = true;
  } else {
    error(line, "connection status '%s' must be OPEN or BLOCKED", t[4]);
    ok = false;
  }
  if (!ok) return;
  RawConn c;
  c.line = line;
  c.id = static_cast<int>(id);
  c.a = static_cast<int>(a);
  c.b = static_cast<int>(b);
  c.cost = static_cast<int>(cost);
  c.blocked = blocked;
  conns_.pushBack(c);
}

void Loader::parseRobot(int line, char** t, int n) {
  if (!fieldCount(line, "ROBOTS", n, 5)) return;
  long long id, start, maxBat, initBat;
  bool ok = intField(line, "robot id", t[0], 0, ID_MAX, id);
  ok = intField(line, "robot start location", t[1], 0, ID_MAX, start) && ok;
  int cap = parseCapability(t[2]);
  if (cap < 0) {
    error(line, "unknown robot capability '%s'", t[2]);
    ok = false;
  }
  ok = intField(line, "robot max_battery", t[3], 1, 1000000, maxBat) && ok;
  ok = intField(line, "robot initial_battery", t[4], 0, 1000000, initBat) && ok;
  if (ok && initBat > maxBat) {
    error(line, "robot %lld initial battery %lld exceeds max battery %lld", id,
          initBat, maxBat);
    ok = false;
  }
  if (!ok) return;
  RawRobot r;
  r.line = line;
  r.id = static_cast<int>(id);
  r.start = static_cast<int>(start);
  r.cap = cap;
  r.maxBat = static_cast<int>(maxBat);
  r.initBat = static_cast<int>(initBat);
  robots_.pushBack(r);
}

void Loader::parseRequest(int line, char** t, int n) {
  if (!fieldCount(line, "REQUESTS", n, 6)) return;
  long long id, time, loc, urg, dur;
  bool ok = intField(line, "request id", t[0], 0, ID_MAX, id);
  ok = intField(line, "request report_time", t[1], 0, 100000000, time) && ok;
  ok = intField(line, "request location", t[2], 0, ID_MAX, loc) && ok;
  int cat = parseCategory(t[3]);
  if (cat < 0) {
    error(line, "unknown request category '%s'", t[3]);
    ok = false;
  }
  ok = intField(line, "request urgency", t[4], 1, 5, urg) && ok;
  ok = intField(line, "request duration_steps", t[5], 1, 1000000, dur) && ok;
  if (!ok) return;
  RawReq q;
  q.line = line;
  q.id = static_cast<int>(id);
  q.time = static_cast<int>(time);
  q.loc = static_cast<int>(loc);
  q.cat = cat;
  q.urg = static_cast<int>(urg);
  q.dur = static_cast<int>(dur);
  reqs_.pushBack(q);
}

void Loader::parseEvent(int line, char** t, int n) {
  if (n < 2) {
    error(line, "EVENTS record needs a time and an event type");
    return;
  }
  long long time;
  bool ok = intField(line, "event time", t[0], 0, 100000000, time);
  int type = parseEventType(t[1]);
  if (type < 0) {
    error(line, "unknown event type '%s'", t[1]);
    return;
  }
  int expected = type == EV_SET_URGENCY ? 4 : 3;
  if (n != expected) {
    error(line, "%s event needs %d fields", t[1], expected);
    return;
  }
  long long a = 0, b = 0;
  ok = intField(line, "event target id", t[2], 0, ID_MAX, a) && ok;
  if (type == EV_SET_URGENCY) {
    ok = intField(line, "event urgency", t[3], 1, 5, b) && ok;
  }
  if (!ok) return;
  RawEvent e;
  e.line = line;
  e.time = static_cast<int>(time);
  e.type = type;
  e.a = static_cast<int>(a);
  e.b = static_cast<int>(b);
  events_.pushBack(e);
}

void Loader::parseRecord(int sec, int line, char** t, int n) {
  switch (sec) {
    case SEC_LOC: parseLocation(line, t, n); break;
    case SEC_CONN: parseConnection(line, t, n); break;
    case SEC_ROB: parseRobot(line, t, n); break;
    case SEC_REQ: parseRequest(line, t, n); break;
    default: parseEvent(line, t, n); break;
  }
}

void Loader::parse(FILE* f) {
  char buf[LINE_CAP];
  char* tok[MAX_TOKENS + 1];
  int lineNo = 0;
  int section = -1;
  int remaining = 0, declared = 0;
  bool sawEnd = false, reportedTrailing = false;
  bool tooLong = false;

  while (readLine(f, buf, LINE_CAP, tooLong)) {
    ++lineNo;
    char* line = buf;
    if (lineNo == 1 && static_cast<unsigned char>(line[0]) == 0xEF &&
        static_cast<unsigned char>(line[1]) == 0xBB &&
        static_cast<unsigned char>(line[2]) == 0xBF) {
      line += 3;  // UTF-8 byte-order mark
    }
    if (tooLong) {
      error(lineNo, "line is longer than %d characters", LINE_CAP - 2);
      continue;
    }
    int n = tokenize(line, tok);
    if (n == 0 || tok[0][0] == '#') continue;  // blank or comment
    if (sawEnd) {
      if (!reportedTrailing) error(lineNo, "content found after END");
      reportedTrailing = true;
      continue;
    }
    if (remaining > 0) {
      if (isKeyword(tok[0])) {
        error(lineNo,
              "section %s declares %d records but only %d were found before "
              "'%s'",
              kSectionNames[section], declared, declared - remaining, tok[0]);
        remaining = 0;
      } else {
        parseRecord(section, lineNo, tok, n);
        --remaining;
        continue;
      }
    }
    // Header level: SCENARIO / PARAM / section header / END.
    if (strcmp(tok[0], "SCENARIO") == 0) {
      if (n != 2) {
        error(lineNo, "SCENARIO needs exactly one name");
      } else if (strlen(tok[1]) >= sizeof s_.params.name) {
        error(lineNo, "scenario name is too long");
      } else {
        strcpy(s_.params.name, tok[1]);
      }
    } else if (strcmp(tok[0], "PARAM") == 0) {
      parseParam(lineNo, tok, n);
    } else if (strcmp(tok[0], "END") == 0) {
      if (n != 1) error(lineNo, "END must be alone on its line");
      sawEnd = true;
    } else {
      int sec = -1;
      for (int i = 0; i < NUM_SECTIONS; ++i) {
        if (strcmp(tok[0], kSectionNames[i]) == 0) sec = i;
      }
      if (sec >= 0) {
        long long count = 0;
        if (n != 2) {
          error(lineNo, "header must look like '%s <count>'", tok[0]);
          section = -1;
        } else if (!intField(lineNo, "record count", tok[1], 0, 1000000,
                             count)) {
          section = -1;
        } else {
          if (seen_[sec]) error(lineNo, "section %s appears twice", tok[0]);
          seen_[sec] = true;
          section = sec;
          declared = remaining = static_cast<int>(count);
        }
      } else if (looksNumeric(tok[0])) {
        error(lineNo,
              "unexpected record: more records than the declared count, or a "
              "record outside any section");
      } else {
        error(lineNo, "unknown keyword '%s'", tok[0]);
      }
    }
  }
  if (remaining > 0) {
    error(lineNo, "section %s declares %d records but only %d were found",
          kSectionNames[section], declared, declared - remaining);
  }
  if (!sawEnd) error(lineNo, "missing END line");
  for (int i = 0; i < NUM_SECTIONS; ++i) {
    if (!seen_[i] && i != SEC_EVT) {
      error(0, "required section %s is missing", kSectionNames[i]);
    }
  }
  if (!gotMaxSteps_) error(0, "PARAM max_steps is required");
}

struct EventLess {
  bool operator()(const ScenarioEvent& a, const ScenarioEvent& b) const {
    return a.time < b.time;
  }
};

void Loader::build() {
  Environment& env = s_.env;

  // Connections: endpoints must exist and differ.
  for (int i = 0; i < conns_.size(); ++i) {
    const RawConn& c = conns_[i];
    int a = env.locationIndex(c.a), b = env.locationIndex(c.b);
    bool ok = true;
    if (a < 0) {
      error(c.line, "connection %d references unknown location %d", c.id, c.a);
      ok = false;
    }
    if (b < 0) {
      error(c.line, "connection %d references unknown location %d", c.id, c.b);
      ok = false;
    }
    if (ok && a == b) {
      error(c.line, "connection %d connects location %d to itself", c.id, c.a);
      ok = false;
    }
    if (!ok) continue;
    if (env.addConnection(c.id, a, b, c.cost, c.blocked) < 0) {
      error(c.line, "duplicate connection id %d", c.id);
    }
  }
  env.finalize();

  // Robots.
  IntMap robotIdx;
  for (int i = 0; i < robots_.size(); ++i) {
    const RawRobot& r = robots_[i];
    int loc = env.locationIndex(r.start);
    if (loc < 0) {
      error(r.line, "robot %d starts at unknown location %d", r.id, r.start);
      continue;
    }
    if (!robotIdx.insert(r.id, s_.robots.size())) {
      error(r.line, "duplicate robot id %d", r.id);
      continue;
    }
    Robot rb;
    rb.id = r.id;
    rb.location = loc;
    rb.capability = r.cap;
    rb.maxBattery = r.maxBat;
    rb.battery = r.initBat;
    s_.robots.pushBack(rb);
  }

  // Requests.
  IntMap reqIdx;
  for (int i = 0; i < reqs_.size(); ++i) {
    const RawReq& q = reqs_[i];
    int loc = env.locationIndex(q.loc);
    if (loc < 0) {
      error(q.line, "request %d is at unknown location %d", q.id, q.loc);
      continue;
    }
    if (!reqIdx.insert(q.id, s_.requests.size())) {
      error(q.line, "duplicate request id %d", q.id);
      continue;
    }
    Request rq;
    rq.id = q.id;
    rq.reportTime = q.time;
    rq.location = loc;
    rq.category = q.cat;
    rq.urgency = q.urg;
    rq.duration = q.dur;
    s_.requests.pushBack(rq);
  }

  // Events: resolve external ids to internal indices.
  for (int i = 0; i < events_.size(); ++i) {
    const RawEvent& e = events_[i];
    ScenarioEvent se;
    se.time = e.time;
    se.type = e.type;
    se.value = e.b;
    int target = -1;
    switch (e.type) {
      case EV_BLOCK:
      case EV_UNBLOCK:
        target = env.connectionIndex(e.a);
        if (target < 0) error(e.line, "event references unknown connection %d", e.a);
        break;
      case EV_STATION_DOWN:
      case EV_STATION_UP:
        target = env.locationIndex(e.a);
        if (target < 0) {
          error(e.line, "event references unknown location %d", e.a);
        } else if (!env.isStation(target)) {
          error(e.line, "location %d is not a CHARGING station", e.a);
          target = -1;
        }
        break;
      case EV_SET_URGENCY:
        target = reqIdx.get(e.a, -1);
        if (target < 0) error(e.line, "event references unknown request %d", e.a);
        break;
      default:
        target = robotIdx.get(e.a, -1);
        if (target < 0) error(e.line, "event references unknown robot %d", e.a);
        break;
    }
    if (target < 0) continue;
    se.target = target;
    s_.events.pushBack(se);
  }
  stableSort(s_.events, EventLess());

  // Warnings (the scenario is still usable).
  if (s_.params.name[0] == '\0') strcpy(s_.params.name, "unnamed");
  if (env.numLocations() > 0 && env.countComponents() > 1) {
    warning(0, "the road network has %d disconnected parts even when every "
               "connection is open",
            env.countComponents());
  }
  bool anyBlockEvent = false;
  for (int i = 0; i < s_.events.size(); ++i) {
    if (s_.events[i].type == EV_BLOCK) anyBlockEvent = true;
  }
  if (env.countBlocked() == 0 && !anyBlockEvent) {
    warning(0, "no blocked connection and no BLOCK event: the environment has "
               "no restriction (required by the project specification)");
  }
  if (s_.robots.empty()) warning(0, "scenario has no robots");
  if (s_.requests.empty()) warning(0, "scenario has no requests");
  if (!seen_[SEC_EVT]) warning(0, "no EVENTS section: nothing changes during the run");
}

}  // namespace

bool loadScenario(const char* path, Scenario& out, LoadReport& report) {
  FILE* f = fopen(path, "r");
  if (!f) {
    Message m;
    snprintf(m.text, sizeof m.text, "cannot open file '%s'", path);
    report.errors.pushBack(m);
    return false;
  }
  Loader loader(out, report);
  loader.parse(f);
  fclose(f);
  loader.build();
  return report.ok();
}

void printLoadReport(FILE* f, const LoadReport& report) {
  for (int i = 0; i < report.errors.size(); ++i) {
    const Message& m = report.errors[i];
    if (m.line > 0) {
      fprintf(f, "ERROR   line %d: %s\n", m.line, m.text);
    } else {
      fprintf(f, "ERROR   %s\n", m.text);
    }
  }
  for (int i = 0; i < report.warnings.size(); ++i) {
    const Message& m = report.warnings[i];
    if (m.line > 0) {
      fprintf(f, "WARNING line %d: %s\n", m.line, m.text);
    } else {
      fprintf(f, "WARNING %s\n", m.text);
    }
  }
}

}  
