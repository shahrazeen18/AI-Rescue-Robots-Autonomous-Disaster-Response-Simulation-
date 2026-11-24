// main.cpp - command-line driver for the AI Rescue Robots simulator.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <unistd.h>

#include <chrono>

#include "logger.h"
#include "scenario.h"
#include "simulation.h"

using namespace std;

using namespace rr;

namespace {

void usage(FILE* f) {
  fprintf(f,
      "AI Rescue Robots simulator (Submission 2: final)\n"
      "\n"
      "Usage: rescue_sim <scenario-file> [options]\n"
      "\n"
      "Options:\n"
      "  --validate         only load and validate the scenario, then exit\n"
      "  --out <file>       write the simulation record (CSV) to <file>\n"
      "  --stats-out <file> write the final statistics to <file>\n"
      "  --steps <n>        run only n time steps, then report the state\n"
      "  --interactive      step-by-step command mode (type 'help')\n"
      "  --show <what>      after the run print env | robots | requests | all\n"
      "  --check            verify internal invariants after every step\n"
      "  --quiet            suppress the load summary and final statistics\n"
      "  --help             this text\n"
      "\n"
      "Exit codes: 0 ok, 1 invalid scenario, 2 usage / file error,\n"
      "            3 internal invariant violated (--check)\n");
}

void interactiveHelp() {
  printf(
      "Commands:\n"
      "  step [n]   advance n time steps (default 1)\n"
      "  run        run to the end of the simulation\n"
      "  env        show environment state\n"
      "  robots     show robot states\n"
      "  requests   show pending / assigned / completed / failed requests\n"
      "  stats      show statistics so far\n"
      "  help       this text\n"
      "  quit       leave (the record and statistics are still written)\n");
}

struct Options {
  const char* scenario = nullptr;
  const char* out = nullptr;
  const char* statsOut = nullptr;
  int steps = -1;
  bool validateOnly = false;
  bool interactive = false;
  bool check = false;
  bool quiet = false;
  bool showEnv = false, showRobots = false, showRequests = false;
};

// Advances up to n steps; returns false on invariant failure.
bool advance(Simulation& sim, int n, bool check) {
  char msg[256];
  for (int i = 0; i < n; ++i) {
    if (!sim.step()) break;
    if (check && !sim.checkInvariants(msg, sizeof msg)) {
      fprintf(stderr, "INVARIANT VIOLATION at t=%d: %s\n", sim.time() - 1, msg);
      return false;
    }
  }
  return true;
}

int interactiveLoop(Simulation& sim, bool check) {
  bool tty = isatty(0) != 0;
  char line[256];
  interactiveHelp();
  for (;;) {
    if (tty) {
      printf("[t=%d] > ", sim.time());
      fflush(stdout);
    }
    if (!fgets(line, sizeof line, stdin)) break;
    char cmd[64] = "";
    int n = 1;
    int got = sscanf(line, "%63s %d", cmd, &n);
    if (got < 1) continue;
    if (got < 2) n = 1;
    if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) break;
    if (strcmp(cmd, "help") == 0) {
      interactiveHelp();
    } else if (strcmp(cmd, "step") == 0) {
      if (n < 1) n = 1;
      int before = sim.time();
      if (!advance(sim, n, check)) return 3;
      printf("advanced %d step(s); now at t=%d%s\n", sim.time() - before,
                  sim.time(), sim.finished() ? " (simulation finished)" : "");
    } else if (strcmp(cmd, "run") == 0) {
      if (!advance(sim, 1 << 30, check)) return 3;
      printf("finished at t=%d: %s\n", sim.time(), sim.terminationReason());
    } else if (strcmp(cmd, "env") == 0) {
      sim.printEnvironment(stdout);
    } else if (strcmp(cmd, "robots") == 0) {
      sim.printRobots(stdout);
    } else if (strcmp(cmd, "requests") == 0) {
      sim.printRequests(stdout);
    } else if (strcmp(cmd, "stats") == 0) {
      sim.printStats(stdout);
    } else {
      printf("unknown command '%s' (type 'help')\n", cmd);
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const char* a = argv[i];
    if (strcmp(a, "--help") == 0 || strcmp(a, "-h") == 0) {
      usage(stdout);
      return 0;
    } else if (strcmp(a, "--validate") == 0) {
      opt.validateOnly = true;
    } else if (strcmp(a, "--interactive") == 0) {
      opt.interactive = true;
    } else if (strcmp(a, "--check") == 0) {
      opt.check = true;
    } else if (strcmp(a, "--quiet") == 0) {
      opt.quiet = true;
    } else if (strcmp(a, "--out") == 0 && i + 1 < argc) {
      opt.out = argv[++i];
    } else if (strcmp(a, "--stats-out") == 0 && i + 1 < argc) {
      opt.statsOut = argv[++i];
    } else if (strcmp(a, "--steps") == 0 && i + 1 < argc) {
      char* end = nullptr;
      long v = strtol(argv[++i], &end, 10);
      if (*end != '\0' || v < 0 || v > 100000000) {
        fprintf(stderr, "invalid value for --steps\n");
        return 2;
      }
      opt.steps = static_cast<int>(v);
    } else if (strcmp(a, "--show") == 0 && i + 1 < argc) {
      const char* w = argv[++i];
      if (strcmp(w, "env") == 0) opt.showEnv = true;
      else if (strcmp(w, "robots") == 0) opt.showRobots = true;
      else if (strcmp(w, "requests") == 0) opt.showRequests = true;
      else if (strcmp(w, "all") == 0)
        opt.showEnv = opt.showRobots = opt.showRequests = true;
      else {
        fprintf(stderr, "invalid value for --show\n");
        return 2;
      }
    } else if (a[0] != '-' && !opt.scenario) {
      opt.scenario = a;
    } else {
      fprintf(stderr, "unknown or incomplete option '%s'\n\n", a);
      usage(stderr);
      return 2;
    }
  }
  if (!opt.scenario) {
    usage(stderr);
    return 2;
  }

  Scenario scenario;
  LoadReport report;
  chrono::steady_clock::time_point loadStart = chrono::steady_clock::now();
  loadScenario(opt.scenario, scenario, report);
  double loadSeconds = chrono::duration<double>(
      chrono::steady_clock::now() - loadStart).count();
  printLoadReport(stderr, report);
  if (!report.ok()) {
    fprintf(stderr, "Scenario '%s' is INVALID (%d error%s); nothing was run.\n",
                 opt.scenario, report.errors.size(),
                 report.errors.size() == 1 ? "" : "s");
    return 1;
  }
  if (!opt.quiet) {
    printf("Loaded scenario '%s': %d locations, %d connections (%d blocked), "
                "%d robots, %d requests, %d events, max_steps %d\n",
                scenario.params.name, scenario.env.numLocations(),
                scenario.env.numConnections(), scenario.env.countBlocked(),
                scenario.robots.size(), scenario.requests.size(),
                scenario.events.size(), scenario.params.maxSteps);
  }
  if (opt.validateOnly) {
    printf("Scenario is valid.\n");
    return 0;
  }

  Logger logger;
  if (opt.out && !logger.open(opt.out)) {
    fprintf(stderr, "cannot write record file '%s'\n", opt.out);
    return 2;
  }
  Simulation sim(scenario, opt.out ? &logger : nullptr);

  int code = 0;
  if (opt.interactive) {
    code = interactiveLoop(sim, opt.check);
  } else {
    int n = opt.steps >= 0 ? opt.steps : (1 << 30);
    if (!advance(sim, n, opt.check)) code = 3;
  }
  sim.finish();
  if (code != 0) return code;

  if (opt.showEnv) sim.printEnvironment(stdout);
  if (opt.showRobots) sim.printRobots(stdout);
  if (opt.showRequests) sim.printRequests(stdout);
  if (!opt.quiet) sim.printStats(stdout);
  if (opt.statsOut) {
    FILE* f = fopen(opt.statsOut, "w");
    if (!f) {
      fprintf(stderr, "cannot write statistics file '%s'\n", opt.statsOut);
      return 2;
    }
    sim.printStats(f);
    fprintf(f, "Load + validate time   : %.6f s\n", loadSeconds);
    fclose(f);
  }
  return 0;
}
