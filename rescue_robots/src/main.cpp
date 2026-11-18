// main.cpp - command-line driver for the AI Rescue Robots simulator.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <unistd.h>

#include <chrono>

#include "logger.h"
#include "scenario.h"
#include "simulation.h"

using namespace rr;

namespace {

void usage(FILE* f) {
  std::fprintf(f,
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
  std::printf(
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
      std::fprintf(stderr, "INVARIANT VIOLATION at t=%d: %s\n", sim.time() - 1, msg);
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
      std::printf("[t=%d] > ", sim.time());
      std::fflush(stdout);
    }
    if (!std::fgets(line, sizeof line, stdin)) break;
    char cmd[64] = "";
    int n = 1;
    int got = std::sscanf(line, "%63s %d", cmd, &n);
    if (got < 1) continue;
    if (got < 2) n = 1;
    if (std::strcmp(cmd, "quit") == 0 || std::strcmp(cmd, "exit") == 0) break;
    if (std::strcmp(cmd, "help") == 0) {
      interactiveHelp();
    } else if (std::strcmp(cmd, "step") == 0) {
      if (n < 1) n = 1;
      int before = sim.time();
      if (!advance(sim, n, check)) return 3;
      std::printf("advanced %d step(s); now at t=%d%s\n", sim.time() - before,
                  sim.time(), sim.finished() ? " (simulation finished)" : "");
    } else if (std::strcmp(cmd, "run") == 0) {
      if (!advance(sim, 1 << 30, check)) return 3;
      std::printf("finished at t=%d: %s\n", sim.time(), sim.terminationReason());
    } else if (std::strcmp(cmd, "env") == 0) {
      sim.printEnvironment(stdout);
    } else if (std::strcmp(cmd, "robots") == 0) {
      sim.printRobots(stdout);
    } else if (std::strcmp(cmd, "requests") == 0) {
      sim.printRequests(stdout);
    } else if (std::strcmp(cmd, "stats") == 0) {
      sim.printStats(stdout);
    } else {
      std::printf("unknown command '%s' (type 'help')\n", cmd);
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const char* a = argv[i];
    if (std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0) {
      usage(stdout);
      return 0;
    } else if (std::strcmp(a, "--validate") == 0) {
      opt.validateOnly = true;
    } else if (std::strcmp(a, "--interactive") == 0) {
      opt.interactive = true;
    } else if (std::strcmp(a, "--check") == 0) {
      opt.check = true;
    } else if (std::strcmp(a, "--quiet") == 0) {
      opt.quiet = true;
    } else if (std::strcmp(a, "--out") == 0 && i + 1 < argc) {
      opt.out = argv[++i];
    } else if (std::strcmp(a, "--stats-out") == 0 && i + 1 < argc) {
      opt.statsOut = argv[++i];
    } else if (std::strcmp(a, "--steps") == 0 && i + 1 < argc) {
      char* end = nullptr;
      long v = std::strtol(argv[++i], &end, 10);
      if (*end != '\0' || v < 0 || v > 100000000) {
        std::fprintf(stderr, "invalid value for --steps\n");
        return 2;
      }
      opt.steps = static_cast<int>(v);
    } else if (std::strcmp(a, "--show") == 0 && i + 1 < argc) {
      const char* w = argv[++i];
      if (std::strcmp(w, "env") == 0) opt.showEnv = true;
      else if (std::strcmp(w, "robots") == 0) opt.showRobots = true;
      else if (std::strcmp(w, "requests") == 0) opt.showRequests = true;
      else if (std::strcmp(w, "all") == 0)
        opt.showEnv = opt.showRobots = opt.showRequests = true;
      else {
        std::fprintf(stderr, "invalid value for --show\n");
        return 2;
      }
    } else if (a[0] != '-' && !opt.scenario) {
      opt.scenario = a;
    } else {
      std::fprintf(stderr, "unknown or incomplete option '%s'\n\n", a);
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
  std::chrono::steady_clock::time_point loadStart = std::chrono::steady_clock::now();
  loadScenario(opt.scenario, scenario, report);
  double loadSeconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - loadStart).count();
  printLoadReport(stderr, report);
  if (!report.ok()) {
    std::fprintf(stderr, "Scenario '%s' is INVALID (%d error%s); nothing was run.\n",
                 opt.scenario, report.errors.size(),
                 report.errors.size() == 1 ? "" : "s");
    return 1;
  }
  if (!opt.quiet) {
    std::printf("Loaded scenario '%s': %d locations, %d connections (%d blocked), "
                "%d robots, %d requests, %d events, max_steps %d\n",
                scenario.params.name, scenario.env.numLocations(),
                scenario.env.numConnections(), scenario.env.countBlocked(),
                scenario.robots.size(), scenario.requests.size(),
                scenario.events.size(), scenario.params.maxSteps);
  }
  if (opt.validateOnly) {
    std::printf("Scenario is valid.\n");
    return 0;
  }

  Logger logger;
  if (opt.out && !logger.open(opt.out)) {
    std::fprintf(stderr, "cannot write record file '%s'\n", opt.out);
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
    FILE* f = std::fopen(opt.statsOut, "w");
    if (!f) {
      std::fprintf(stderr, "cannot write statistics file '%s'\n", opt.statsOut);
      return 2;
    }
    sim.printStats(f);
    std::fprintf(f, "Load + validate time   : %.6f s\n", loadSeconds);
    std::fclose(f);
  }
  return 0;
}
