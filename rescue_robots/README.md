# AI Rescue Robots — Final Project (Submission 2)

A deterministic, discrete-time rescue-coordination simulator in C++17 for the Data Structures Lab
(Fall 2026, DS/AI). A fleet of robots serves rescue requests in a disaster-damaged road network while
roads block and reopen, charging stations fail, robots go offline and priorities change.

**All core data structures are written from scratch** — dynamic array, hash map, binary heap, stable merge
sort, graph (CSR adjacency lists). There are **no STL containers and no `std::string`** (`make test`
scans the sources to prove it). Only the C library (`stdio`, `string.h`, …) and `<chrono>` for timing are used.

---

## 1. Quick start (Ubuntu / VS Code terminal)

```bash
sudo apt update && sudo apt install -y build-essential     # g++ and make (once)
cd rescue_robots
make                                                       # builds everything, no warnings
./rescue_sim data/scenario_provided.txt                    # run the provided scenario
make test                                                  # full verification suite (≈15 s)
```

Run every command from the project root (the `data/...` paths are relative).

## 2. Build

| Command | Result |
|---|---|
| `make` | `rescue_sim` (simulator), `gen_scenario` (scenario generator), `test_runner`, `fuzz_loader` — compiled with `g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic` |
| `make asan` | the same programs built with AddressSanitizer + UBSan into `build/*_asan` |
| `make test` | builds everything, then runs `tests/run_tests.sh` |
| `make bench` | runs the performance benchmark (`tools/benchmark.sh`) → `output/benchmark.csv` |
| `make clean` | removes build products |

## 3. Running the simulator

```
./rescue_sim <scenario-file> [options]
```

| Option | Meaning |
|---|---|
| `--out <file>` | write the simulation record (CSV, see §6) |
| `--stats-out <file>` | write the final statistics to a file as well |
| `--steps <n>` | run only `n` time steps, then report the state |
| `--interactive` | step-by-step mode: `step [n]`, `run`, `env`, `robots`, `requests`, `stats`, `help`, `quit` |
| `--show env\|robots\|requests\|all` | print the environment / robot states / pending-assigned-completed-failed requests after the run |
| `--validate` | only load and validate the scenario |
| `--check` | verify internal consistency invariants after every step (exit code 3 if violated) |
| `--quiet` | suppress the load summary and the final statistics on stdout |
| `--help` | usage |

Exit codes: `0` ok · `1` invalid scenario (nothing is run) · `2` usage / file error · `3` invariant violated.

Examples:

```bash
./rescue_sim data/scenario_provided.txt --out output/record.csv --stats-out output/stats.txt
./rescue_sim data/scenario_provided.txt --steps 60 --show all        # state at t = 60
./rescue_sim data/scenario_provided.txt --interactive
./rescue_sim data/scenario_stress.txt --check
./gen_scenario my_test 300 700 30 300 800 5 hard > data/my_test.txt  # make your own scenario
```

## 4. Project layout

```
src/        vector.h  int_map.h  min_heap.h  sort_util.h        own data structures (no STL)
            types.*  environment.*  robot.*  rescue_request.*    domain model
            scenario.*  simulation.*  logger.*  main.cpp          loader/validator, engine, record, CLI
data/       scenario_provided.txt (unchanged) + 10 more scenarios (§5)
tests/      test_main.cpp (unit + regression), fuzz_loader.cpp (mutation fuzzer), run_tests.sh,
            invalid/ (34 malformed inputs), golden/ (reference output), aging_off.txt, valid_base.txt
tools/      gen_scenario.cpp (deterministic generator), benchmark.sh, plot_benchmark.py (optional charts)
output/     record + statistics from running every scenario, benchmark.csv
docs/       report.pdf / report.md, benchmark.png
```

## 5. Scenarios (all run without any source change)

| File | Size (locations / connections / robots / requests / steps) | Purpose |
|---|---|---|
| `scenario_tiny.txt` | 8 / 10 / 3 / 6 / 60 | basic lifecycle, one blocked road restored later |
| `scenario_reroute.txt` | 7 / 8 / 2 / 3 / 120 | road blocked **while a robot is travelling** → reroute; a cut-off location that waits for a road |
| `scenario_conflict.txt` | 6 / 6 / 3 / 6 / 100 | several candidate robots, one robot / several requests, no capable robot (request FAILS), urgency change alters the dispatch order |
| `scenario_battery.txt` | 6 / 7 / 2 / 2 / 150 | insufficient battery → charging, station goes DOWN → divert, robot offline mid-task → reassignment |
| `scenario_aging.txt` (+ `tests/aging_off.txt`) | 4 / 4 / 1 / 32 / 150 | starvation demo: an urgency-1 request vs. a stream of urgency-5 requests, with and without aging |
| **Level 1 – easy** `scenario_level1_easy.txt` | 60 / 130 / 12 / 60 / 250 | generated, few disruptions, generous batteries |
| **Level 2 – moderate** `scenario_provided.txt` | 56 / 117 / 12 / 60 / 250 | the supplied minimum working scenario |
| **Level 3 – hard** `scenario_level3_hard.txt` | 100 / 215 / 8 / 110 / 400 | generated overload: request burst, 28 roads blocked, two cut-off locations, 107 environment changes, small batteries, scarce capabilities; 33 requests are still unresolved at the end |
| **Stress** `scenario_stress.txt` | 120 / 260 / 24 / 120 / 600 | required stress scenario (≥ 2× the minimum working sizes) |
| `scenario_medium.txt` | 80 / 170 / 16 / 80 / 350 | generated, between minimum and stress |
| `scenario_scale_1000.txt` | 1000 / 2500 / 50 / 500 / 1000 | scalability demonstration |

Regenerate any generated file: `./gen_scenario <name> <locations> <connections> <robots> <requests> <max_steps> <seed> [normal|easy|hard]`
(e.g. `./gen_scenario level3_hard 100 215 8 110 400 33 hard`). The generator uses its own random generator, so the same
arguments always give the same file.

## 6. Input format

Exactly the format of `scenario_provided.txt`: lines starting with `#` and blank lines are ignored; `SCENARIO <name>`;
`PARAM <key> <integer>`; five sections `LOCATIONS`, `CONNECTIONS`, `ROBOTS`, `REQUESTS`, `EVENTS`, each written as
`KEYWORD <count>` followed by exactly `<count>` records, one per line; the file ends with `END`. Tokens are separated
by spaces or tabs; ids are non-negative integers and need not be consecutive. Records with the same time keep their
file order.

```
LOCATIONS    id name type                                  JUNCTION BUILDING HOSPITAL DEPOT CHARGING COMMAND
CONNECTIONS  id location_a location_b cost OPEN|BLOCKED    undirected, cost >= 1
ROBOTS       id start_location capability max_battery initial_battery   MEDICAL FIRE SUPPLY SEARCH GENERAL
REQUESTS     id report_time location category urgency duration_steps    category: MEDICAL FIRE SUPPLY SEARCH; urgency 1..5
EVENTS       time BLOCK|UNBLOCK <connection_id>
             time STATION_DOWN|STATION_UP <location_id>
             time SET_URGENCY <request_id> <new_urgency>
             time ROBOT_OFFLINE|ROBOT_ONLINE <robot_id>
```

**Parameters.** Required: `max_steps`. The five parameters of the provided file are used as described in §8:
`speed` (default 1), `battery_per_cost_unit` (1), `battery_per_work_step` (2), `recharge_per_step` (5).
**Our optional extensions** (not used by the provided file; defaults shown, all documented here):

| `PARAM` | Default | Meaning |
|---|---|---|
| `stop_when_done` | 1 | `1`: stop as soon as every request is COMPLETED or FAILED; `0`: always run `max_steps` steps |
| `max_reassign` | 5 | a request FAILS after being released by robots more than this many times |
| `low_battery_pct` | 50 | an idle robot below this % of its maximum battery goes to charge |
| `aging_steps` | 0 (off) | with `n > 0` a waiting request gains +1 effective urgency per `n` steps waited (prevents starvation) |
| `route_strategy` | 0 | `0` automatic, `1` force one search per request, `2` force one search per idle robot (testing/benchmarking only; see §9) |

Unknown `PARAM` keys give a warning and are ignored.

**Validation.** Every problem is reported with its line number, several at once, and an invalid scenario is never run
(exit code 1): count/record mismatch in either direction, missing `END` / section / `max_steps`, content after `END`,
unknown keywords, types, categories, capabilities or statuses, non-integer or out-of-range numbers, wrong field count,
duplicate ids, connections to unknown locations or from a location to itself, cost < 1, robots/requests at unknown
locations, `initial_battery > max_battery`, urgency outside 1..5, duration < 1, events naming unknown
connections/requests/robots, `STATION_*` events on a non-`CHARGING` location, over-long lines or names. Warnings (the
scenario still runs): a road network that is disconnected even with all roads open, no restricted connection and no
`BLOCK` event, no robots / requests / events.

## 7. Output

**Simulation record** (`--out`, CSV with header): `time,event,robot,request,object,value,detail` — ids are the external
ids from the input file; an empty cell means "not applicable". Identical input always gives a byte-identical record.

| `event` | robot / request / object | `value` | `detail` |
|---|---|---|---|
| `SIM_START` | | `max_steps` | scenario name and sizes |
| `REQUEST_REPORTED` | request / location id | urgency | category, duration |
| `REQUEST_STATE` | request | current urgency | `FROM->TO reason` (NOT_REPORTED, PENDING, ASSIGNED, IN_PROGRESS, COMPLETED, FAILED) |
| `ROBOT_STATE` | robot | battery | `FROM->TO reason` (AVAILABLE, TRAVELLING, PERFORMING, RETURNING, RECHARGING, UNAVAILABLE) |
| `CONFLICT` | chosen robot / request | number of candidate robots | |
| `ASSIGN` | robot / request / location id | route cost | route length in edges, battery |
| `MOVE` | robot / request / connection id | battery before the move | `from->to cost=` |
| `REROUTE` | robot / request / blocked connection id (or charger id for a return trip) | new route cost | |
| `ARRIVE`, `WORK_START`, `WORK_DONE` | robot / request | battery / start step / turnaround (report → done) | |
| `REASSIGN` | robot / request | times this request was released | reason (robot offline, route lost, battery) |
| `REQUEST_FAILED` | request | | reason |
| `RETURN_TO_CHARGER` | robot / charger id | route cost | |
| `ENV_BLOCK`, `ENV_UNBLOCK` | connection id | robots whose route used it (BLOCK) | endpoints, changed / no-op |
| `STATION_DOWN`, `STATION_UP` | location id | 0 / 1 | |
| `URGENCY_CHANGE` | request | new urgency | or "ignored: request already …" |
| `ROBOT_OFFLINE`, `ROBOT_ONLINE` | robot | 1 changed / 0 no-op | |
| `REJECTED` | robot or request | battery | an illegal transition or an action refused for lack of battery |
| `SIM_END` | | steps simulated | termination reason |
| `FINAL_STAT` | | the number | its name (`completed`, `failed`, `pending`, `assignments`, `reroutes`, `conflicts`, `total_distance`, `battery_*`, `max_response_time`, `route_searches`, …) |
| `FINAL_ROBOT` | robot | tasks completed | final state, battery, busy steps, distance, battery used / recharged |

**Final statistics** (stdout, `--stats-out`): requests completed / failed / remaining, average and maximum response
time (report → start of work), response time per urgency level, robot utilization, distance, battery used and
recharged, assignments / reassignments / reroutes, conflicts, environment changes, rejected actions, route-search work
counters, execution time, and a per-robot table. Execution time is deliberately **not** part of the record.

`output/` already contains the record and statistics of every scenario, e.g.
`output/scenario_provided_record.csv` and `output/scenario_provided_stats.txt` (the provided scenario).

## 8. Simulation rules (summary — see the report for the reasoning)

* **Time.** One step = one unit of simulated time. A robot moves up to `speed` cost units and does one unit of rescue
  work per step. Steps `t = 0 … max_steps − 1`.
* **Order inside every step (fixed and consistent):** (1) events with `time ≤ t` in file order → (2) requests with
  `report_time ≤ t` become PENDING in file order → (3) dispatch → (4) every robot advances one step in ascending robot
  id → (5) termination check.
* **Environment.** Undirected connections with integer cost; the route criterion is **minimum total cost** (Dijkstra
  over OPEN connections). A BLOCKED connection cannot be entered until an `UNBLOCK` event.
* **Dispatch.** Pending requests are taken in the order *effective urgency ↓, report time ↑, request id ↑*
  (effective urgency = urgency, plus waited steps / `aging_steps` when aging is on). A request is given to an idle
  robot that is capable (`GENERAL` serves any category), can reach it over open roads and has enough battery for
  *route + work + the trip from the request to the nearest UP charging station*. Among candidates: **cheapest route →
  specialist before GENERAL → lowest robot id**. A request nobody can take stays PENDING (for example while its
  location is cut off) and is retried every step. A request whose category no robot can ever serve FAILS at once.
* **Lifecycle.** `NOT_REPORTED → PENDING → ASSIGNED → IN_PROGRESS → COMPLETED`; `ASSIGNED/IN_PROGRESS → PENDING` when
  released; `FAILED` when no robot can ever serve it or after more than `max_reassign` releases. A request has at most
  one robot, and only `IN_PROGRESS → COMPLETED` completes it, so double completion is impossible. Illegal transitions
  are rejected, counted and logged (`REJECTED`).
* **Road blocked while travelling.** A robot already on a road finishes it; at the next node it checks its next
  connection. If that is blocked it re-plans; with no alternative route, or not enough battery for the new route, the
  request is released to PENDING (reassignment) and the robot becomes AVAILABLE.
* **Robot offline.** Its request is released immediately; interrupted work restarts from zero for the next robot. A robot
  that is mid-road finishes that road after coming back online.
* **Station down.** Robots recharging there stop; robots heading there re-plan to the nearest UP station.
* **Resources.** Moving costs `battery_per_cost_unit` per cost unit, work costs `battery_per_work_step` per step; an UP
  station gives `recharge_per_step` per step up to the maximum. An idle robot goes to the nearest UP station when below
  `low_battery_pct` or when a pending request is infeasible only because of its battery.
* **Termination.** `max_steps` reached, or (if `stop_when_done`) every request COMPLETED or FAILED.
* **Metrics.** Response time = step of the first work unit − report time. Utilization = robot-steps TRAVELLING or
  PERFORMING / (robots × steps).

## 9. Performance notes

`make bench` generates scenarios from 50 to 3,200 locations (and up to 3,200 requests / 640 robots) and writes
`output/benchmark.csv` (median of several runs, plus deterministic work counters). On the machine used for the report
the 3,200-location / 3,200-request / 640-robot scenario takes about 2.2 s for 500 steps; the provided scenario takes about
1 ms. Route searches use one of two strategies per dispatch pass, chosen deterministically only for speed: one search per
idle robot (cached while it stays put and roads do not change) when more requests wait than robots are idle, otherwise one
early-exit search per request. When several routes have exactly the same minimum cost the two strategies may pick
different ones; a given strategy always picks the same one. Details and charts are in `docs/report.pdf`.

## 10. Verification (`make test`)

* unit tests of every data structure and the state machines, **Dijkstra vs. Floyd–Warshall** on random graphs with
  blocked edges, early-exit search vs. full search, route-strategy agreement, aging, loader facts for the provided file;
* the same tests and **every scenario under AddressSanitizer + UBSan**, with the invariant checker run after every step;
* every malformed input in `tests/invalid/` must be rejected without a crash;
* **golden regression**: the provided scenario's record and statistics must equal `tests/golden/`;
* determinism (byte-identical records on repeated runs) and CSV well-formedness / event coverage;
* **mutation fuzzing** of scenario files under the sanitizers (corrupted counts, ids, numbers, truncation, long lines …);
* source scan proving no STL containers or `std::string` are used.

## 11. Known limitations

Greedy (non-optimal) matching of requests to robots; no pre-emption of robots that are returning to charge; interrupted
work restarts from zero; each assignment needs a shortest-path search, so total work grows roughly quadratically when the
whole instance is scaled. See the report (Limitations) for the discussion.
