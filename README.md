# AI Rescue Robots – how to run it (Windows)

This is our disaster-response simulator, written in C++. Everything below is typed into PowerShell, nothing else needs to be installed except a C++ compiler.

## Before you start

You need `g++` (MinGW or MSYS2). To check, type:

```
g++ --version
```

If it prints a version, you're good. If it says "not recognized", install MSYS2 from msys2.org, add its `ucrt64\bin` folder to your PATH, and open a fresh PowerShell.

Then open PowerShell **inside this project folder** (the one that contains `src`, `data` and `tests`). Easy way: open the folder in File Explorer, click the address bar, type `powershell` and press Enter. Every command below is meant to be run from there.

## 1. Compile

Copy this whole line, paste it, press Enter:

```
g++ -std=c++17 -O2 src/types.cpp src/environment.cpp src/robot.cpp src/rescue_request.cpp src/logger.cpp src/scenario.cpp src/simulation.cpp src/main.cpp -o rescue_sim.exe
```

If nothing is printed and you get a new prompt, it worked. You now have `rescue_sim.exe`.

## 2. Run the scenario we were given

```
.\rescue_sim.exe data/scenario_provided.txt
```

You should see the final statistics. The important lines are `completed : 60`, `failed : 0` and `simulated 204 steps`.

## 3. Save the record and the statistics to files

```
.\rescue_sim.exe data/scenario_provided.txt --out output/my_record.csv --stats-out output/my_stats.txt
```

Open `my_record.csv` in Excel to see every event, step by step. (The results we already saved for you are in `output/scenario_provided_record.csv` and `output/scenario_provided_stats.txt`.)

## 4. Run the other scenarios

```
.\rescue_sim.exe data/scenario_level1_easy.txt
.\rescue_sim.exe data/scenario_level3_hard.txt
.\rescue_sim.exe data/scenario_stress.txt
.\rescue_sim.exe data/scenario_tiny.txt
.\rescue_sim.exe data/scenario_reroute.txt
.\rescue_sim.exe data/scenario_conflict.txt
.\rescue_sim.exe data/scenario_battery.txt
.\rescue_sim.exe data/scenario_aging.txt
.\rescue_sim.exe data/scenario_medium.txt
```

In short: `level1_easy`, `scenario_provided` and `level3_hard` get harder one after the other, and `stress` is the big one (120 locations, 24 robots, 120 requests, 600 steps). `level3_hard` is overloaded on purpose, so some requests are still waiting at the end. `tiny`, `reroute`, `conflict`, `battery` and `aging` are small ones that each show one feature (a road blocked while a robot is driving, robots competing for requests, a charger going down, and so on).

## 5. Watch it happen

Run only the first 60 steps and print everything at that moment:

```
.\rescue_sim.exe data/scenario_provided.txt --steps 60 --show all
```

Go through it step by step yourself:

```
.\rescue_sim.exe data/scenario_provided.txt --interactive
```

Then type `step`, `step 10`, `robots`, `requests`, `env`, `stats`, `run`, `help` or `quit`.

Run it with an internal self-check after every step (it prints an error and stops if anything is inconsistent):

```
.\rescue_sim.exe data/scenario_stress.txt --check
```

Only load and check a file without running it:

```
.\rescue_sim.exe data/scenario_provided.txt --validate
```

Add `--quiet` to any of these if you don't want the statistics printed.

## 6. Run the tests

```
g++ -std=c++17 -O2 -Isrc tests/test_main.cpp src/types.cpp src/environment.cpp src/robot.cpp src/rescue_request.cpp src/logger.cpp src/scenario.cpp src/simulation.cpp -o test_runner.exe
.\test_runner.exe
```

The last line should say `20339 checks, 0 failed`. The tests create two temporary files, `output/test_a.csv` and `output/test_b.csv`, which you can delete afterwards.

## 7. See what happens with a broken input file

```
.\rescue_sim.exe tests/invalid/count_too_many.txt
$LASTEXITCODE
```

It prints an error with the line number, refuses to run, and `$LASTEXITCODE` shows `1`. (The other broken files are `count_too_few.txt`, `missing_end.txt` and `bad_status.txt` in the same folder.)

## If something goes wrong

- *"g++ is not recognized"*: the compiler isn't installed or isn't on your PATH. See "Before you start", then open a new PowerShell.
- *"No such file" or "cannot open"*: you're probably not in the project folder. Run `dir` – you should see `src`, `data` and `tests`.
- *Compile errors*: check `g++ --version`, we need version 7 or newer.

## What the input file looks like

It's a plain text file, exactly like `scenario_provided.txt`. Lines starting with `#` and blank lines are ignored. It starts with `SCENARIO <name>` and some `PARAM <name> <number>` lines, and then has five sections, each written as a word and a count followed by exactly that many lines:

```
LOCATIONS    id name type                           type: JUNCTION BUILDING HOSPITAL DEPOT CHARGING COMMAND
CONNECTIONS  id location_a location_b cost status   status: OPEN or BLOCKED (roads work both ways)
ROBOTS       id start_location capability max_battery initial_battery   capability: MEDICAL FIRE SUPPLY SEARCH GENERAL
REQUESTS     id report_time location category urgency duration_steps    category: MEDICAL FIRE SUPPLY SEARCH, urgency 1 to 5
EVENTS       time BLOCK|UNBLOCK connection_id
             time STATION_DOWN|STATION_UP location_id
             time SET_URGENCY request_id new_urgency
             time ROBOT_OFFLINE|ROBOT_ONLINE robot_id
END
```

The file must end with `END`. If a count doesn't match the number of lines, or anything else is wrong (unknown ids, bad numbers, and so on), the program reports every problem with its line number and does not run.

Parameters: `max_steps` is required. `speed` (1), `battery_per_cost_unit` (1), `battery_per_work_step` (2) and `recharge_per_step` (5) are optional, with the defaults in brackets. We added a few optional ones of our own: `stop_when_done` (1 = stop as soon as every request is finished), `max_reassign` (5), `low_battery_pct` (50), `aging_steps` (0 = off; waiting requests slowly gain urgency) and `route_strategy` (0 = automatic, only for experiments).

## What the output looks like

**The record** (`--out`) is a CSV with the header `time,event,robot,request,object,value,detail`. An empty cell means "not applicable". Running the same file twice gives exactly the same record. The event types are:

| event | what it means |
|---|---|
| `SIM_START`, `SIM_END` | the run begins / ends (with the reason) |
| `REQUEST_REPORTED` | a request shows up |
| `REQUEST_STATE` | a request changes state (pending, assigned, in progress, completed, failed) |
| `ROBOT_STATE` | a robot changes state (available, travelling, performing, returning, recharging, unavailable) |
| `ASSIGN` | a robot is given a request (value = route cost) |
| `CONFLICT` | several robots could take the same request |
| `MOVE`, `ARRIVE` | a robot moves along a road / arrives at a location |
| `WORK_START`, `WORK_DONE` | rescue work starts / finishes |
| `REROUTE` | a blocked road forced a new route |
| `REASSIGN` | a request was taken away from a robot (offline, route lost, battery) |
| `REQUEST_FAILED` | a request failed, with the reason |
| `RETURN_TO_CHARGER` | a robot goes to a charging station |
| `ENV_BLOCK`, `ENV_UNBLOCK` | a road closes / reopens |
| `STATION_DOWN`, `STATION_UP` | a charging station goes out / comes back |
| `URGENCY_CHANGE` | a request's urgency changed |
| `ROBOT_OFFLINE`, `ROBOT_ONLINE` | a robot goes offline / comes back |
| `REJECTED` | an illegal action was refused (this stays at 0 in all our runs) |
| `FINAL_STAT`, `FINAL_ROBOT` | the final numbers, one line each, at the end of the record |

**The statistics** (printed on screen, and in the file given to `--stats-out`) show how many requests were completed, failed and left over, the average and maximum response time (also per urgency level), robot utilization, distance travelled, battery used and recharged, the number of assignments, reassignments, reroutes and conflicts, how many environment changes were applied, rejected actions, how much work the route searches did, the execution time, and a table for every robot.

For how and why it works the way it does, read `report.pdf`.
