#!/usr/bin/env bash
# Performance benchmark: generates scenarios of many sizes, runs each REPS times
# and writes output/benchmark.csv (median wall-clock times + deterministic work
# counters). Run from the project root after `make`:  ./tools/benchmark.sh
# Usage: ./tools/benchmark.sh [reps]   (default 5)
set -eu
cd "$(dirname "$0")/.."
REPS=${1:-5}
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
OUT=output/benchmark.csv
echo "experiment,locations,connections,robots,requests,steps,completed,records,searches,nodes_settled,edges_scanned,sim_ms,load_ms,sim_ms_nolog" > "$OUT"

median() { sort -n | awk '{a[NR]=$1} END{print a[int((NR+1)/2)]}'; }
field()  { grep "$2" "$1" | head -1 | sed -E "s/.*: *$3.*/\1/"; }

run_case() {  # experiment L C R Q T seed
  local exp=$1 L=$2 C=$3 R=$4 Q=$5 T=$6 seed=$7
  local f="$TMP/s.txt"
  ./gen_scenario "bench_${exp}" "$L" "$C" "$R" "$Q" "$T" "$seed" > "$f"
  local sims="" loads="" nolog=""
  local i
  for ((i = 0; i < REPS; i++)); do
    ./rescue_sim "$f" --quiet --out "$TMP/rec.csv" --stats-out "$TMP/st.txt"
    sims="$sims $(grep 'Execution time' "$TMP/st.txt" | sed -E 's/.*: *([0-9.]+) s/\1/')"
    loads="$loads $(grep 'Load + validate' "$TMP/st.txt" | sed -E 's/.*: *([0-9.]+) s/\1/')"
    ./rescue_sim "$f" --quiet --stats-out "$TMP/st2.txt"
    nolog="$nolog $(grep 'Execution time' "$TMP/st2.txt" | sed -E 's/.*: *([0-9.]+) s/\1/')"
  done
  local sim_ms load_ms nolog_ms
  sim_ms=$(for v in $sims; do awk -v v="$v" 'BEGIN{printf "%.3f\n", v*1000}'; done | median)
  load_ms=$(for v in $loads; do awk -v v="$v" 'BEGIN{printf "%.3f\n", v*1000}'; done | median)
  nolog_ms=$(for v in $nolog; do awk -v v="$v" 'BEGIN{printf "%.3f\n", v*1000}'; done | median)
  local completed searches nodes edges records
  completed=$(grep -E '^  completed' "$TMP/st.txt" | sed -E 's/.*: *([0-9]+).*/\1/')
  searches=$(grep 'Route searches' "$TMP/st.txt" | sed -E 's/.*: *([0-9]+) \(([0-9]+) nodes settled, ([0-9]+) edges.*/\1/')
  nodes=$(grep 'Route searches' "$TMP/st.txt" | sed -E 's/.*: *([0-9]+) \(([0-9]+) nodes settled, ([0-9]+) edges.*/\2/')
  edges=$(grep 'Route searches' "$TMP/st.txt" | sed -E 's/.*: *([0-9]+) \(([0-9]+) nodes settled, ([0-9]+) edges.*/\3/')
  records=$(($(wc -l < "$TMP/rec.csv") - 1))
  echo "$exp,$L,$C,$R,$Q,$T,$completed,$records,$searches,$nodes,$edges,$sim_ms,$load_ms,$nolog_ms" | tee -a "$OUT"
}

echo "# A: everything scaled together (x2 per step), 500 steps"
for k in 1 2 4 8 16 32 64; do
  run_case A $((50*k)) $((110*k)) $((10*k)) $((50*k)) 500 7
done
echo "# B: only locations/connections grow (10 robots, 100 requests, 500 steps)"
for k in 1 2 4 8 16 32 64; do
  run_case B $((50*k)) $((110*k)) 10 100 500 7
done
echo "# C: only requests grow (200 locations, 20 robots, 500 steps)"
for k in 1 2 4 8 16 32; do
  run_case C 200 440 20 $((50*k)) 500 7
done
echo "# D: only robots grow (400 locations, 200 requests, 500 steps)"
for k in 1 2 4 8 16 32; do
  run_case D 400 900 $((5*k)) 200 500 7
done
echo "# E: steps grow with the workload (200 locations, 20 robots; requests = steps/5)"
for t in 250 500 1000 2000 4000 8000; do
  run_case E 200 440 20 $((t/5)) $t 7
done
echo "wrote $OUT"
