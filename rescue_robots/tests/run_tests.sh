#!/usr/bin/env bash
# Full verification (run from the project root: `make test`, which builds
# everything including the sanitizer binaries first):
#   unit tests (normal + AddressSanitizer/UBSan), every scenario with invariant
#   checks, invalid inputs rejected, golden-record regression, determinism,
#   record well-formedness, mutation fuzzing, and an STL-free source scan.
set -u
cd "$(dirname "$0")/.."
fail=0
pass() { echo "  PASS  $1"; }
bad()  { echo "  FAIL  $1"; fail=1; }
mkdir -p output
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "[1] unit + regression tests (optimised build and sanitizer build)"
if ./test_runner | tail -1; ./test_runner >/dev/null; then pass "test_runner"; else bad "test_runner"; fi
if ./build/test_runner_asan >/dev/null 2>"$TMP/asan.txt"; then pass "test_runner under ASan+UBSan"; else bad "test_runner under ASan+UBSan"; cat "$TMP/asan.txt" | head -20; fi

echo "[2] every working scenario, invariants checked after each step (normal and sanitizer builds)"
for f in data/scenario_*.txt; do
  b=$(basename "$f" .txt)
  if ./rescue_sim "$f" --check --quiet --out "output/${b}_record.csv" --stats-out "output/${b}_stats.txt" &&
     ./build/rescue_sim_asan "$f" --check --quiet >/dev/null 2>"$TMP/e.txt"; then pass "$f"; else bad "$f"; head -5 "$TMP/e.txt"; fi
done

echo "[3] invalid scenarios must be rejected (exit code 1, no sanitizer report)"
for f in tests/invalid/*.txt; do
  [ "$(basename "$f")" = "expected.txt" ] && continue
  ./build/rescue_sim_asan "$f" --validate --quiet >/dev/null 2>"$TMP/e.txt"
  code=$?
  if [ $code -eq 1 ] && ! grep -q -E "Sanitizer|runtime error" "$TMP/e.txt"; then pass "$f"; else bad "$f (exit $code)"; fi
done

echo "[4] golden regression: provided scenario record and statistics must not change"
./rescue_sim data/scenario_provided.txt --quiet --out "$TMP/prov.csv" --stats-out "$TMP/prov_stats.txt"
if cmp -s "$TMP/prov.csv" tests/golden/provided_record.csv; then pass "record identical to tests/golden"; else bad "record differs from tests/golden/provided_record.csv"; fi
grep -v -E "Execution|Load" "$TMP/prov_stats.txt" > "$TMP/prov_stats_clean.txt"
if cmp -s "$TMP/prov_stats_clean.txt" tests/golden/provided_stats.txt; then pass "statistics identical to tests/golden"; else bad "statistics differ from tests/golden/provided_stats.txt"; fi

echo "[5] determinism (two runs, byte-identical records)"
for s in provided stress level3_hard scale_1000; do
  ./rescue_sim data/scenario_$s.txt --quiet --out "$TMP/a.csv"
  ./rescue_sim data/scenario_$s.txt --quiet --out "$TMP/b.csv"
  if cmp -s "$TMP/a.csv" "$TMP/b.csv"; then pass "$s"; else bad "$s"; fi
done

echo "[6] record well-formedness (7 columns everywhere; every event type present in provided run)"
if awk -F, 'NF!=7 {exit 1}' tests/golden/provided_record.csv; then pass "7 columns on every line"; else bad "malformed CSV line"; fi
for ev in SIM_START REQUEST_REPORTED REQUEST_STATE ROBOT_STATE ASSIGN CONFLICT MOVE ARRIVE WORK_START WORK_DONE \
          ENV_BLOCK ENV_UNBLOCK STATION_DOWN STATION_UP URGENCY_CHANGE ROBOT_OFFLINE ROBOT_ONLINE REASSIGN \
          RETURN_TO_CHARGER SIM_END FINAL_STAT FINAL_ROBOT; do
  if grep -q ",$ev," tests/golden/provided_record.csv; then :; else bad "event $ev missing from provided record"; fi
done
echo "  (event-type coverage checked)"

echo "[7] mutation fuzzing under ASan+UBSan (random corruptions of scenario files)"
for spec in "provided 11" "battery 12" "level3_hard 13" "reroute 14"; do
  set -- $spec
  if ./build/fuzz_loader_asan data/scenario_$1.txt 2500 $2 "$TMP/fz.txt" > "$TMP/fz.out" 2>&1; then pass "$1: $(cat "$TMP/fz.out")"; else bad "fuzz $1"; tail -5 "$TMP/fz.out"; fi
done

echo "[8] no STL containers / std::string in src, tools, tests"
if grep -nE '#include *<(vector|map|set|unordered_map|unordered_set|list|deque|queue|stack|array|string|algorithm|priority_queue|forward_list|bitset)>' \
     src/*.cpp src/*.h tools/*.cpp tests/*.cpp; then bad "STL include found"; else pass "no STL includes"; fi
if grep -nE 'std::(vector|map|set|string|list|deque|queue|stack|array|sort|priority_queue|unordered)' \
     src/*.cpp src/*.h tools/*.cpp tests/*.cpp; then bad "STL use found"; else pass "no std:: containers"; fi

echo
if [ $fail -eq 0 ]; then echo "ALL TESTS PASSED"; else echo "SOME TESTS FAILED"; fi
exit $fail
