#!/usr/bin/env python3
"""Draws the benchmark charts from output/benchmark.csv (optional tooling).
Usage: python3 tools/plot_benchmark.py [out_dir]   (needs matplotlib)"""
import csv, sys, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

out = sys.argv[1] if len(sys.argv) > 1 else "docs"
rows = list(csv.DictReader(open("output/benchmark.csv")))
def sel(exp): return [r for r in rows if r["experiment"] == exp]
def col(rs, k): return [float(r[k]) for r in rs]

fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
A = sel("A")
ax[0].loglog(col(A, "locations"), col(A, "sim_ms"), "o-", label="simulation time")
ax[0].loglog(col(A, "locations"), col(A, "nodes_settled"), "s--", label="nodes settled (work count)")
x = col(A, "locations"); ref = [col(A, "sim_ms")[0] * (v / x[0]) ** 2 for v in x]
ax[0].loglog(x, ref, ":", color="gray", label="quadratic reference")
ax[0].set_title("A: all dimensions x2 per step"); ax[0].set_xlabel("locations (robots, requests scale too)")
ax[0].set_ylabel("ms  /  nodes"); ax[0].legend(fontsize=7); ax[0].grid(True, which="both", alpha=.3)

for exp, key, lab in (("B", "locations", "locations only"), ("C", "requests", "requests only"), ("D", "robots", "robots only")):
    s = sel(exp)
    ax[1].loglog(col(s, key), col(s, "sim_ms"), "o-", label=lab)
ax[1].set_title("B/C/D: one dimension at a time"); ax[1].set_xlabel("size of the varied dimension")
ax[1].set_ylabel("simulation time (ms)"); ax[1].legend(fontsize=7); ax[1].grid(True, which="both", alpha=.3)

E = sel("E")
ax[2].plot(col(E, "steps"), col(E, "sim_ms"), "o-", label="simulation time")
ax[2].set_title("E: steps grow with the workload"); ax[2].set_xlabel("time steps (requests = steps/5)")
ax[2].set_ylabel("ms"); ax[2].grid(True, alpha=.3)
plt.tight_layout()
os.makedirs(out, exist_ok=True)
plt.savefig(os.path.join(out, "benchmark.png"), dpi=150)
print("wrote", os.path.join(out, "benchmark.png"))
