#!/usr/bin/env python3
"""
grade.py — score a student submission for the Shortest-Path competition.

For each instance:
  1. Run the unmodified foundation -> T_base, expected output.
  2. Run the student's solver     -> T_student, student output.
  3. Diff outputs. Any mismatch  -> instance score = 0.
  4. Else compute speedup = T_base / T_student, clip to [0.1, 100].

Aggregate scores:
  - Per-category geometric mean of speedups.
  - Two tracks (algorithm: ROAD/GRID/CLUSTER; engineering: SOCIAL).
  - Overall: geometric mean across all instances.

Usage:
  python3 grade.py --foundation ./foundation --solver ./solver \\
                   --instances instances.txt
where instances.txt has one line per instance:
   <category> <graph_path> <query_path>

Each line's category must be one of: ROAD, GRID, CLUSTER, SOCIAL.
"""

import argparse
import math
import subprocess
import sys
import tempfile
import time
from collections import defaultdict
from pathlib import Path

ALGO_CATEGORIES = {"ROAD", "GRID", "CLUSTER"}
ENGI_CATEGORIES = {"SOCIAL"}
ALL_CATEGORIES = ALGO_CATEGORIES | ENGI_CATEGORIES

REPEATS = 3            # take best-of-N to reduce noise
LARGE_ONLY = True      # run only *_large.graph entries from instances.txt
CLIP_LO, CLIP_HI = 0.1, 100.0


def time_run(binary: Path, graph: Path, queries: Path, out: Path) -> float:
    """Run binary, return wall-clock seconds. Exits on non-zero status."""
    t0 = time.perf_counter()
    r = subprocess.run(
        [str(binary), str(graph), str(queries), str(out)],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
    )
    t1 = time.perf_counter()
    if r.returncode != 0:
        sys.stderr.write(f"[fail] {binary} exit={r.returncode}\n{r.stderr.decode()}\n")
        sys.exit(2)
    return t1 - t0


def best_of(binary: Path, graph: Path, queries: Path, out: Path, n: int) -> float:
    return min(time_run(binary, graph, queries, out) for _ in range(n))


def files_equal(a: Path, b: Path) -> bool:
    with a.open() as fa, b.open() as fb:
        return [ln.strip() for ln in fa] == [ln.strip() for ln in fb]


def geomean(xs):
    xs = [x for x in xs if x > 0]
    if not xs:
        return 0.0
    return math.exp(sum(math.log(x) for x in xs) / len(xs))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--foundation", required=True, type=Path)
    ap.add_argument("--solver",     required=True, type=Path)
    ap.add_argument("--instances",  required=True, type=Path)
    args = ap.parse_args()

    # Resolve binaries so subprocess doesn't try PATH-lookup on "foundation".
    args.foundation = args.foundation.resolve()
    args.solver     = args.solver.resolve()

    instances = []
    for line in args.instances.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 3 or parts[0] not in ALL_CATEGORIES:
            sys.exit(f"bad instances line: {line}")
        instances.append((parts[0], Path(parts[1]), Path(parts[2])))

    if LARGE_ONLY:
        instances = [
            (cat, gpath, qpath)
            for cat, gpath, qpath in instances
            if gpath.stem.endswith("_large")
        ]
        if not instances:
            sys.exit("no *_large.graph instances found")

    speedups_by_cat = defaultdict(list)
    rows = []

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        for i, (cat, gpath, qpath) in enumerate(instances):
            ref_out = td / f"{i}.ref"
            stu_out = td / f"{i}.stu"

            t_base = best_of(args.foundation, gpath, qpath, ref_out, REPEATS)
            t_stu  = best_of(args.solver,     gpath, qpath, stu_out, REPEATS)

            if not files_equal(ref_out, stu_out):
                speedup = 0.0
                note = "WRONG"
            else:
                raw = t_base / max(t_stu, 1e-9)
                speedup = max(CLIP_LO, min(CLIP_HI, raw))
                note = "ok"
                speedups_by_cat[cat].append(speedup)

            rows.append((cat, gpath.name, t_base, t_stu, speedup, note))

    print(f"{'category':<8} {'instance':<24} {'T_base':>10} {'T_solver':>10} "
          f"{'speedup':>9}  note")
    print("-" * 78)
    for cat, name, tb, ts, sp, note in rows:
        print(f"{cat:<8} {name:<24} {tb:>10.4f} {ts:>10.4f} {sp:>9.3f}  {note}")

    print()
    for cat in sorted(ALL_CATEGORIES):
        if speedups_by_cat[cat]:
            print(f"{cat:<8} geomean = {geomean(speedups_by_cat[cat]):.3f}  "
                  f"(n={len(speedups_by_cat[cat])})")

    algo = sum((speedups_by_cat[c] for c in ALGO_CATEGORIES), [])
    engi = sum((speedups_by_cat[c] for c in ENGI_CATEGORIES), [])
    overall = algo + engi

    print()
    print(f"Algorithm track   geomean = {geomean(algo):.3f}  (n={len(algo)})")
    print(f"Engineering track geomean = {geomean(engi):.3f}  (n={len(engi)})")
    print(f"Overall           geomean = {geomean(overall):.3f}  (n={len(overall)})")


if __name__ == "__main__":
    main()
