#!/usr/bin/env python3
"""
Cached local grader for development.

This does not replace the official grade.py.  It keeps the foundation binary
unmodified, but caches each foundation output/time so repeated local solver
experiments do not rerun the very slow large baseline.

Example:
  python3 scripts/grade_cached.py --foundation ./foundation --solver ./solver \
      --instances instances.txt --base-repeats 1 --solver-repeats 3 --progress
"""

import argparse
import hashlib
import math
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path


ALL_CATEGORIES = {"ROAD", "GRID", "CLUSTER", "SOCIAL"}
ALGO_CATEGORIES = {"ROAD", "GRID", "CLUSTER"}
ENGI_CATEGORIES = {"SOCIAL"}
CLIP_LO, CLIP_HI = 0.1, 100.0


def geomean(xs):
    xs = [x for x in xs if x > 0]
    return math.exp(sum(math.log(x) for x in xs) / len(xs)) if xs else 0.0


def file_fingerprint(path):
    path = Path(path)
    st = path.stat()
    h = hashlib.sha1()
    h.update(str(path.resolve()).encode())
    h.update(str(st.st_size).encode())
    h.update(str(int(st.st_mtime_ns)).encode())
    return h.hexdigest()[:16]


def cache_key(cat, graph, queries):
    return f"{cat}_{Path(graph).stem}_{file_fingerprint(graph)}_{file_fingerprint(queries)}"


def time_run(binary, graph, queries, out):
    t0 = time.perf_counter()
    r = subprocess.run(
        [str(binary), str(graph), str(queries), str(out)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    t1 = time.perf_counter()
    if r.returncode != 0:
        sys.stderr.write(f"[fail] {binary} exit={r.returncode}\n{r.stderr.decode()}\n")
        sys.exit(2)
    return t1 - t0


def best_of(binary, graph, queries, out, repeats):
    return min(time_run(binary, graph, queries, out) for _ in range(repeats))


def files_equal(a, b):
    with Path(a).open() as fa, Path(b).open() as fb:
        return [ln.strip() for ln in fa] == [ln.strip() for ln in fb]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--foundation", required=True, type=Path)
    ap.add_argument("--solver", required=True, type=Path)
    ap.add_argument("--instances", required=True, type=Path)
    ap.add_argument("--cache-dir", type=Path, default=Path(".grade_cache"))
    ap.add_argument("--base-repeats", type=int, default=1)
    ap.add_argument("--solver-repeats", type=int, default=3)
    ap.add_argument("--progress", action="store_true")
    args = ap.parse_args()

    args.foundation = args.foundation.resolve()
    args.solver = args.solver.resolve()
    args.cache_dir.mkdir(parents=True, exist_ok=True)

    instances = []
    for line in args.instances.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 3 or parts[0] not in ALL_CATEGORIES:
            sys.exit(f"bad instances line: {line}")
        instances.append((parts[0], Path(parts[1]), Path(parts[2])))

    rows = []
    speedups_by_cat = defaultdict(list)

    for i, (cat, graph, queries) in enumerate(instances):
        key = cache_key(cat, graph, queries)
        ref_out = args.cache_dir / f"{key}.ref"
        base_time_file = args.cache_dir / f"{key}.base_time"
        stu_out = args.cache_dir / f"{key}.solver"

        if args.progress:
            print(f"[{i+1}/{len(instances)}] {cat} {graph.name} ...", flush=True)

        if ref_out.exists() and base_time_file.exists():
            t_base = float(base_time_file.read_text().strip())
            base_note = "cached"
        else:
            t_base = best_of(args.foundation, graph, queries, ref_out, args.base_repeats)
            base_time_file.write_text(f"{t_base:.9f}\n")
            base_note = f"base{args.base_repeats}"

        t_solver = best_of(args.solver, graph, queries, stu_out, args.solver_repeats)
        if files_equal(ref_out, stu_out):
            raw = t_base / max(t_solver, 1e-9)
            speedup = max(CLIP_LO, min(CLIP_HI, raw))
            note = "ok"
            speedups_by_cat[cat].append(speedup)
        else:
            speedup = 0.0
            note = "WRONG"

        rows.append((cat, graph.name, t_base, t_solver, speedup, note, base_note))

    print(f"{'category':<8} {'instance':<24} {'T_base':>10} {'T_solver':>10} "
          f"{'speedup':>9}  note   base")
    print("-" * 88)
    for cat, name, tb, ts, sp, note, base_note in rows:
        print(f"{cat:<8} {name:<24} {tb:>10.4f} {ts:>10.4f} {sp:>9.3f}  "
              f"{note:<5}  {base_note}")

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
