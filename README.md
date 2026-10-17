# Shortest-Path Algorithms Competition

## Project overview

**Goal:** Accelerate exact shortest-path queries in C++17 while preserving the reference Dijkstra solver's output.

**Implemented improvements:** cache-friendly CSR adjacency lists, buffered I/O, reusable per-query state, specialized priority queues, and ALT/A* landmark heuristics. The solver also uses graph-family-specific strategies for road, grid, cluster, and social-network instances.

**Reported evaluation:** All 12 released test instances passed correctness checks. The documented **overall geometric-mean speedup is 64.894×** versus the foundation solver, based on the repository's grading procedure and test hardware. These results are specific to the released benchmarks, not a general performance guarantee.

- [Read the optimization report](./optimization_report.md)
- [Inspect the optimized solver](./solver.cpp)
- [View the grading log](./grade_full.txt)

---

## Competition setup and original task instructions

Your job: take the provided Dijkstra implementation and make it **faster** on the
released graph instances, without changing its output.

This README contains everything you need.

## What's in this repo

```
dijkstra_foundation.cpp   the baseline you must build on
Makefile                  builds  `foundation`  and  `solver`
grade.py                  scoring script (correctness + speedup)
scripts/download_large.sh fetch the large-tier instances
samples/
    tiny.graph            6-node hand-built graph
    tiny.queries          7 queries
    tiny.expected         expected output (for sanity)
instances/
    *_small.{graph,queries}    small tier (committed in repo)
    *_medium.{graph,queries}   medium tier (committed in repo)
    *_large.{graph,queries}    large tier — NOT in repo, see below
```

## Large tier — download separately

The large-tier instances (~242 MB total) are distributed via the
project's **GitHub Releases**, not committed to the repo. Fetch them:

```sh
scripts/download_large.sh
```

This drops the 8 large files into `instances/` so every other command
in this README works without modification.

## 1. Build and sanity-check the foundation

```sh
make foundation
./foundation samples/tiny.graph samples/tiny.queries /tmp/out.txt
diff /tmp/out.txt samples/tiny.expected   # must be empty
```

If `diff` prints anything, something is wrong with your environment — stop and ask.

## 2. Measure the baseline on your machine

Run the **unmodified** `foundation` binary on every released instance and record
the wall-clock time. These are your `T_base` values. Submit them in
`baseline_times.txt`.

```sh
time ./foundation public/road_medium.graph public/road_medium.queries /tmp/out.txt
```

## 3. Fork the foundation into `solver.cpp`

```sh
cp dijkstra_foundation.cpp solver.cpp
make solver
```

Now improve `solver.cpp`. Your submission must:

- Keep the same command-line interface: `./solver <graph> <queries> <output>`.
- Read the same file formats as the foundation (see the parsing code in
  `dijkstra_foundation.cpp`).
- Produce **byte-identical** output to the foundation on every instance
  (modulo the trailing newline).
- Be a derivative of the foundation, not a rewrite. Submit a
  `diff_from_foundation.patch`.

## 4. Score yourself locally

Create an `instances.txt` like:

```
ROAD     public/road_small.graph     public/road_small.queries
GRID     public/grid_small.graph     public/grid_small.queries
CLUSTER  public/cluster_small.graph  public/cluster_small.queries
SOCIAL   public/social_small.graph   public/social_small.queries
```

Then:

```sh
python3 grade.py \
    --foundation ./foundation \
    --solver     ./solver \
    --instances  instances.txt
```

You'll see per-instance speedups, per-category geometric means, and the two
track scores. `note=WRONG` means correctness failed — fix that first; speed is
worthless if the answers are wrong.

## Foundation baseline times (reference)

Wall-clock for the **unmodified foundation** on each released instance,
measured on the instructor's machine (Apple Silicon, single-threaded,
`-O2 -std=c++17`). Each instance has **10,000 queries**. Your absolute
times will differ — what matters for grading is the ratio
`T_base / T_solver` on **your** hardware.

| Instance        |       V |        E | Coords |       Time | Distance range          |
|-----------------|--------:|---------:|:------:|-----------:|-------------------------|
| road_small      |  10,000 |   27,154 |  yes   |  **1.88 s** | 4,862 – 1,479,093       |
| grid_small      |   8,992 |   15,990 |  yes   |  **1.91 s** | 0 – 610                 |
| cluster_small   |  10,000 |   29,818 |  yes   |  **2.49 s** | 0 – 2,015,466           |
| social_small    |  10,000 |   29,994 |   no   |  **6.12 s** | 1 – 245                 |
| road_medium     |  99,856 |  273,474 |  yes   | **23.84 s** | 1,510 – 1,398,547       |
| grid_medium     |  89,840 |  161,154 |  yes   | **22.44 s** | 1 – 1,863               |
| cluster_medium  |  99,935 |  299,212 |  yes   | **30.11 s** | 1,615 – 3,634,279       |
| social_medium   | 100,000 |  299,994 |   no   | **70.63 s** | 0 – 269                 |

Notes:

- 100% of queries are reachable on every instance (largest connected
  component equals `V`).
- The foundation is deterministic — re-running it on the same input
  yields byte-identical output (verified by SHA-1 on all small and
  medium instances). Your solver must do the same.

Large-tier (1M-node) baseline times will be published after the full
large-tier run completes.

## What to submit

- `solver.cpp` (+ any headers)
- `Makefile`
- `baseline_times.txt`
- `solver_times.txt`
- `diff_from_foundation.patch`
- `report.pdf` (one page)
- `machine.txt`

## Rules

C++17 only, std library only, single-threaded final binary, 4 GB memory cap,
no precomputed answers in the binary, output must match the foundation exactly,
and the solver must be derived from `dijkstra_foundation.cpp` rather than a
parallel rewrite.
