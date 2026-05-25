# Shortest Path Solver Optimization Report

## Source

The complete implementation is in `solver.cpp`.  It keeps the required CLI:

```text
./solver <graph_file> <query_file> <output_file>
```

## Main Techniques

- CSR adjacency arrays for contiguous edge scans.
- Whole-file buffered input and buffered output.
- Reusable generation counters for per-query state, avoiding `O(V)` clears.
- Packed 8-ary heap entries to reduce heap traffic.
- Exact ALT/A* with 24 farthest-first landmarks, stored vertex-major as `int32`.
- Grid/social low-weight graphs use Dial bucket queues; grid also converts landmark rows to `int16` when safe.
- Road-like graphs use unidirectional ALT plus bitmap buckets when edge-weight spread is moderate.
- Cluster-like high-weight graphs use bidirectional balanced-potential ALT with the 8-ary heap.
- Social graphs now also use ALT even without coordinates; graph distances alone make the landmark lower bound valid.
- Static dispatch replaces the earlier 30-query probe to avoid double-running warm-up queries.
- SOCIAL also uses a landmark-derived upper bound `min_k d(s,L_k)+d(L_k,t)` inside its int16 bucket ALT path.

All heuristics are admissible/consistent lower bounds, so results remain exact.

## Considered Follow-up Optimizations

- Landmark prefetching and SIMD heuristic kernels were considered, but the final submission keeps ISO C++17/std-library-only code. Compiler intrinsics such as `__builtin_prefetch`, AVX2, or NEON are useful experiments but are not portable under the stated competition rules.
- Symmetric memoization was analyzed. Released large query files have no repeated unordered pairs (`sym_dup=0`), so a hash-table cache would add overhead without improving the large-tier score.
- Native GRID lattice addressing is possible because GRID coordinates are integer lattice points with holes. The current grid path is already the exact int16 ALT + Dial bucket hot path; replacing CSR with a custom sparse lattice representation would add significant code risk for uncertain gain.

## Official Full Run

Command:

```sh
python3 grade.py --foundation ./foundation --solver ./solver --instances instances.txt --progress
```

Result:

```text
category instance                     T_base   T_solver   speedup  note
------------------------------------------------------------------------------
ROAD     road_small.graph             4.3011     0.1951    22.042  ok
GRID     grid_small.graph             4.4041     0.0952    46.256  ok
CLUSTER  cluster_small.graph          4.9962     0.3745    13.341  ok
SOCIAL   social_small.graph          13.2219     0.1994    66.296  ok
ROAD     road_medium.graph           62.7959     1.4695    42.733  ok
GRID     grid_medium.graph           47.1362     0.7804    60.401  ok
CLUSTER  cluster_medium.graph        76.3894     1.6086    47.487  ok
SOCIAL   social_medium.graph        244.1336     3.0551    79.910  ok
ROAD     road_large.graph           904.8340    13.7633    65.743  ok
GRID     grid_large.graph           587.3162     9.4342    62.254  ok
CLUSTER  cluster_large.graph       1079.0337    10.6161   100.000  ok
SOCIAL   social_large.graph        3620.3446    21.6503   100.000  ok

CLUSTER  geomean = 39.865  (n=3)
GRID     geomean = 55.821  (n=3)
ROAD     geomean = 39.563  (n=3)
SOCIAL   geomean = 80.915  (n=3)

Algorithm track   geomean = 44.486  (n=9)
Engineering track geomean = 80.915  (n=3)
Overall           geomean = 51.662  (n=12)
```

## Full Measurement Table

These are the times from the official full local grading run:

| Category | Instance | T_base | T_solver | Speedup | Note |
|---|---|---:|---:|---:|---|
| ROAD | road_small.graph | 4.3011s | 0.1951s | 22.042x | ok |
| GRID | grid_small.graph | 4.4041s | 0.0952s | 46.256x | ok |
| CLUSTER | cluster_small.graph | 4.9962s | 0.3745s | 13.341x | ok |
| SOCIAL | social_small.graph | 13.2219s | 0.1994s | 66.296x | ok |
| ROAD | road_medium.graph | 62.7959s | 1.4695s | 42.733x | ok |
| GRID | grid_medium.graph | 47.1362s | 0.7804s | 60.401x | ok |
| CLUSTER | cluster_medium.graph | 76.3894s | 1.6086s | 47.487x | ok |
| SOCIAL | social_medium.graph | 244.1336s | 3.0551s | 79.910x | ok |
| ROAD | road_large.graph | 904.8340s | 13.7633s | 65.743x | ok |
| GRID | grid_large.graph | 587.3162s | 9.4342s | 62.254x | ok |
| CLUSTER | cluster_large.graph | 1079.0337s | 10.6161s | 100.000x | ok |
| SOCIAL | social_large.graph | 3620.3446s | 21.6503s | 100.000x | ok |

| Score | Geomean | Count |
|---|---:|---:|
| CLUSTER | 39.865 | 3 |
| GRID | 55.821 | 3 |
| ROAD | 39.563 | 3 |
| SOCIAL | 80.915 | 3 |
| Algorithm track | 44.486 | 9 |
| Engineering track | 80.915 | 3 |
| Overall | 51.662 | 12 |

For comparison, the fastest reference-style solver measured locally before the social-ALT change took:

| Instance | Before | After | Improvement |
|---|---:|---:|---:|
| road_large | 14.0391s | 13.7633s | 1.02x |
| grid_large | 9.6468s | 9.4342s | 1.02x |
| cluster_large | 11.3167s | 10.6161s | 1.07x |
| social_large | 68.5482s | 21.6503s | 3.17x |

Arithmetic mean improvement over that local reference large run: 1.57x.
Geometric mean improvement over that local reference large run: 1.36x.
