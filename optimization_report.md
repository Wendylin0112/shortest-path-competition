# Shortest Path Solver Optimization Report

## Source

The complete implementation is in `solver.cpp`. It keeps the required command
line interface:

```text
./solver <graph_file> <query_file> <output_file>
```

The baseline is the original foundation Dijkstra implementation: it answers each
query by running a fresh shortest-path search with heavier parsing, per-query
state clearing, and a general priority queue. The final solver keeps the same
exact shortest-path semantics, but reduces both engineering overhead and the
number/cost of explored states.

## Summary vs Original Dijkstra

Compared with the original Dijkstra solver, the final solver uses:

- A cache-friendly CSR graph representation instead of repeatedly scanning
  pointer-heavy adjacency structures.
- Whole-file input parsing and buffered output instead of repeated formatted I/O.
- Generation counters to reuse distance/state arrays without clearing `O(V)`
  memory for every query.
- Custom heap and bucket queues that fit the graph's weight profile better than
  a generic priority queue.
- Exact ALT/A* landmarks to guide search toward the target and avoid expanding
  most vertices that cannot improve the answer.
- Graph-family dispatch so road, grid, cluster, and social instances each use
  the fastest exact query path observed for their structure.

All reported runs below returned `ok` on every instance.

## Engineering Optimizations

### CSR adjacency layout

The graph is stored as flat CSR arrays:

- `head[u]..head[u + 1]` gives the edge range for vertex `u`.
- `to[]` and `w[]` are contiguous arrays, so neighbor scans are sequential.
- Neighbors are sorted by destination, giving deterministic access patterns and
  slightly better locality.

This replaces the heavier foundation-style adjacency handling and improves the
hot relaxation loop used by Dijkstra, A*, preprocessing, and fallback paths.

### Fast input and output

The solver reads each graph/query file into memory with `<cstdio>` and parses
integers using a hand-written scanner. Results are written through a 1 MiB output
buffer. This removes the high overhead of `fscanf`/per-line output in the
baseline, which matters because every grading run contains many queries.

### Reusable per-query state

Instead of clearing full `dist[]`/visited arrays for each query, the solver uses
generation counters. Each vertex state stores the generation in which it is
valid. Starting a new query increments the generation and only touched vertices
need to be overwritten. This avoids an `O(V)` memory clear per query.

### Custom priority queues

The solver replaces `std::priority_queue` with compact queues:

- A flat 4-ary/8-ary heap lowers tree height and improves cache behavior.
- Heap entries are packed into one `uint64_t` (`key` in the high bits, vertex id
  in the low bits), cutting heap-entry memory traffic.
- Small-integer-weight graphs use Dial-style cyclic bucket queues for near
  `O(1)` push/pop instead of `O(log n)` heap operations.
- Bitmap-augmented buckets accelerate sparse bucket scans by locating the next
  non-empty bucket with word-level bit operations.

These changes do not alter shortest-path correctness; they only change the data
structure used to process the same monotone keys.

## Algorithmic Optimizations

### Exact ALT/A*

The main algorithmic improvement over original Dijkstra is exact ALT:

```text
h(v, t) = max_k |dist(L_k, v) - dist(L_k, t)|
```

The solver builds `K = 24` farthest-first landmarks and stores exact graph
distances from each landmark to every vertex. By the triangle inequality, this
heuristic is admissible and consistent, so A* still returns the exact shortest
path distance.

In practice, ALT avoids exploring large regions whose lower bound already proves
they cannot improve the best path. This is the main reason the solver reaches
tens of times speedup over the original Dijkstra on large graphs.

### Bidirectional ALT with balanced potentials

For graph families where two-front search is faster, the solver uses a
bidirectional ALT/A* variant with balanced potentials. The forward and backward
searches run Dijkstra on reduced, non-negative keys, and the Pohl stopping rule
terminates when the two frontiers can no longer improve the current best path.

This is especially effective on grid, cluster, and social-style graphs, where
meeting in the middle reduces the amount of explored state.

### Landmark table compression

Grid and social instances have small enough landmark distances to use an `int16`
landmark table when safe. This reduces landmark-row memory bandwidth and cache
pressure in the heuristic kernel. If distances do not fit, the solver keeps the
normal `int32` table.

### Cluster overlay shortcut

Cluster graphs contain dense local regions joined by relatively few expensive
cross edges. The solver builds an exact boundary overlay for cluster-like graphs:

- Local components are formed from edges below a threshold.
- Distances from component boundary vertices to local vertices are precomputed.
- A dense boundary graph is closed with Floyd-Warshall.
- Cross-component queries can often be answered exactly through the overlay.

If the overlay cannot answer a query, the solver falls back to the normal exact
ALT path.

## Graph-Family Dispatch

The solver statically chooses the fastest exact path based on graph properties:

| Graph family | Active path | Reason |
|---|---|---|
| ROAD | Unidirectional ALT with bitmap buckets | Road graphs are high-diameter; target-directed search plus efficient sparse bucket scans works well. |
| GRID | Unidirectional/bidirectional ALT with Dial buckets and `int16` landmarks | Edge weights are tiny, so bucket queues are faster than heaps; compressed landmarks reduce cache traffic. |
| CLUSTER | Bidirectional ALT with 8-ary heap plus exact overlay shortcut | Large weights make plain buckets expensive; balanced bidirectional search and overlay shortcuts reduce work. |
| SOCIAL | Bidirectional ALT with Dial buckets, `int16` landmarks, and landmark upper bound | Small-world structure benefits from two-front search; landmark upper bounds improve early stopping/pruning. |

The dispatch avoids the earlier probe-based choice, so the first queries are not
double-run just to select a mode.

## Correctness

The solver remains exact:

- CSR/I/O/heap/bucket changes are representation and data-structure changes only.
- Dial buckets are used only where key growth is bounded by the graph's maximum
  edge weight or by the reduced-cost bound from the consistent ALT heuristic.
- ALT lower bounds come from exact landmark shortest-path distances.
- Bidirectional ALT uses balanced potentials and an exact stopping condition.
- Cluster overlay answers only cases where the boundary decomposition is exact;
  otherwise it falls back to ALT.

The final full run reported `ok` for all 12 released instances.

## Final Full Run

Command:

```sh
python3 grade.py --foundation ./foundation --solver ./solver --instances instances.txt --progress
```

Result from `grade_full_rerun.txt`:

```text
category instance                     T_base   T_solver   speedup  note
------------------------------------------------------------------------------
ROAD     road_small.graph             3.3806     0.1421    23.794  ok
GRID     grid_small.graph             3.5424     0.0595    59.488  ok
CLUSTER  cluster_small.graph          4.3778     0.1671    26.197  ok
SOCIAL   social_small.graph          12.0911     0.1432    84.428  ok
ROAD     road_medium.graph           58.0842     1.2890    45.061  ok
GRID     grid_medium.graph           46.7139     0.6476    72.129  ok
CLUSTER  cluster_medium.graph        69.5936     0.6537   100.000  ok
SOCIAL   social_medium.graph        208.7952     1.7656   100.000  ok
ROAD     road_large.graph           861.0777    13.3459    64.520  ok
GRID     grid_large.graph           637.4887     7.5035    84.958  ok
CLUSTER  cluster_large.graph       1069.4073     8.7728   100.000  ok
SOCIAL   social_large.graph        3611.7596    15.8306   100.000  ok

CLUSTER  geomean = 63.986  (n=3)
GRID     geomean = 71.436  (n=3)
ROAD     geomean = 41.051  (n=3)
SOCIAL   geomean = 94.514  (n=3)

Algorithm track   geomean = 57.250  (n=9)
Engineering track geomean = 94.514  (n=3)
Overall           geomean = 64.894  (n=12)
```

## Measurement Table

| Category | Instance | T_base | T_solver | Speedup | Note |
|---|---|---:|---:|---:|---|
| ROAD | road_small.graph | 3.3806s | 0.1421s | 23.794x | ok |
| GRID | grid_small.graph | 3.5424s | 0.0595s | 59.488x | ok |
| CLUSTER | cluster_small.graph | 4.3778s | 0.1671s | 26.197x | ok |
| SOCIAL | social_small.graph | 12.0911s | 0.1432s | 84.428x | ok |
| ROAD | road_medium.graph | 58.0842s | 1.2890s | 45.061x | ok |
| GRID | grid_medium.graph | 46.7139s | 0.6476s | 72.129x | ok |
| CLUSTER | cluster_medium.graph | 69.5936s | 0.6537s | 100.000x | ok |
| SOCIAL | social_medium.graph | 208.7952s | 1.7656s | 100.000x | ok |
| ROAD | road_large.graph | 861.0777s | 13.3459s | 64.520x | ok |
| GRID | grid_large.graph | 637.4887s | 7.5035s | 84.958x | ok |
| CLUSTER | cluster_large.graph | 1069.4073s | 8.7728s | 100.000x | ok |
| SOCIAL | social_large.graph | 3611.7596s | 15.8306s | 100.000x | ok |

| Score | Geomean | Count |
|---|---:|---:|
| CLUSTER | 63.986 | 3 |
| GRID | 71.436 | 3 |
| ROAD | 41.051 | 3 |
| SOCIAL | 94.514 | 3 |
| Algorithm track | 57.250 | 9 |
| Engineering track | 94.514 | 3 |
| Overall | 64.894 | 12 |

## Notes on Rejected or Optional Optimizations

- SIMD/prefetch experiments were considered, but the final submission keeps the
  main solver portable ISO C++17/std-library code.
- Query sorting and probe-based dispatch are available behind flags but disabled
  in the final default build because they did not consistently improve the full
  score.
- Contraction-hierarchy experiments remain disabled: preprocessing cost and code
  risk were worse than the active ALT-based path for the released instances.
