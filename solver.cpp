// solver.cpp
//
// Phase E solver: engineering + bidirectional Dijkstra + ALT (A* with
// Landmarks).  Output byte-identical to dijkstra_foundation.cpp.
//
// Engineering layer (Phase A):
//   * whole-file <cstdio> input + hand-rolled integer parser  (no fscanf)
//   * CSR adjacency (flat int arrays), neighbours sorted by destination
//   * 4-ary min-heap on a flat array (no std::priority_queue)
//   * Generation counter on dist[] -- avoids O(V) memset per query
//   * 1 MiB output buffer, single fwrite at end (no fprintf)
//
// Algorithm dispatch:
//   * All released graphs use exact ALT preprocess (K=24 farthest-first
//     landmarks), including social even though it has no coordinates.
//   * Static dispatch by weight profile:
//       - road-like graphs: unidirectional ALT, bitmap bucket when useful
//       - grid/social small-weight graphs: bidirectional ALT + Dial buckets
//       - cluster-like high-weight graphs: bidirectional ALT + 8-ary heap
//
// Portability: ISO C++17 standard library only -- no POSIX (mmap/open/write)
// or third-party headers.  All I/O goes through <cstdio>, exactly as the
// foundation baseline does.
//
// CLI unchanged:
//     ./solver <graph_file> <query_file> <output_file>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <queue>
#include <utility>
#include <functional>
#include <chrono>

using std::vector;

// ---------------------------------------------------------------------------
// File input: slurp the whole file into a heap buffer with <cstdio>.
//
// Replaces an earlier mmap-based reader.  Competition rules say "std library
// only"; mmap/open/fstat are POSIX, not ISO C++, so we use fopen/fread/fclose
// (the same facilities the foundation baseline uses).  Reading the whole file
// up-front is functionally identical to mmap here -- the parser scans the
// buffer exactly once, sequentially.
// ---------------------------------------------------------------------------

struct FileIn {
    vector<char> storage;
    const char*  data = nullptr;
    size_t       size = 0;

    bool open_file(const char* path) {
        std::FILE* f = std::fopen(path, "rb");
        if (!f) return false;
        if (std::fseek(f, 0, SEEK_END) != 0) { std::fclose(f); return false; }
        long sz = std::ftell(f);
        if (sz < 0) { std::fclose(f); return false; }
        std::rewind(f);
        storage.resize((size_t)sz);
        size_t got = (sz > 0)
            ? std::fread(storage.data(), 1, (size_t)sz, f)
            : 0;
        std::fclose(f);
        if (got != (size_t)sz) return false;
        data = storage.empty() ? "" : storage.data();
        size = (size_t)sz;
        return true;
    }
};

struct Parser {
    const char* p;
    const char* end;
    Parser(const char* d, size_t n) : p(d), end(d + n) {}

    inline void skip_ws() {
        while (p < end) {
            char c = *p;
            if (c == ' ' || c == '\n' || c == '\r' || c == '\t') ++p;
            else break;
        }
    }
    inline int32_t i32() {
        skip_ws();
        bool neg = false;
        if (p < end && (*p == '-' || *p == '+')) { neg = (*p == '-'); ++p; }
        int32_t x = 0;
        while (p < end) {
            char c = *p;
            if (c < '0' || c > '9') break;
            x = x * 10 + (c - '0');
            ++p;
        }
        return neg ? -x : x;
    }
    inline double f64() {
        skip_ws();
        char* endp;
        double v = std::strtod(p, &endp);
        p = endp;
        return v;
    }
};

// ---------------------------------------------------------------------------
// Output buffer: 1 MiB growable, single fwrite on destruction
// ---------------------------------------------------------------------------

struct OutBuf {
    char*      buf = nullptr;
    size_t     cap = 0;
    size_t     pos = 0;
    std::FILE* f   = nullptr;

    bool open_file(const char* path, size_t initial_cap = 1u << 20) {
        f = std::fopen(path, "wb");
        if (!f) return false;
        cap = initial_cap;
        buf = (char*)std::malloc(cap);
        pos = 0;
        return buf != nullptr;
    }
    void grow(size_t need) {
        while (cap - pos < need) cap *= 2;
        buf = (char*)std::realloc(buf, cap);
    }
    inline void put_int64_nl(int64_t v) {
        if (cap - pos < 24) grow(24);
        char* out = buf + pos;
        if (v < 0) {
            *out++ = '-';
            uint64_t uv = (uint64_t)(-v);
            char tmp[20]; int k = 0;
            do { tmp[k++] = '0' + (uv % 10); uv /= 10; } while (uv);
            while (k--) *out++ = tmp[k];
        } else if (v == 0) {
            *out++ = '0';
        } else {
            uint64_t uv = (uint64_t)v;
            char tmp[20]; int k = 0;
            do { tmp[k++] = '0' + (uv % 10); uv /= 10; } while (uv);
            while (k--) *out++ = tmp[k];
        }
        *out++ = '\n';
        pos = (size_t)(out - buf);
    }
    ~OutBuf() {
        if (f) {
            if (pos) std::fwrite(buf, 1, pos, f);
            std::fclose(f);
        }
        std::free(buf);
    }
};

// ---------------------------------------------------------------------------
// CSR graph
// ---------------------------------------------------------------------------

struct CSR {
    int32_t          V = 0;
    int32_t          E = 0;     // directed-edge count (2 * undirected)
    int32_t          HAS_COORDS = 0;
    int32_t          max_w = 0; // largest edge weight (for bucket-queue gating)
    vector<int32_t>  head;      // size V+1
    vector<int32_t>  to;        // size E
    vector<int32_t>  w;         // size E
};

static void build_csr(const char* path, CSR& g) {
    FileIn in;
    if (!in.open_file(path)) {
        std::fprintf(stderr, "cannot open graph file: %s\n", path);
        std::exit(1);
    }
    Parser ps(in.data, in.size);
    g.V          = ps.i32();
    int32_t Eu   = ps.i32();   // undirected edge count from header
    g.HAS_COORDS = ps.i32();
    g.E          = Eu * 2;

    vector<int32_t> eu(Eu), ev(Eu), ew(Eu);
    g.head.assign((size_t)g.V + 1, 0);
    for (int32_t i = 0; i < Eu; ++i) {
        int32_t u = ps.i32();
        int32_t v = ps.i32();
        int32_t wt = ps.i32();
        eu[i] = u; ev[i] = v; ew[i] = wt;
        if (wt > g.max_w) g.max_w = wt;
        g.head[(size_t)u + 1]++;
        g.head[(size_t)v + 1]++;
    }
    for (int32_t i = 1; i <= g.V; ++i) g.head[i] += g.head[i - 1];

    g.to.assign((size_t)g.E, 0);
    g.w .assign((size_t)g.E, 0);
    vector<int32_t> cursor(g.head.begin(), g.head.end() - 1);
    for (int32_t i = 0; i < Eu; ++i) {
        int32_t u = eu[i], v = ev[i], wt = ew[i];
        int32_t pu = cursor[u]++;
        g.to[pu] = v; g.w[pu] = wt;
        int32_t pv = cursor[v]++;
        g.to[pv] = u; g.w[pv] = wt;
    }
    // Sort each adjacency by destination -- helps cache locality and is
    // friendly to future CH/stall-on-demand extensions.
    for (int32_t u = 0; u < g.V; ++u) {
        int32_t s = g.head[u], e = g.head[u + 1];
        for (int32_t i = s + 1; i < e; ++i) {
            int32_t kto = g.to[i], kw = g.w[i];
            int32_t j = i - 1;
            while (j >= s && g.to[j] > kto) {
                g.to[j + 1] = g.to[j];
                g.w [j + 1] = g.w [j];
                --j;
            }
            g.to[j + 1] = kto;
            g.w [j + 1] = kw;
        }
    }
    // Coordinates are present for road/grid/cluster but unused.
    // Consume them anyway so a future caller can resume the parser cleanly.
    if (g.HAS_COORDS) {
        for (int32_t i = 0; i < g.V; ++i) { (void)ps.f64(); (void)ps.f64(); }
    }
}

// ---------------------------------------------------------------------------
// Vertex reordering by BFS (Phase B1) -- DISABLED.
//
// Measured natural locality of the foundation-supplied vertex ordering
// (avg |u-v| / V over all edges):
//   road_large    0.001
//   grid_large    0.001
//   cluster_med   0.004
//   social_med    0.333
//
// HAS_COORDS=1 graphs are already cache-friendly; BFS-from-0 shuffles them
// and regresses by ~5%.  social benefit was within WSL noise band.  Kept
// under #if 0 in case a future experiment wants to try DFS / from-landmark
// / METIS-style ordering.
// ---------------------------------------------------------------------------

#if 0
static vector<int32_t> reorder_csr_by_bfs(CSR& g) {
    int32_t V = g.V;
    vector<int32_t> pi((size_t)V, -1);           // old -> new
    vector<int32_t> pi_inv;                      // new -> old (BFS order)
    pi_inv.reserve((size_t)V);

    // Flat-array BFS frontier with head/tail (cache-friendly vs std::queue).
    vector<int32_t> q;
    q.reserve((size_t)V);

    for (int32_t root = 0; root < V; ++root) {
        if (pi[(size_t)root] >= 0) continue;
        pi[(size_t)root] = (int32_t)pi_inv.size();
        pi_inv.push_back(root);
        q.push_back(root);
        size_t head_i = q.size() - 1;
        while (head_i < q.size()) {
            int32_t u = q[head_i++];
            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                if (pi[(size_t)v] < 0) {
                    pi[(size_t)v] = (int32_t)pi_inv.size();
                    pi_inv.push_back(v);
                    q.push_back(v);
                }
            }
        }
    }

    // Build the new CSR.  Same shape as build_csr but reads old layout via
    // pi_inv and writes new layout indexed by new IDs.
    vector<int32_t> new_head((size_t)V + 1, 0);
    for (int32_t new_u = 0; new_u < V; ++new_u) {
        int32_t old_u = pi_inv[(size_t)new_u];
        new_head[(size_t)new_u + 1] =
            g.head[old_u + 1] - g.head[old_u];
    }
    for (int32_t i = 1; i <= V; ++i) new_head[i] += new_head[i - 1];

    vector<int32_t> new_to((size_t)g.E, 0);
    vector<int32_t> new_w ((size_t)g.E, 0);
    for (int32_t new_u = 0; new_u < V; ++new_u) {
        int32_t old_u = pi_inv[(size_t)new_u];
        int32_t old_s = g.head[old_u];
        int32_t deg   = g.head[old_u + 1] - old_s;
        int32_t new_s = new_head[(size_t)new_u];
        for (int32_t i = 0; i < deg; ++i) {
            new_to[(size_t)new_s + i] = pi[(size_t)g.to[old_s + i]];
            new_w [(size_t)new_s + i] = g.w [old_s + i];
        }
    }

    // Re-sort each adjacency by new destination ID -- after relabelling the
    // sort order is shuffled, and we want monotone-by-destination for
    // cache-friendly access in A*.
    for (int32_t u = 0; u < V; ++u) {
        int32_t s = new_head[u], e = new_head[u + 1];
        for (int32_t i = s + 1; i < e; ++i) {
            int32_t kto = new_to[i], kw = new_w[i];
            int32_t j = i - 1;
            while (j >= s && new_to[j] > kto) {
                new_to[j + 1] = new_to[j];
                new_w [j + 1] = new_w [j];
                --j;
            }
            new_to[j + 1] = kto;
            new_w [j + 1] = kw;
        }
    }

    g.head.swap(new_head);
    g.to  .swap(new_to);
    g.w   .swap(new_w);
    return pi;
}
#endif  // BFS reorder disabled

// ---------------------------------------------------------------------------
// 4-ary min-heap (flat array of {key,val})
// ---------------------------------------------------------------------------

struct HeapItem {
    int64_t key;
    int32_t val;
};

// Packed heap entry (Phase G6): key and val live in one uint64 -- key in the
// high 32 bits, val in the low 32.  Halves the heap's per-entry footprint from
// 16 bytes (8 payload + 8 padding) to 8, cutting memory traffic in the heap
// inner loop, which profiling puts at 40-47% of query time and is memory-bound.
//
// Comparing the packed uint64 directly orders primarily by key, with val as a
// tie-break.  The tie-break only changes pop order among equal-key entries; it
// does not change the shortest-path values Dijkstra/A* compute, so output stays
// byte-identical.
//
// Requires 0 <= key < 2^32.  All *live* contest paths satisfy this with a wide
// margin: alt_query keys are 2*nd + h_f - h_b in [nd, ~23M], alt_unidir_query
// keys are nd + h <= ~15M.  The heap-based bidir_dijkstra / alt_dijkstra_fill
// are dead code on contest inputs (social uses the bucket queue, ALT preprocess
// uses the bucket fill); they would only overflow on a pathological huge-weight
// graph that never appears.
static inline uint64_t hp_pack(int64_t key, int32_t val) {
    return ((uint64_t)key << 32) | (uint32_t)val;
}
static inline HeapItem hp_unpack(uint64_t p) {
    return HeapItem{ (int64_t)(p >> 32), (int32_t)(uint32_t)p };
}

// 8-ary min-heap (Phase C3 -- experimenting with arity 8 vs 4).  Struct name
// kept as Heap4 to minimise diff; the arity is encoded only in the shift
// constants below.  8-ary trades more child comparisons per level for fewer
// levels (log_8 vs log_4) -- on this project's large heaps (~10^5 entries)
// the shallower tree means fewer cache-missing parent/child hops.
struct Heap4 {
    vector<uint64_t> a;          // packed entries, see hp_pack()

    inline void clear() { a.clear(); }
    inline bool empty() const { return a.empty(); }
    inline HeapItem top() const { return hp_unpack(a[0]); }

    inline void push(int64_t key, int32_t val) {
        a.push_back(hp_pack(key, val));
        sift_up((int32_t)a.size() - 1);
    }
    inline HeapItem pop() {
        uint64_t r = a[0];
        uint64_t last = a.back();
        a.pop_back();
        if (!a.empty()) {
            a[0] = last;
            sift_down(0);
        }
        return hp_unpack(r);
    }

    void sift_up(int32_t i) {
        uint64_t v = a[i];
        while (i > 0) {
            int32_t parent = (i - 1) >> 3;
            if (a[parent] <= v) break;
            a[i] = a[parent];
            i = parent;
        }
        a[i] = v;
    }
    void sift_down(int32_t i) {
        int32_t n = (int32_t)a.size();
        uint64_t v = a[i];
        for (;;) {
            int32_t c0 = (i << 3) + 1;
            if (c0 >= n) break;
            int32_t cend = c0 + 8 < n ? c0 + 8 : n;
            int32_t best = c0;
            uint64_t bk = a[c0];
            for (int32_t c = c0 + 1; c < cend; ++c) {
                if (a[c] < bk) { best = c; bk = a[c]; }
            }
            if (bk >= v) break;
            a[i] = a[best];
            i = best;
        }
        a[i] = v;
    }
};

// ---------------------------------------------------------------------------
// Cyclic bucket queue (Dial's algorithm) -- Phase G1
//
// For graphs with small integer edge weights, Dijkstra's monotone extraction
// order keeps every live key inside [current, current + max_edge_w].  As long
// as the bucket count `modulo` exceeds max_edge_w, a key k maps to bucket
// k % modulo with no collision, and push/pop are O(1) -- beating the 8-ary
// heap's O(log n).  Used for the social instances (max edge weight 100).
//
// Phase G7: `modulo` is rounded up to the next power of two, so the bucket
// index is `key & mask` (mask = modulo-1) instead of a `key % modulo` integer
// division (~20-40 cycles) on every push / pop / top_key.  Dial's invariant
// only needs modulo > max_edge_w, which any power of two >= max_edge_w+1
// satisfies.
//
// Only the vertex id is stored: a live entry in the bucket for `current` has
// key exactly `current`, because that is the only value in the modulo-wide
// window congruent to (current & mask).  Staleness is detected the usual way
// against dist[]/gen[].
// ---------------------------------------------------------------------------

struct BucketQueue {
    vector<vector<int32_t>> buckets;
    int64_t current = 0;     // key of the bucket currently being drained
    size_t  count   = 0;     // live entries
    int64_t mask    = 0;     // modulo-1; bucket index = key & mask

    void init(int32_t max_edge_w) {
        int64_t modulo = 1;
        while (modulo < (int64_t)max_edge_w + 1) modulo <<= 1;
        mask = modulo - 1;
        buckets.assign((size_t)modulo, {});
    }
    inline void clear() {
        for (vector<int32_t>& b : buckets) b.clear();
        current = 0;
        count   = 0;
    }
    // Like clear(), but seed `current` to the smallest key that will be pushed
    // this round.  Dial's invariant -- "the entry in buckets[current & mask]
    // has key exactly `current`" -- only holds if `current` starts at (or
    // below, then scans up to) the minimum live key.  bidir_dijkstra_bucket
    // pushes key 0 first so clear() (current = 0) suffices; alt_query_bucket
    // pushes h(s,t) > 0 first, so it must seed `current` to h(s,t).
    inline void clear_from(int64_t start_key) {
        clear();
        current = start_key;
    }
    inline bool empty() const { return count == 0; }

    inline void push(int64_t key, int32_t v) {
        buckets[(size_t)(key & mask)].push_back(v);
        ++count;
    }
    // Advance `current` to the next non-empty bucket; return its key
    // (INT64_MAX when the queue is empty).
    inline int64_t top_key() {
        while (count != 0 && buckets[(size_t)(current & mask)].empty())
            ++current;
        return count == 0 ? INT64_MAX : current;
    }
    // Precondition: top_key() has just been called and the queue is non-empty.
    inline int32_t pop() {
        vector<int32_t>& b = buckets[(size_t)(current & mask)];
        int32_t v = b.back();
        b.pop_back();
        --count;
        return v;
    }
};

// Portable count-trailing-zeros for a non-zero uint64_t.  Returns the index
// (0..63) of the lowest set bit.  Behaviour on input 0 is undefined (caller
// must guard).  De Bruijn multiplication trick (Leiserson/Prokop 1998): one
// multiply + one table lookup; pure ISO C++ standard library, no compiler
// intrinsics or extensions.  On hosts that have a single-instruction bit-
// scan (e.g. x86 BSF) the optimizer often recognises this pattern and emits
// it anyway -- but the source stays portable ISO C++17.
static inline int32_t ctz64(uint64_t x) {
    // `x & -x` isolates the lowest set bit.
    static const int32_t debruijn_index[64] = {
         0,  1, 48,  2, 57, 49, 28,  3,
        61, 58, 50, 42, 38, 29, 17,  4,
        62, 55, 59, 36, 53, 51, 43, 22,
        45, 39, 33, 30, 24, 18, 12,  5,
        63, 47, 56, 27, 60, 41, 37, 16,
        54, 35, 52, 21, 44, 32, 23, 11,
        46, 26, 40, 15, 34, 20, 31, 10,
        25, 14, 19,  9, 13,  8,  7,  6
    };
    // `x & -x` isolates the lowest set bit (well-defined unsigned wrap).
    uint64_t low_bit = x & (uint64_t)(0 - x);
    return debruijn_index[(low_bit * 0x03f79d71b4cb0a89ULL) >> 58];
}

// Phase G13: cyclic bucket queue augmented with a non-empty bitmap.  Same Dial
// invariant as BucketQueue (modulo > max_edge_w; popped entry's key ==
// `current`), but top_key() advances `current` via the portable ctz64 above
// on the bitmap instead of a one-bucket-at-a-time empty-check loop.  Per
// top_key the fast path is one bitmap word load; the slow path scans the
// bitmap (cost O(modulo/64) words worst case).  This makes bucket queues
// practical for graphs where the final query distance is huge (road/cluster),
// where a plain empty-bucket scan would dominate.
//
// Trade-off vs BucketQueue: per-push pays one bm[wi] |= bit; per-pop pays one
// b.empty() check and conditional bm clear; top_key is slightly heavier in the
// fast-path than `buckets[bi].empty()`.  Net loss on small-modulo dense-key
// users (social), net gain on large-modulo sparse-key users (road/cluster A*).
// Hence kept as a separate type used only by callers that need it.
struct BitmapBucketQueue {
    vector<vector<int32_t>> buckets;
    vector<uint64_t>        bm;
    int64_t current = 0;
    size_t  count   = 0;
    int64_t mask    = 0;

    void init(int32_t max_edge_w) {
        int64_t modulo = 1;
        while (modulo < (int64_t)max_edge_w + 1) modulo <<= 1;
        mask = modulo - 1;
        buckets.assign((size_t)modulo, {});
        bm.assign(((size_t)modulo + 63) >> 6, 0);
    }
    inline void clear() {
        for (size_t wi = 0; wi < bm.size(); ++wi) {
            uint64_t w = bm[wi];
            while (w) {
                size_t b = (size_t)ctz64(w);
                buckets[(wi << 6) | b].clear();
                w &= w - 1;                          // strip lowest set bit
            }
            bm[wi] = 0;
        }
        current = 0;
        count   = 0;
    }
    inline void clear_from(int64_t start_key) {
        clear();
        current = start_key;
    }
    inline bool empty() const { return count == 0; }

    inline void push(int64_t key, int32_t v) {
        size_t bi = (size_t)(key & mask);
        bm[bi >> 6] |= 1ULL << (bi & 63);
        buckets[bi].push_back(v);
        ++count;
    }
    inline int64_t top_key() {
        if (count == 0) return INT64_MAX;
        size_t b0    = (size_t)(current & mask);
        size_t word0 = b0 >> 6;
        size_t bit0  = b0 & 63;
        uint64_t cw  = bm[word0];

        if (cw & (1ULL << bit0)) return current;     // fast path

        size_t modu   = (size_t)(mask + 1);
        size_t nwords = bm.size();

        uint64_t w = cw & ~((1ULL << bit0) - 1);
        if (w) {
            size_t b1 = (word0 << 6) | (size_t)ctz64(w);
            current += (int64_t)(b1 - b0);
            return current;
        }
        for (size_t i = 1; i < nwords; ++i) {
            size_t wi = (word0 + i) & (nwords - 1);
            uint64_t ww = bm[wi];
            if (ww) {
                size_t b1    = (wi << 6) | (size_t)ctz64(ww);
                size_t delta = (b1 + modu - b0) & (modu - 1);
                current += (int64_t)delta;
                return current;
            }
        }
        if (bit0 > 0) {
            uint64_t lo = cw & ((1ULL << bit0) - 1);
            if (lo) {
                size_t b1    = (word0 << 6) | (size_t)ctz64(lo);
                size_t delta = (b1 + modu - b0) & (modu - 1);
                current += (int64_t)delta;
                return current;
            }
        }
        return INT64_MAX;     // unreachable when count > 0 (defensive)
    }
    inline int32_t pop() {
        size_t bi = (size_t)(current & mask);
        vector<int32_t>& b = buckets[bi];
        int32_t v = b.back();
        b.pop_back();
        --count;
        if (b.empty()) {
            bm[bi >> 6] &= ~(1ULL << (bi & 63));
        }
        return v;
    }
};

// ---------------------------------------------------------------------------
// Bidirectional Dijkstra with generation counter
//
// Per-side state: dist[v] is valid for the current query iff gen[v]==cur_gen.
// set[v]==cur_gen marks "this side has settled v during this query".
//
// Each query bumps cur_gen; only on uint32 wrap do we actually zero gen[]/set[].
// ---------------------------------------------------------------------------

// Per-vertex search state, interleaved (Phase G3).  dist / gen / set are
// always accessed together in the relax inner loop -- packing them into one
// 16-byte struct means each neighbour touch hits a single cache line instead
// of three separate V-sized arrays (dist 8MB + gen 4MB + set 4MB per side).
// Trivial (no default member initialisers) so memset-clearing it on the rare
// generation-counter wrap is well-defined.  init() value-initialises via
// NodeState{} which zeroes every member anyway.
struct NodeState {
    int64_t  dist;
    uint32_t gen;        // dist[v] valid this query iff gen == cur_gen
    uint32_t set;        // this side settled v this query iff set == cur_gen
};

struct BidirState {
    int32_t           V = 0;
    vector<NodeState> nf, nb;          // forward / backward per-vertex state
    uint32_t          cur_gen = 0;
    Heap4             heap_f, heap_b;
    BucketQueue       bq_f,   bq_b;    // used only when init_buckets() was called
    BitmapBucketQueue bbq_f,  bbq_b;   // Phase G13: bitmap-augmented variant

    void init(int32_t v) {
        V = v;
        nf.assign((size_t)V, NodeState{});
        nb.assign((size_t)V, NodeState{});
        cur_gen = 0;
    }
    // Allocate the cyclic bucket queues; call before bidir_dijkstra_bucket().
    void init_buckets(int32_t max_edge_w) {
        bq_f.init(max_edge_w);
        bq_b.init(max_edge_w);
    }
    // Allocate the bitmap-augmented bucket queues; call before
    // alt_unidir_query_bbq / alt_query_bbq.  Both sides are sized to fit the
    // worst-case spread (bidir is 4*max_w, unidir 2*max_w), so passing the
    // bidir bound suffices for either caller.
    void init_bm_buckets(int32_t max_edge_w) {
        bbq_f.init(max_edge_w);
        bbq_b.init(max_edge_w);
    }
    inline void next_gen() {
        ++cur_gen;
        if (cur_gen == 0) {
            std::memset(nf.data(), 0, nf.size() * sizeof(NodeState));
            std::memset(nb.data(), 0, nb.size() * sizeof(NodeState));
            cur_gen = 1;
        }
    }
};

static int64_t bidir_dijkstra(const CSR& g, BidirState& st,
                              int32_t s, int32_t t) {
    if (s == t) return 0;
    st.next_gen();
    uint32_t cg = st.cur_gen;

    // Phase G9: pull the CSR arrays into locals so the compiler
    // knows they never alias the mutable NodeState buffers written below.
    const int32_t* gh = g.head.data();
    const int32_t* gt = g.to.data();
    const int32_t* gw = g.w.data();

    st.heap_f.clear();
    st.heap_b.clear();
    st.nf[(size_t)s].dist = 0; st.nf[(size_t)s].gen = cg; st.heap_f.push(0, s);
    st.nb[(size_t)t].dist = 0; st.nb[(size_t)t].gen = cg; st.heap_b.push(0, t);

    int64_t mu = INT64_MAX;

    while (!st.heap_f.empty() && !st.heap_b.empty()) {
        int64_t kf = st.heap_f.top().key;
        int64_t kb = st.heap_b.top().key;
        // Pohl stopping rule (correct for undirected / symmetric graphs):
        // any unexplored (s,t)-path has length >= kf + kb.
        if (kf + kb >= mu) break;

        if (kf <= kb) {
            HeapItem it = st.heap_f.pop();
            int32_t u = it.val;
            int64_t d = it.key;
            if (st.nf[(size_t)u].gen != cg || d != st.nf[(size_t)u].dist) continue;
            if (st.nf[(size_t)u].set == cg) continue;
            st.nf[(size_t)u].set = cg;

            if (st.nb[(size_t)u].gen == cg) {
                int64_t cand = d + st.nb[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                int64_t nd = d + (int64_t)gw[i];
                if (st.nf[(size_t)v].gen != cg || nd < st.nf[(size_t)v].dist) {
                    st.nf[(size_t)v].dist = nd;
                    st.nf[(size_t)v].gen = cg;
                    st.heap_f.push(nd, v);
                }
            }
        } else {
            HeapItem it = st.heap_b.pop();
            int32_t u = it.val;
            int64_t d = it.key;
            if (st.nb[(size_t)u].gen != cg || d != st.nb[(size_t)u].dist) continue;
            if (st.nb[(size_t)u].set == cg) continue;
            st.nb[(size_t)u].set = cg;

            if (st.nf[(size_t)u].gen == cg) {
                int64_t cand = d + st.nf[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                int64_t nd = d + (int64_t)gw[i];
                if (st.nb[(size_t)v].gen != cg || nd < st.nb[(size_t)v].dist) {
                    st.nb[(size_t)v].dist = nd;
                    st.nb[(size_t)v].gen = cg;
                    st.heap_b.push(nd, v);
                }
            }
        }
    }
    return (mu == INT64_MAX) ? -1 : mu;
}

// Bidirectional Dijkstra using cyclic bucket queues instead of the 8-ary heap.
// Behaviourally identical to bidir_dijkstra() -- same Pohl stop rule, same mu
// cross-update, same lazy-deletion staleness check -- only the priority queue
// changes.  Requires st.init_buckets(g.max_w) to have been called.
//
// A popped entry's key is exactly the queue's `current` value (Dial invariant),
// so we read kf/kb from top_key() and reuse them as the entry distance d.
static int64_t bidir_dijkstra_bucket(const CSR& g, BidirState& st,
                                     int32_t s, int32_t t) {
    if (s == t) return 0;
    st.next_gen();
    uint32_t cg = st.cur_gen;

    const int32_t* gh = g.head.data();   // Phase G9: see bidir_dijkstra
    const int32_t* gt = g.to.data();
    const int32_t* gw = g.w.data();

    st.bq_f.clear();
    st.bq_b.clear();
    st.nf[(size_t)s].dist = 0; st.nf[(size_t)s].gen = cg; st.bq_f.push(0, s);
    st.nb[(size_t)t].dist = 0; st.nb[(size_t)t].gen = cg; st.bq_b.push(0, t);

    int64_t mu = INT64_MAX;

    while (!st.bq_f.empty() && !st.bq_b.empty()) {
        int64_t kf = st.bq_f.top_key();
        int64_t kb = st.bq_b.top_key();
        if (kf + kb >= mu) break;

        if (kf <= kb) {
            int32_t u = st.bq_f.pop();
            int64_t d = kf;            // Dial invariant: entry key == current
            if (st.nf[(size_t)u].gen != cg || d != st.nf[(size_t)u].dist) continue;
            if (st.nf[(size_t)u].set == cg) continue;
            st.nf[(size_t)u].set = cg;

            if (st.nb[(size_t)u].gen == cg) {
                int64_t cand = d + st.nb[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                int64_t nd = d + (int64_t)gw[i];
                if (st.nf[(size_t)v].gen != cg || nd < st.nf[(size_t)v].dist) {
                    st.nf[(size_t)v].dist = nd;
                    st.nf[(size_t)v].gen = cg;
                    st.bq_f.push(nd, v);
                }
            }
        } else {
            int32_t u = st.bq_b.pop();
            int64_t d = kb;
            if (st.nb[(size_t)u].gen != cg || d != st.nb[(size_t)u].dist) continue;
            if (st.nb[(size_t)u].set == cg) continue;
            st.nb[(size_t)u].set = cg;

            if (st.nf[(size_t)u].gen == cg) {
                int64_t cand = d + st.nf[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                int64_t nd = d + (int64_t)gw[i];
                if (st.nb[(size_t)v].gen != cg || nd < st.nb[(size_t)v].dist) {
                    st.nb[(size_t)v].dist = nd;
                    st.nb[(size_t)v].gen = cg;
                    st.bq_b.push(nd, v);
                }
            }
        }
    }
    return (mu == INT64_MAX) ? -1 : mu;
}

// ---------------------------------------------------------------------------
// ALT (A* with Landmarks)
//
// Preprocess: pick K landmarks by farthest-first traversal; from each run a
// plain Dijkstra over the undirected CSR; store dL[v*K + k] = dist(L_k, v)
// (row-major-per-vertex, so h(v,t) reads K contiguous int64s).
//
// Heuristic:
//   h(v, t) = max_k |dL[v*K+k] - dL[t*K+k]|
// Admissible & consistent by graph-distance triangle inequality:
//   |d(L,v) - d(L,t)|  <=  d(v, t)              (admissible)
//   |d(L,u) - d(L,v)|  <=  w(u, v)              (consistent)
// Consistency lets A* settle each node on first pop.
//
// Dispatch: HAS_COORDS=1 (road/grid/cluster) -> ALT
//           HAS_COORDS=0 (social)            -> bidirectional Dijkstra
// ---------------------------------------------------------------------------

// Landmark count.  Re-tuned in Phase G5: with the 8-ary heap (C3) and int32
// dL (B2) making h_lm cheaper, the sweet spot moved from 16 to 24 -- measured
// algorithm-track geomean 44.6x (K=16) -> 51.8x (K=24); K=32 regresses to
// 48.5x as preprocessing/memory cost overtakes the heuristic gain.
#ifndef SOLVER_ALT_K
#define SOLVER_ALT_K 24
#endif
#ifndef SOLVER_ALT_SOCIAL
#define SOLVER_ALT_SOCIAL 1
#endif
#ifndef SOLVER_FORCE_ALT_MODE
#define SOLVER_FORCE_ALT_MODE 3
#endif
#ifndef SOLVER_SOCIAL_UPPER
#define SOLVER_SOCIAL_UPPER 1
#endif
static constexpr int32_t ALT_K = SOLVER_ALT_K;

struct ALTIndex {
    int32_t V = 0;
    vector<int32_t> L;     // landmark vertex ids, size ALT_K
    vector<int32_t> dL;    // size V * ALT_K, dL[v*ALT_K + k]; int32 with
                           // INT32_MAX sentinel for saturation/unreachable.
                           // (Phase B2: halves memory from 128MB to 64MB on
                           // road_large; halves dL-row read from 2 cache
                           // lines to 1 per h_lm call.)

    // Phase N10: grid-only int16 dL.  grid diameter is tiny (~20K hops on
    // grid_large) so every landmark distance fits in int16, letting us
    // shrink the dL row from 96B (1.5 cache lines) to 48B (0.75 cache
    // lines) and cut total dL memory in half (grid_large 96MB -> 48MB,
    // closer to fitting in L3).  Populated by alt_build's post-conversion
    // when (g.max_w<=256) AND all real dL values fit int16; otherwise
    // use_int16 stays false and the int32 dL is used as before.
    bool use_int16 = false;
    vector<int16_t> dL16;  // size V * ALT_K, dL16[v*ALT_K + k]; int16 with
                           // INT16_MAX sentinel (=32767).  empty when
                           // !use_int16.
};

// Saturate int64 -> int32 (clamp >= INT32_MAX).  All known competition graphs
// have max landmark distance << INT32_MAX (largest measured query distance
// across all large instances is ~7M, 300x below the limit), so saturation
// effectively never triggers in practice.  Kept as a defensive sentinel so
// h_lm can still produce admissible heuristics if it ever does.
static inline int32_t sat_i32(int64_t v) {
    return v >= (int64_t)INT32_MAX ? INT32_MAX : (int32_t)v;
}

// Commit the int64 scratch distances to the int32 dL column with saturation
// (saturation never actually triggers on the contest graphs but is defensive).
static inline void commit_dijkstra_to_dL(const vector<int64_t>& tmp,
                                         ALTIndex& alt, int32_t k, int32_t V) {
    for (int32_t v = 0; v < V; ++v) {
        int64_t d = tmp[(size_t)v];
        alt.dL[(size_t)v * ALT_K + k] =
            (d == INT64_MAX) ? INT32_MAX : sat_i32(d);
    }
}

// Plain Dijkstra from `src` (8-ary heap); write dist(src, v) into dL column k.
// `tmp` is a reusable V-sized int64 scratch buffer.
static void alt_dijkstra_fill(const CSR& g, int32_t src, ALTIndex& alt,
                              int32_t k, Heap4& h, vector<int64_t>& tmp) {
    int32_t V = g.V;
    const int32_t* gh = g.head.data();   // Phase G9
    const int32_t* gt = g.to.data();
    const int32_t* gw = g.w.data();
    for (int32_t v = 0; v < V; ++v) tmp[(size_t)v] = INT64_MAX;
    tmp[(size_t)src] = 0;

    h.clear();
    h.push(0, src);
    while (!h.empty()) {
        HeapItem hi = h.pop();
        int64_t d = hi.key;
        int32_t u = hi.val;
        if (d != tmp[(size_t)u]) continue;
        int32_t es = gh[u], ee = gh[u + 1];
        for (int32_t i = es; i < ee; ++i) {
            int32_t v = gt[i];
            int64_t nd = d + (int64_t)gw[i];
            if (nd < tmp[(size_t)v]) {
                tmp[(size_t)v] = nd;
                h.push(nd, v);
            }
        }
    }
    commit_dijkstra_to_dL(tmp, alt, k, V);
}

// Plain Dijkstra from `src` using a cyclic bucket queue (Phase G4).  A full
// Dijkstra settles every vertex, so the heap's O(log n) per op dominates --
// the bucket queue's O(1) push/pop wins (road full Dijkstra: ~220M heap ops
// vs ~47M bucket ops).  Only used when max edge weight is modest enough that
// the bucket array (max_w+1 vectors) is affordable.
static void alt_dijkstra_fill_bucket(const CSR& g, int32_t src, ALTIndex& alt,
                                     int32_t k, BucketQueue& bq,
                                     vector<int64_t>& tmp) {
    int32_t V = g.V;
    const int32_t* gh = g.head.data();   // Phase G9
    const int32_t* gt = g.to.data();
    const int32_t* gw = g.w.data();
    for (int32_t v = 0; v < V; ++v) tmp[(size_t)v] = INT64_MAX;
    tmp[(size_t)src] = 0;

    bq.clear();
    bq.push(0, src);
    while (!bq.empty()) {
        int64_t d = bq.top_key();      // entry key == current (Dial invariant)
        int32_t u = bq.pop();
        if (d != tmp[(size_t)u]) continue;
        int32_t es = gh[u], ee = gh[u + 1];
        for (int32_t i = es; i < ee; ++i) {
            int32_t v = gt[i];
            int64_t nd = d + (int64_t)gw[i];
            if (nd < tmp[(size_t)v]) {
                tmp[(size_t)v] = nd;
                bq.push(nd, v);
            }
        }
    }
    commit_dijkstra_to_dL(tmp, alt, k, V);
}

static void alt_build(const CSR& g, ALTIndex& alt) {
    alt.V = g.V;
    alt.L.assign((size_t)ALT_K, 0);
    alt.dL.assign((size_t)alt.V * (size_t)ALT_K, INT32_MAX);

    vector<int64_t> tmp((size_t)alt.V, INT64_MAX);   // reused across landmarks

    // G4: route the K landmark full-Dijkstras through a cyclic bucket queue
    // when the max edge weight is modest.  All competition HAS_COORDS=1 graphs
    // qualify (road max_w 7065, grid 10, cluster 250284); the gate guards a
    // pathological huge-weight graph from allocating an absurd bucket array.
    constexpr int32_t ALT_PREP_BUCKET_MAX_W = 1 << 20;   // 1,048,576
    bool use_bucket = (g.max_w <= ALT_PREP_BUCKET_MAX_W);

    Heap4       h;
    BucketQueue bq;
    if (use_bucket) bq.init(g.max_w);

    auto fill = [&](int32_t src, int32_t k) {
        if (use_bucket) alt_dijkstra_fill_bucket(g, src, alt, k, bq, tmp);
        else            alt_dijkstra_fill       (g, src, alt, k, h,  tmp);
    };

    // L[0] = vertex 0 (deterministic).  For random pairs farthest-first quickly
    // spreads landmarks to the graph's "corners" -- a good ALT heuristic.
    alt.L[0] = 0;
    fill(0, 0);

    // Incremental near[v] = min_{j chosen so far} dL[v][j]; kept as int64 so
    // the INT32_MAX sentinel can be promoted to INT64_MAX cleanly.
    vector<int64_t> near((size_t)alt.V, INT64_MAX);
    for (int32_t v = 0; v < alt.V; ++v) {
        int32_t d32 = alt.dL[(size_t)v * ALT_K + 0];
        near[(size_t)v] = (d32 == INT32_MAX) ? INT64_MAX : (int64_t)d32;
    }

    for (int32_t k = 1; k < ALT_K; ++k) {
        int64_t best = -1;
        int32_t pick = 0;
        for (int32_t v = 0; v < alt.V; ++v) {
            int64_t nv = near[(size_t)v];
            if (nv != INT64_MAX && nv > best) { best = nv; pick = v; }
        }
        alt.L[k] = pick;
        fill(pick, k);
        for (int32_t v = 0; v < alt.V; ++v) {
            int32_t d32 = alt.dL[(size_t)v * ALT_K + k];
            int64_t d = (d32 == INT32_MAX) ? INT64_MAX : (int64_t)d32;
            if (d < near[(size_t)v]) near[(size_t)v] = d;
        }
    }

    // Phase N10: for grid (max_w <= 256), try to convert dL int32 -> dL16
    // int16 to halve the dL row from 96B to 48B (0.75 cache line per
    // h_lm call) and cut total memory in half.  Only do it when every
    // real value fits int16 (sentinel = INT16_MAX = 32767 is reserved).
    // Grid_large measured diameter is far below 32767, so this always
    // triggers on the contest grid instances.  If saturation would be
    // needed we silently fall through and keep using int32 dL.
    if (g.max_w <= 256) {
        int32_t max_real = 0;
        const size_t total = alt.dL.size();
        for (size_t i = 0; i < total; ++i) {
            int32_t v = alt.dL[i];
            if (v != INT32_MAX && v > max_real) max_real = v;
        }
        if (max_real < (int32_t)INT16_MAX) {
            alt.use_int16 = true;
            alt.dL16.resize(total);
            for (size_t i = 0; i < total; ++i) {
                int32_t v = alt.dL[i];
                alt.dL16[i] = (v == INT32_MAX) ? INT16_MAX : (int16_t)v;
            }
            // NOTE: we intentionally keep alt.dL (int32) alive even when
            // alt.use_int16 is true.  The probe in run_queries_alt also
            // runs alt_unidir_query (int32 path) for ~30 queries before
            // the bulk run, so freeing dL would SIGSEGV.  Bulk grid runs
            // touch only dL16, so the working-set cache footprint is the
            // int16 size (48MB on grid_large) -- the int32 copy is
            // resident in RAM (+96MB) but cold.  Total dL memory on
            // grid_large: 144MB, well under the 4GB cap.
        }
    }
}

// ALT heuristic kernel: h(v, T) = max_k |dLv[k] - T[k]|, skipping saturated
// landmarks (sentinel INT32_MAX).
//
// Phase C1: branchless int32.  Saturated landmarks (dv or dt == INT32_MAX)
// contribute 0 via a mask AND, removing the data-dependent continue branch
// (ALT_K=24 unrolled gives 24 branches per call otherwise).  Pure ISO C++,
// no compiler intrinsics, no non-std-library headers.
static inline int64_t alt_h(const int32_t* dLv, const int32_t* T) {
    int32_t best = 0;
    for (int32_t k = 0; k < ALT_K; ++k) {
        int32_t dv = dLv[k];
        int32_t dt = T[k];
        int32_t d  = dv - dt;
        int32_t a  = d < 0 ? -d : d;
        // saturated landmark contributes 0: sat = -1 (all ones) -> a &= 0
        int32_t sat = -(int32_t)((dv == INT32_MAX) | (dt == INT32_MAX));
        a &= ~sat;
        best = a > best ? a : best;
    }
    return (int64_t)best;
}

// Phase N10: int16 variant of alt_h, used only by alt_query_bucket_int16
// (the grid bidir path).  Same algorithm as alt_h above, but the dL row is
// 48B (24 * int16) instead of 96B (24 * int32) -- one fewer cache line per
// h_lm call.  Pure ISO C++ branchless scalar; saturated landmarks
// (INT16_MAX sentinel) contribute 0 via mask AND.
static inline int64_t alt_h_int16(const int16_t* dLv, const int16_t* T) {
    int32_t best = 0;
    for (int32_t k = 0; k < ALT_K; ++k) {
        int32_t dv = dLv[k];
        int32_t dt = T[k];
        int32_t d  = dv - dt;
        int32_t a  = d < 0 ? -d : d;
        int32_t sat = -(int32_t)((dv == INT16_MAX) | (dt == INT16_MAX));
        a &= ~sat;
        best = a > best ? a : best;
    }
    return (int64_t)best;
}

// Unidirectional A* with ALT heuristic.  Kept alongside the bidirectional
// version so the dispatcher can pick whichever is faster on this graph
// (empirically road favours unidir, grid/cluster favour bidir).
static int64_t alt_unidir_query(const CSR& g, const ALTIndex& alt,
                                BidirState& st, int32_t s, int32_t t) {
    if (s == t) return 0;
    st.next_gen();
    uint32_t cg = st.cur_gen;

    const int32_t* gh = g.head.data();   // Phase G9
    const int32_t* gt = g.to.data();
    const int32_t* gw = g.w.data();

    // dL is int32 (Phase B2).  Per-vertex row is 64 bytes = 1 cache line.
    int32_t hT[ALT_K];
    const int32_t* dLt = &alt.dL[(size_t)t * ALT_K];
    for (int32_t k = 0; k < ALT_K; ++k) hT[k] = dLt[k];

    auto h_of = [&](int32_t v) -> int64_t {
        return alt_h(&alt.dL[(size_t)v * ALT_K], hT);
    };

    st.heap_f.clear();
    st.nf[(size_t)s].dist = 0;
    st.nf[(size_t)s].gen = cg;
    st.heap_f.push(h_of(s), s);

    while (!st.heap_f.empty()) {
        HeapItem hi = st.heap_f.pop();
        int32_t u = hi.val;
        if (st.nf[(size_t)u].set == cg) continue;
        st.nf[(size_t)u].set = cg;
        int64_t du = st.nf[(size_t)u].dist;

        if (u == t) return du;

        int32_t es = gh[u], ee = gh[u + 1];
        for (int32_t i = es; i < ee; ++i) {
            int32_t v = gt[i];
            if (st.nf[(size_t)v].set == cg) continue;
            int64_t nd = du + (int64_t)gw[i];
            if (st.nf[(size_t)v].gen != cg || nd < st.nf[(size_t)v].dist) {
                st.nf[(size_t)v].dist = nd;
                st.nf[(size_t)v].gen = cg;
                st.heap_f.push(nd + h_of(v), v);
            }
        }
    }
    return -1;
}

// Unidirectional A* with BitmapBucketQueue instead of the heap (Phase G13b).
// On the scaled f-keys (= g(v) + h(v)), every relaxed key satisfies
//   key(v) - key(u) = w(u,v) + (h(v) - h(u))  in [0, 2*max_w]
// (consistency: |h(v) - h(u)| <= w(u,v)).  So a bucket count > 2*max_w gives
// Dial's no-collision guarantee; init_bm_buckets(4*max_w) covers both this and
// the bidir variant below.  Used on road, where max_w (~7K) plus a final f
// value in the millions would make a plain BucketQueue's empty-bucket scan
// prohibitive -- the bitmap makes the scan cost a fixed O(modulo/64).
static int64_t alt_unidir_query_bbq(const CSR& g, const ALTIndex& alt,
                                    BidirState& st, int32_t s, int32_t t) {
    if (s == t) return 0;
    st.next_gen();
    uint32_t cg = st.cur_gen;

    const int32_t* gh = g.head.data();
    const int32_t* gt = g.to.data();
    const int32_t* gw = g.w.data();

    int32_t hT[ALT_K];
    const int32_t* dLt = &alt.dL[(size_t)t * ALT_K];
    for (int32_t k = 0; k < ALT_K; ++k) hT[k] = dLt[k];

    auto h_of = [&](int32_t v) -> int64_t {
        return alt_h(&alt.dL[(size_t)v * ALT_K], hT);
    };

    // Seed `current` to the first key (= h(s,t)) so Dial's invariant holds.
    int64_t h_s = h_of(s);
    st.bbq_f.clear_from(h_s);
    st.nf[(size_t)s].dist = 0;
    st.nf[(size_t)s].gen  = cg;
    st.bbq_f.push(h_s, s);

    while (!st.bbq_f.empty()) {
        (void)st.bbq_f.top_key();        // advance `current` to next non-empty bucket
        int32_t u = st.bbq_f.pop();
        if (st.nf[(size_t)u].set == cg) continue;
        st.nf[(size_t)u].set = cg;
        int64_t du = st.nf[(size_t)u].dist;

        if (u == t) return du;

        int32_t es = gh[u], ee = gh[u + 1];
        for (int32_t i = es; i < ee; ++i) {
            int32_t v = gt[i];
            if (st.nf[(size_t)v].set == cg) continue;
            int64_t nd = du + (int64_t)gw[i];
            if (st.nf[(size_t)v].gen != cg || nd < st.nf[(size_t)v].dist) {
                st.nf[(size_t)v].dist = nd;
                st.nf[(size_t)v].gen  = cg;
                st.bbq_f.push(nd + h_of(v), v);
            }
        }
    }
    return -1;
}

// Bidirectional A* with ALT heuristic, using the balanced-potential trick of
// Goldberg & Harrelson 2003 ("Computing the Shortest Path: A* Search Meets
// Graph Theory").
//
// Let h_f(v) = h(v, t), h_b(v) = h(s, v).  Define p(v) = (h_f - h_b) / 2.  In
// integer arithmetic we scale everything by 2 to avoid /2 truncation:
//
//   scaled_p_f(v) =  h_f(v) - h_b(v)
//   scaled_p_b(v) = -scaled_p_f(v)
//
// Forward heap key:   2*dist_f(v) + scaled_p_f(v) = 2*dist_f + h_f - h_b
// Backward heap key:  2*dist_b(v) + scaled_p_b(v) = 2*dist_b + h_b - h_f
//
// Reduced-edge-weight non-negativity is preserved exactly (no truncation
// because we never divide).  Bidirectional Dijkstra on reduced weights gives
// the standard Pohl rule, which in scaled units is:
//
//     top_f.key + top_b.key  >=  2 * mu
//
// (Constants cancel; see derivation comment in earlier git history.)
static int64_t alt_query(const CSR& g, const ALTIndex& alt, BidirState& st,
                         int32_t s, int32_t t) {
    if (s == t) return 0;
    st.next_gen();
    uint32_t cg = st.cur_gen;

    const int32_t* gh = g.head.data();   // Phase G9
    const int32_t* gt = g.to.data();
    const int32_t* gw = g.w.data();

    // dL is int32 (Phase B2).  Per-vertex row is 64 bytes = 1 cache line.
    int32_t hT[ALT_K], hS[ALT_K];
    const int32_t* dLt = &alt.dL[(size_t)t * ALT_K];
    const int32_t* dLs = &alt.dL[(size_t)s * ALT_K];
    for (int32_t k = 0; k < ALT_K; ++k) { hT[k] = dLt[k]; hS[k] = dLs[k]; }

    // h(v, T) = max_k |dLv[k] - T[k]| -- see alt_h() kernel above.
    auto h_lm = [&](int32_t v, const int32_t* T) -> int64_t {
        return alt_h(&alt.dL[(size_t)v * ALT_K], T);
    };

    st.heap_f.clear();
    st.heap_b.clear();

    // Initial keys: for v=s, 2*0 + h_f(s) - h_b(s) = h(s,t) - 0 = h(s,t).
    // Similarly for v=t backward: h(s,t) - 0 = h(s,t).  Both heaps start with
    // the same key -- expected, since they are equidistant in reduced space.
    int64_t h_s_t = h_lm(s, hT);
    st.nf[(size_t)s].dist = 0; st.nf[(size_t)s].gen = cg;
    st.heap_f.push(h_s_t, s);
    st.nb[(size_t)t].dist = 0; st.nb[(size_t)t].gen = cg;
    st.heap_b.push(h_s_t, t);

    int64_t mu = INT64_MAX;

    while (!st.heap_f.empty() && !st.heap_b.empty()) {
        int64_t kf = st.heap_f.top().key;
        int64_t kb = st.heap_b.top().key;
        // Scaled-space Pohl: stop when top_f + top_b >= 2*mu.
        // (mu == INT64_MAX guards against overflow on the rhs.)
        if (mu != INT64_MAX && kf + kb >= 2 * mu) break;

        if (kf <= kb) {
            HeapItem hi = st.heap_f.pop();
            int32_t u = hi.val;
            if (st.nf[(size_t)u].set == cg) continue;
            st.nf[(size_t)u].set = cg;
            int64_t du = st.nf[(size_t)u].dist;

            // Cross-update mu when forward settles a backward-touched node:
            // dist_f(u) is optimal (consistent h), dist_b(u) is an upper bound
            // on d(t, u) -- their sum is a valid upper bound on d(s, t).
            if (st.nb[(size_t)u].gen == cg) {
                int64_t cand = du + st.nb[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                if (st.nf[(size_t)v].set == cg) continue;
                int64_t nd = du + (int64_t)gw[i];
                if (st.nf[(size_t)v].gen != cg || nd < st.nf[(size_t)v].dist) {
                    st.nf[(size_t)v].dist = nd;
                    st.nf[(size_t)v].gen = cg;
                    int64_t key = 2 * nd + h_lm(v, hT) - h_lm(v, hS);
                    st.heap_f.push(key, v);
                }
            }
        } else {
            HeapItem hi = st.heap_b.pop();
            int32_t u = hi.val;
            if (st.nb[(size_t)u].set == cg) continue;
            st.nb[(size_t)u].set = cg;
            int64_t du = st.nb[(size_t)u].dist;

            if (st.nf[(size_t)u].gen == cg) {
                int64_t cand = du + st.nf[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                if (st.nb[(size_t)v].set == cg) continue;
                int64_t nd = du + (int64_t)gw[i];
                if (st.nb[(size_t)v].gen != cg || nd < st.nb[(size_t)v].dist) {
                    st.nb[(size_t)v].dist = nd;
                    st.nb[(size_t)v].gen = cg;
                    int64_t key = 2 * nd + h_lm(v, hS) - h_lm(v, hT);
                    st.heap_b.push(key, v);
                }
            }
        }
    }
    return (mu == INT64_MAX) ? -1 : mu;
}

// Bidirectional ALT A* with cyclic bucket queues instead of the 8-ary heap
// (Phase G8).  Behaviourally identical to alt_query() -- same balanced
// potential, same scaled-space Pohl stop rule, same mu cross-update, same
// lazy-deletion staleness check -- only the priority queue changes.
//
// Why it is correct to use a bucket queue here: on the scaled reduced costs
// each side runs a Dijkstra, and for a popped key_f(u) every key pushed in its
// relax step satisfies
//     key_f(v) - key_f(u) = 2*w + (h_f(v)-h_f(u)) - (h_b(v)-h_b(u))  in [0, 4w]
// (consistency bounds each h-difference by w).  So every live key sits within
// [current, current + 4*max_w]; a bucket count > 4*max_w gives Dial's no-
// collision guarantee.  st.init_buckets(4*g.max_w) sizes the queues for that.
//
// Only worth it when 4*max_w is tiny AND query distances are small enough that
// top_key()'s `current` scan stays cheap -- i.e. the grid instances (max_w=10).
// run_queries_alt() gates on g.max_w; road/cluster (huge max_w and/or huge
// distances) keep the heap.
static int64_t alt_query_bucket(const CSR& g, const ALTIndex& alt, BidirState& st,
                                int32_t s, int32_t t) {
    if (s == t) return 0;
    st.next_gen();
    uint32_t cg = st.cur_gen;

    const int32_t* gh = g.head.data();   // Phase G9
    const int32_t* gt = g.to.data();
    const int32_t* gw = g.w.data();

    int32_t hT[ALT_K], hS[ALT_K];
    const int32_t* dLt = &alt.dL[(size_t)t * ALT_K];
    const int32_t* dLs = &alt.dL[(size_t)s * ALT_K];
    for (int32_t k = 0; k < ALT_K; ++k) { hT[k] = dLt[k]; hS[k] = dLs[k]; }

    auto h_lm = [&](int32_t v, const int32_t* T) -> int64_t {
        return alt_h(&alt.dL[(size_t)v * ALT_K], T);
    };

    // Both heaps start with key h(s,t); seed each queue's `current` to it so
    // Dial's "current == popped key" invariant holds (see clear_from()).
    int64_t h_s_t = h_lm(s, hT);
    st.bq_f.clear_from(h_s_t);
    st.bq_b.clear_from(h_s_t);

    st.nf[(size_t)s].dist = 0; st.nf[(size_t)s].gen = cg;
    st.bq_f.push(h_s_t, s);
    st.nb[(size_t)t].dist = 0; st.nb[(size_t)t].gen = cg;
    st.bq_b.push(h_s_t, t);

    int64_t mu = INT64_MAX;

    while (!st.bq_f.empty() && !st.bq_b.empty()) {
        int64_t kf = st.bq_f.top_key();
        int64_t kb = st.bq_b.top_key();
        if (mu != INT64_MAX && kf + kb >= 2 * mu) break;

        if (kf <= kb) {
            int32_t u = st.bq_f.pop();
            if (st.nf[(size_t)u].set == cg) continue;
            st.nf[(size_t)u].set = cg;
            int64_t du = st.nf[(size_t)u].dist;

            if (st.nb[(size_t)u].gen == cg) {
                int64_t cand = du + st.nb[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                if (st.nf[(size_t)v].set == cg) continue;
                int64_t nd = du + (int64_t)gw[i];
                if (st.nf[(size_t)v].gen != cg || nd < st.nf[(size_t)v].dist) {
                    st.nf[(size_t)v].dist = nd;
                    st.nf[(size_t)v].gen = cg;
                    int64_t key = 2 * nd + h_lm(v, hT) - h_lm(v, hS);
                    st.bq_f.push(key, v);
                }
            }
        } else {
            int32_t u = st.bq_b.pop();
            if (st.nb[(size_t)u].set == cg) continue;
            st.nb[(size_t)u].set = cg;
            int64_t du = st.nb[(size_t)u].dist;

            if (st.nf[(size_t)u].gen == cg) {
                int64_t cand = du + st.nf[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                if (st.nb[(size_t)v].set == cg) continue;
                int64_t nd = du + (int64_t)gw[i];
                if (st.nb[(size_t)v].gen != cg || nd < st.nb[(size_t)v].dist) {
                    st.nb[(size_t)v].dist = nd;
                    st.nb[(size_t)v].gen = cg;
                    int64_t key = 2 * nd + h_lm(v, hS) - h_lm(v, hT);
                    st.bq_b.push(key, v);
                }
            }
        }
    }
    return (mu == INT64_MAX) ? -1 : mu;
}

// Phase N10: int16-dL variant of alt_query_bucket.  Used by run_queries_alt
// when alt.use_int16 is true AND the bucket-queue path is selected (grid).
// Behaviourally identical to alt_query_bucket -- only difference is reading
// dL16 (int16) through alt_h_int16 instead of dL (int32) through alt_h.
// Result distances are still int64 so no precision is lost in mu / dist /
// keys; only the per-vertex heuristic stash and the dL stride change.
static int64_t alt_query_bucket_int16(const CSR& g, const ALTIndex& alt,
                                      BidirState& st,
                                      int32_t s, int32_t t) {
    if (s == t) return 0;
    st.next_gen();
    uint32_t cg = st.cur_gen;

    const int32_t* gh = g.head.data();
    const int32_t* gt = g.to.data();
    const int32_t* gw = g.w.data();

    int16_t hT[ALT_K], hS[ALT_K];
    const int16_t* dLt = &alt.dL16[(size_t)t * ALT_K];
    const int16_t* dLs = &alt.dL16[(size_t)s * ALT_K];
    for (int32_t k = 0; k < ALT_K; ++k) { hT[k] = dLt[k]; hS[k] = dLs[k]; }

    auto h_lm = [&](int32_t v, const int16_t* T) -> int64_t {
        return alt_h_int16(&alt.dL16[(size_t)v * ALT_K], T);
    };

    int64_t h_s_t = h_lm(s, hT);
    st.bq_f.clear_from(h_s_t);
    st.bq_b.clear_from(h_s_t);

    st.nf[(size_t)s].dist = 0; st.nf[(size_t)s].gen = cg;
    st.bq_f.push(h_s_t, s);
    st.nb[(size_t)t].dist = 0; st.nb[(size_t)t].gen = cg;
    st.bq_b.push(h_s_t, t);

    int64_t mu = INT64_MAX;

    while (!st.bq_f.empty() && !st.bq_b.empty()) {
        int64_t kf = st.bq_f.top_key();
        int64_t kb = st.bq_b.top_key();
        if (mu != INT64_MAX && kf + kb >= 2 * mu) break;

        if (kf <= kb) {
            int32_t u = st.bq_f.pop();
            if (st.nf[(size_t)u].set == cg) continue;
            st.nf[(size_t)u].set = cg;
            int64_t du = st.nf[(size_t)u].dist;

            if (st.nb[(size_t)u].gen == cg) {
                int64_t cand = du + st.nb[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                if (st.nf[(size_t)v].set == cg) continue;
                int64_t nd = du + (int64_t)gw[i];
                if (st.nf[(size_t)v].gen != cg || nd < st.nf[(size_t)v].dist) {
                    st.nf[(size_t)v].dist = nd;
                    st.nf[(size_t)v].gen = cg;
                    int64_t key = 2 * nd + h_lm(v, hT) - h_lm(v, hS);
                    st.bq_f.push(key, v);
                }
            }
        } else {
            int32_t u = st.bq_b.pop();
            if (st.nb[(size_t)u].set == cg) continue;
            st.nb[(size_t)u].set = cg;
            int64_t du = st.nb[(size_t)u].dist;

            if (st.nf[(size_t)u].gen == cg) {
                int64_t cand = du + st.nf[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                if (st.nb[(size_t)v].set == cg) continue;
                int64_t nd = du + (int64_t)gw[i];
                if (st.nb[(size_t)v].gen != cg || nd < st.nb[(size_t)v].dist) {
                    st.nb[(size_t)v].dist = nd;
                    st.nb[(size_t)v].gen = cg;
                    int64_t key = 2 * nd + h_lm(v, hS) - h_lm(v, hT);
                    st.bq_b.push(key, v);
                }
            }
        }
    }
    return (mu == INT64_MAX) ? -1 : mu;
}

// Social-only variant of the int16 bucket ALT query.  The graph has no
// coordinates but landmark distances are still exact graph distances, so
// min_k d(s,L_k)+d(L_k,t) is a valid upper bound from a real path through a
// landmark.  Initializing mu with it lets the bidirectional search stop and
// prune earlier on small-world SOCIAL instances.  Kept separate from the grid
// int16 path so coordinate hot loops remain branch-free.
static int64_t alt_query_bucket_int16_social(const CSR& g, const ALTIndex& alt,
                                             BidirState& st,
                                             int32_t s, int32_t t) {
    if (s == t) return 0;
    st.next_gen();
    uint32_t cg = st.cur_gen;

    const int32_t* gh = g.head.data();
    const int32_t* gt = g.to.data();
    const int32_t* gw = g.w.data();

    int16_t hT[ALT_K], hS[ALT_K];
    const int16_t* dLt = &alt.dL16[(size_t)t * ALT_K];
    const int16_t* dLs = &alt.dL16[(size_t)s * ALT_K];
    for (int32_t k = 0; k < ALT_K; ++k) { hT[k] = dLt[k]; hS[k] = dLs[k]; }

    int64_t mu = INT64_MAX;
    for (int32_t k = 0; k < ALT_K; ++k) {
        int32_t ds = hS[k], dt = hT[k];
        if (ds != INT16_MAX && dt != INT16_MAX) {
            int64_t cand = (int64_t)ds + (int64_t)dt;
            if (cand < mu) mu = cand;
        }
    }

    auto h_lm = [&](int32_t v, const int16_t* T) -> int64_t {
        return alt_h_int16(&alt.dL16[(size_t)v * ALT_K], T);
    };

    int64_t h_s_t = h_lm(s, hT);
    if (mu != INT64_MAX && h_s_t >= mu) return mu;
    st.bq_f.clear_from(h_s_t);
    st.bq_b.clear_from(h_s_t);

    st.nf[(size_t)s].dist = 0; st.nf[(size_t)s].gen = cg;
    st.bq_f.push(h_s_t, s);
    st.nb[(size_t)t].dist = 0; st.nb[(size_t)t].gen = cg;
    st.bq_b.push(h_s_t, t);

    while (!st.bq_f.empty() && !st.bq_b.empty()) {
        int64_t kf = st.bq_f.top_key();
        int64_t kb = st.bq_b.top_key();
        if (mu != INT64_MAX && kf + kb >= 2 * mu) break;

        if (kf <= kb) {
            int32_t u = st.bq_f.pop();
            if (st.nf[(size_t)u].set == cg) continue;
            st.nf[(size_t)u].set = cg;
            int64_t du = st.nf[(size_t)u].dist;

            if (st.nb[(size_t)u].gen == cg) {
                int64_t cand = du + st.nb[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                if (st.nf[(size_t)v].set == cg) continue;
                int64_t nd = du + (int64_t)gw[i];
                if (mu != INT64_MAX && nd >= mu) continue;
                if (st.nf[(size_t)v].gen != cg || nd < st.nf[(size_t)v].dist) {
                    st.nf[(size_t)v].dist = nd;
                    st.nf[(size_t)v].gen = cg;
                    int64_t key = 2 * nd + h_lm(v, hT) - h_lm(v, hS);
                    st.bq_f.push(key, v);
                }
            }
        } else {
            int32_t u = st.bq_b.pop();
            if (st.nb[(size_t)u].set == cg) continue;
            st.nb[(size_t)u].set = cg;
            int64_t du = st.nb[(size_t)u].dist;

            if (st.nf[(size_t)u].gen == cg) {
                int64_t cand = du + st.nf[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                if (st.nb[(size_t)v].set == cg) continue;
                int64_t nd = du + (int64_t)gw[i];
                if (mu != INT64_MAX && nd >= mu) continue;
                if (st.nb[(size_t)v].gen != cg || nd < st.nb[(size_t)v].dist) {
                    st.nb[(size_t)v].dist = nd;
                    st.nb[(size_t)v].gen = cg;
                    int64_t key = 2 * nd + h_lm(v, hS) - h_lm(v, hT);
                    st.bq_b.push(key, v);
                }
            }
        }
    }
    return (mu == INT64_MAX) ? -1 : mu;
}

// Bidirectional ALT A* with BitmapBucketQueue (Phase G13c).  Identical algorithm
// to alt_query_bucket above -- balanced potential, scaled-space Pohl stop rule,
// mu cross-update, lazy-deletion staleness -- only the priority queue type
// changes.  Used on cluster, where max_w (~250K) makes the bidir reduced-cost
// spread up to 4*max_w ~= 1M.  A plain BucketQueue here would scan up to 1M
// empty buckets per top_key call; the bitmap reduces that to scanning
// modulo/64 = ~16K uint64 words (in L2).
static int64_t alt_query_bbq(const CSR& g, const ALTIndex& alt, BidirState& st,
                             int32_t s, int32_t t) {
    if (s == t) return 0;
    st.next_gen();
    uint32_t cg = st.cur_gen;

    const int32_t* gh = g.head.data();
    const int32_t* gt = g.to.data();
    const int32_t* gw = g.w.data();

    int32_t hT[ALT_K], hS[ALT_K];
    const int32_t* dLt = &alt.dL[(size_t)t * ALT_K];
    const int32_t* dLs = &alt.dL[(size_t)s * ALT_K];
    for (int32_t k = 0; k < ALT_K; ++k) { hT[k] = dLt[k]; hS[k] = dLs[k]; }

    auto h_lm = [&](int32_t v, const int32_t* T) -> int64_t {
        return alt_h(&alt.dL[(size_t)v * ALT_K], T);
    };

    int64_t h_s_t = h_lm(s, hT);
    st.bbq_f.clear_from(h_s_t);
    st.bbq_b.clear_from(h_s_t);

    st.nf[(size_t)s].dist = 0; st.nf[(size_t)s].gen = cg;
    st.bbq_f.push(h_s_t, s);
    st.nb[(size_t)t].dist = 0; st.nb[(size_t)t].gen = cg;
    st.bbq_b.push(h_s_t, t);

    int64_t mu = INT64_MAX;

    while (!st.bbq_f.empty() && !st.bbq_b.empty()) {
        int64_t kf = st.bbq_f.top_key();
        int64_t kb = st.bbq_b.top_key();
        if (mu != INT64_MAX && kf + kb >= 2 * mu) break;

        if (kf <= kb) {
            int32_t u = st.bbq_f.pop();
            if (st.nf[(size_t)u].set == cg) continue;
            st.nf[(size_t)u].set = cg;
            int64_t du = st.nf[(size_t)u].dist;

            if (st.nb[(size_t)u].gen == cg) {
                int64_t cand = du + st.nb[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                if (st.nf[(size_t)v].set == cg) continue;
                int64_t nd = du + (int64_t)gw[i];
                if (st.nf[(size_t)v].gen != cg || nd < st.nf[(size_t)v].dist) {
                    st.nf[(size_t)v].dist = nd;
                    st.nf[(size_t)v].gen = cg;
                    int64_t key = 2 * nd + h_lm(v, hT) - h_lm(v, hS);
                    st.bbq_f.push(key, v);
                }
            }
        } else {
            int32_t u = st.bbq_b.pop();
            if (st.nb[(size_t)u].set == cg) continue;
            st.nb[(size_t)u].set = cg;
            int64_t du = st.nb[(size_t)u].dist;

            if (st.nf[(size_t)u].gen == cg) {
                int64_t cand = du + st.nf[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }

            int32_t es = gh[u], ee = gh[u + 1];
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = gt[i];
                if (st.nb[(size_t)v].set == cg) continue;
                int64_t nd = du + (int64_t)gw[i];
                if (st.nb[(size_t)v].gen != cg || nd < st.nb[(size_t)v].dist) {
                    st.nb[(size_t)v].dist = nd;
                    st.nb[(size_t)v].gen = cg;
                    int64_t key = 2 * nd + h_lm(v, hS) - h_lm(v, hT);
                    st.bbq_b.push(key, v);
                }
            }
        }
    }
    return (mu == INT64_MAX) ? -1 : mu;
}

// ---------------------------------------------------------------------------
// Contraction Hierarchies (CH) -- disabled.
//
// Kept under `#if 0` so the implementation survives in the tree but does not
// compile.  road/grid/cluster preprocessing dominated wall-clock budget for
// CH; the active path is now ALT.  Re-enable by flipping the guard to `#if 1`
// (and bring back the CH dispatch in main()).
// ---------------------------------------------------------------------------

#if 0
// ---------------------------------------------------------------------------
// Contraction Hierarchies (CH)
//
// Construction:
//   * Mutable adjacency `vector<vector<DynEdge>>` seeded from CSR
//   * Iteratively contract nodes in increasing-importance order
//   * Importance proxy = active_deg + 2 * contracted_neighbours
//   * For each pair (u_i, u_j) of v's still-active neighbours, run a bounded
//     witness Dijkstra from u_i (ignoring v and contracted nodes). If no
//     witness path of cost <= w(u_i,v)+w(v,u_j) exists, add a shortcut
//   * When v is contracted, drop every edge pointing to v from each neighbour's
//     adjacency. After all V contractions, every remaining entry in adj[u]
//     points to a node v with rank[v] > rank[u] (i.e. the up-graph)
//
// Query:
//   * Bidirectional Dijkstra on the up-graph only
//   * Stall-on-demand: before settling u, check if any up-neighbour x has
//     dist[x] + w(u,x) < d. If so, u's tentative distance was a detour --
//     skip it
// ---------------------------------------------------------------------------

struct DynEdge { int32_t to; int32_t w; };

struct CHIndex {
    int32_t          V = 0;
    vector<int32_t>  rank;
    vector<int32_t>  up_head;   // size V+1
    vector<int32_t>  up_to;
    vector<int32_t>  up_w;
};

struct CHBuilder {
    int32_t                  V = 0;
    vector<vector<DynEdge>>  adj;
    vector<uint8_t>          contracted;
    vector<int32_t>          rank;
    vector<int32_t>          contracted_nbr;
    int32_t                  next_rank = 0;

    vector<int64_t>   w_dist;
    vector<int32_t>   w_depth;
    vector<uint32_t>  w_gen;
    uint32_t          cur_gen = 0;
    Heap4             heap;

    int32_t settled_limit = 200;
    int32_t hop_limit     = 5;

    vector<DynEdge> active_;  // scratch reused across contract() calls

    void init(const CSR& g) {
        V = g.V;
        adj.assign((size_t)V, {});
        contracted.assign((size_t)V, 0);
        rank.assign((size_t)V, -1);
        contracted_nbr.assign((size_t)V, 0);
        w_dist .assign((size_t)V, 0);
        w_depth.assign((size_t)V, 0);
        w_gen  .assign((size_t)V, 0);
        next_rank = 0;
        cur_gen = 0;
        for (int32_t u = 0; u < V; ++u) {
            int32_t s = g.head[u], e = g.head[u + 1];
            adj[(size_t)u].reserve((size_t)(e - s));
            for (int32_t i = s; i < e; ++i) {
                adj[(size_t)u].push_back({g.to[i], g.w[i]});
            }
        }
    }

    // Bounded witness search from `src`, ignoring `skip` and contracted nodes.
    // Stops when: d >= threshold, settled_limit nodes settled, or queue empty.
    // hop_limit caps how far from src the search will relax (critical for
    // high-diameter graphs like road networks).
    void witness_search(int32_t src, int32_t skip, int64_t threshold) {
        ++cur_gen;
        if (cur_gen == 0) {
            std::memset(w_gen.data(), 0, w_gen.size() * sizeof(uint32_t));
            cur_gen = 1;
        }
        heap.clear();
        w_dist [(size_t)src] = 0;
        w_depth[(size_t)src] = 0;
        w_gen  [(size_t)src] = cur_gen;
        heap.push(0, src);
        int32_t settled = 0;
        while (!heap.empty()) {
            HeapItem it = heap.pop();
            int32_t u = it.val;
            int64_t d = it.key;
            if (w_gen[(size_t)u] != cur_gen || d != w_dist[(size_t)u]) continue;
            if (d >= threshold) return;
            if (++settled > settled_limit) return;
            int32_t depth_u = w_depth[(size_t)u];
            if (depth_u >= hop_limit) continue;
            for (const DynEdge& e : adj[(size_t)u]) {
                int32_t v = e.to;
                if (contracted[(size_t)v]) continue;
                if (v == skip) continue;
                int64_t nd = d + (int64_t)e.w;
                if (nd >= threshold) continue;
                if (w_gen[(size_t)v] != cur_gen || nd < w_dist[(size_t)v]) {
                    w_dist [(size_t)v] = nd;
                    w_depth[(size_t)v] = depth_u + 1;
                    w_gen  [(size_t)v] = cur_gen;
                    heap.push(nd, v);
                }
            }
        }
    }

    void collect_active(int32_t v) {
        active_.clear();
        const auto& nv = adj[(size_t)v];
        active_.reserve(nv.size());
        for (const DynEdge& e : nv) {
            // Filter contracted and self-loops (defensive vs input self-loops
            // and post-shortcut artifacts that would produce phantom edges).
            if (!contracted[(size_t)e.to] && e.to != v) active_.push_back(e);
        }
    }

    // Returns number of shortcuts. If commit, also installs shortcuts in adj
    // and marks v contracted (with rank). Removes edges pointing to v from
    // each surviving neighbour's adjacency to keep adj lists thinned out.
    int32_t contract(int32_t v, bool commit) {
        collect_active(v);
        vector<DynEdge> nv = active_;  // snapshot: adding shortcuts may realloc
        int32_t n = (int32_t)nv.size();

        auto finalize_neighbours = [&](){
            for (const DynEdge& e : nv) {
                int32_t u = e.to;
                contracted_nbr[(size_t)u]++;
                auto& au = adj[(size_t)u];
                size_t wpos = 0;
                for (size_t i = 0; i < au.size(); ++i) {
                    if (au[i].to != v) {
                        if (wpos != i) au[wpos] = au[i];
                        ++wpos;
                    }
                }
                au.resize(wpos);
            }
        };

        if (n < 2) {
            if (commit) {
                contracted[(size_t)v] = 1;
                rank[(size_t)v] = next_rank++;
                finalize_neighbours();
            }
            return 0;
        }

        int32_t shortcuts = 0;
        struct Sc { int32_t a, b, w; };
        static thread_local vector<Sc> sc_buf;
        if (commit) sc_buf.clear();

        for (int32_t i = 0; i < n; ++i) {
            int32_t u_i  = nv[(size_t)i].to;
            int32_t w_iv = nv[(size_t)i].w;
            int64_t max_thresh = 0;
            for (int32_t j = 0; j < n; ++j) {
                if (j == i) continue;
                int64_t cand = (int64_t)w_iv + nv[(size_t)j].w;
                if (cand > max_thresh) max_thresh = cand;
            }
            witness_search(u_i, v, max_thresh + 1);
            for (int32_t j = i + 1; j < n; ++j) {
                int32_t u_j  = nv[(size_t)j].to;
                int32_t w_jv = nv[(size_t)j].w;
                int64_t cand = (int64_t)w_iv + w_jv;
                int64_t wd = (w_gen[(size_t)u_j] == cur_gen)
                           ? w_dist[(size_t)u_j] : INT64_MAX;
                if (wd > cand) {
                    ++shortcuts;
                    if (commit && cand <= INT32_MAX) {
                        sc_buf.push_back({u_i, u_j, (int32_t)cand});
                    }
                }
            }
        }

        if (commit) {
            // Dedup-on-insert: if adj[a] already has an edge to b, update its
            // weight to min(existing, new) instead of appending. This prevents
            // O(n^2) blowup from duplicate shortcuts accumulating in adj lists
            // (catastrophic on high-diameter sparse graphs like road network,
            // where witness_search misses long-path witnesses under hop_limit).
            for (const Sc& s : sc_buf) {
                if (s.a == s.b) continue;          // defensive: skip self-loops
                {
                    auto& aa = adj[(size_t)s.a];
                    bool found = false;
                    for (auto& e : aa) {
                        if (e.to == s.b) {
                            if (s.w < e.w) e.w = s.w;
                            found = true;
                            break;
                        }
                    }
                    if (!found) aa.push_back({s.b, s.w});
                }
                {
                    auto& bb = adj[(size_t)s.b];
                    bool found = false;
                    for (auto& e : bb) {
                        if (e.to == s.a) {
                            if (s.w < e.w) e.w = s.w;
                            found = true;
                            break;
                        }
                    }
                    if (!found) bb.push_back({s.a, s.w});
                }
            }
            contracted[(size_t)v] = 1;
            rank[(size_t)v] = next_rank++;
            finalize_neighbours();
        }
        return shortcuts;
    }

    inline int32_t cheap_imp(int32_t v) {
        int32_t d = 0;
        for (const DynEdge& e : adj[(size_t)v]) {
            if (!contracted[(size_t)e.to]) ++d;
        }
        return d + 2 * contracted_nbr[(size_t)v];
    }

    void build(CHIndex& out) {
        using PQItem = std::pair<int32_t, int32_t>;
        std::priority_queue<PQItem, vector<PQItem>, std::greater<PQItem>> pq;
        vector<int32_t> key((size_t)V);
        for (int32_t v = 0; v < V; ++v) {
            key[(size_t)v] = cheap_imp(v);
            pq.push({key[(size_t)v], v});
        }

        int32_t done = 0;
        while (!pq.empty()) {
            auto [k, v] = pq.top(); pq.pop();
            if (contracted[(size_t)v]) continue;
            if (k != key[(size_t)v]) continue;
            int32_t now_k = cheap_imp(v);
            if (now_k != k) {
                key[(size_t)v] = now_k;
                pq.push({now_k, v});
                continue;
            }
            contract(v, true);
            ++done;
            if ((done & 0xFFFF) == 0) {
                std::fprintf(stderr, "[ch] %d / %d ...\n", done, V);
            }
            for (const DynEdge& e : active_) {
                if (contracted[(size_t)e.to]) continue;
                int32_t nk = cheap_imp(e.to);
                // Only push if key decreased. Increases get caught by the
                // inline recheck in the main loop, avoiding redundant PQ
                // entries that would just be popped and re-pushed.
                if (nk < key[(size_t)e.to]) {
                    key[(size_t)e.to] = nk;
                    pq.push({nk, e.to});
                }
            }
        }

        // Build up-graph CSR from surviving adj entries. After contraction
        // every remaining edge from u points to a later-contracted node v
        // (rank[v] > rank[u]).
        out.V = V;
        out.rank = rank;
        out.up_head.assign((size_t)V + 1, 0);

        auto dedup = [](vector<DynEdge>& v) {
            if (v.size() < 2) return;
            std::sort(v.begin(), v.end(), [](const DynEdge& a, const DynEdge& b){
                return a.to < b.to || (a.to == b.to && a.w < b.w);
            });
            size_t wpos = 0;
            for (size_t i = 0; i < v.size(); ++i) {
                if (i == 0 || v[i].to != v[wpos - 1].to) {
                    v[wpos++] = v[i];
                }
            }
            v.resize(wpos);
        };
        for (int32_t u = 0; u < V; ++u) {
            dedup(adj[(size_t)u]);
            out.up_head[(size_t)u + 1] = (int32_t)adj[(size_t)u].size();
        }
        for (int32_t i = 1; i <= V; ++i) {
            out.up_head[(size_t)i] += out.up_head[(size_t)i - 1];
        }
        int32_t total_up = out.up_head[(size_t)V];
        out.up_to.assign((size_t)total_up, 0);
        out.up_w .assign((size_t)total_up, 0);
        for (int32_t u = 0; u < V; ++u) {
            int32_t p = out.up_head[(size_t)u];
            for (const DynEdge& e : adj[(size_t)u]) {
                out.up_to[(size_t)p] = e.to;
                out.up_w [(size_t)p] = e.w;
                ++p;
            }
        }
        adj.clear(); adj.shrink_to_fit();
        std::fprintf(stderr, "[ch] contracted %d / %d, up_E = %d\n",
                     done, V, total_up);
    }
};

// CH bidirectional query with stall-on-demand. Reuses BidirState; reads only
// the up-graph (CHIndex::up_*).
static int64_t ch_query(const CHIndex& ch, BidirState& st,
                        int32_t s, int32_t t) {
    if (s == t) return 0;
    st.next_gen();
    uint32_t cg = st.cur_gen;

    st.heap_f.clear();
    st.heap_b.clear();
    st.nf[(size_t)s].dist = 0; st.nf[(size_t)s].gen = cg; st.heap_f.push(0, s);
    st.nb[(size_t)t].dist = 0; st.nb[(size_t)t].gen = cg; st.heap_b.push(0, t);

    int64_t mu = INT64_MAX;

    while (!st.heap_f.empty() || !st.heap_b.empty()) {
        int64_t kf = st.heap_f.empty() ? INT64_MAX : st.heap_f.top().key;
        int64_t kb = st.heap_b.empty() ? INT64_MAX : st.heap_b.top().key;
        int64_t kmin = (kf < kb) ? kf : kb;
        if (kmin >= mu) break;

        if (kf <= kb) {
            HeapItem it = st.heap_f.pop();
            int32_t u = it.val; int64_t d = it.key;
            if (st.nf[(size_t)u].gen != cg || d != st.nf[(size_t)u].dist) continue;
            if (st.nf[(size_t)u].set == cg) continue;

            int32_t es = ch.up_head[(size_t)u], ee = ch.up_head[(size_t)u + 1];
            bool stalled = false;
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = ch.up_to[(size_t)i];
                if (st.nf[(size_t)v].gen != cg) continue;
                if (st.nf[(size_t)v].dist + ch.up_w[(size_t)i] < d) {
                    stalled = true; break;
                }
            }
            if (stalled) continue;
            st.nf[(size_t)u].set = cg;

            if (st.nb[(size_t)u].gen == cg) {
                int64_t cand = d + st.nb[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = ch.up_to[(size_t)i];
                int64_t nd = d + ch.up_w[(size_t)i];
                if (st.nf[(size_t)v].gen != cg || nd < st.nf[(size_t)v].dist) {
                    st.nf[(size_t)v].dist = nd;
                    st.nf[(size_t)v].gen = cg;
                    st.heap_f.push(nd, v);
                }
            }
        } else {
            HeapItem it = st.heap_b.pop();
            int32_t u = it.val; int64_t d = it.key;
            if (st.nb[(size_t)u].gen != cg || d != st.nb[(size_t)u].dist) continue;
            if (st.nb[(size_t)u].set == cg) continue;

            int32_t es = ch.up_head[(size_t)u], ee = ch.up_head[(size_t)u + 1];
            bool stalled = false;
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = ch.up_to[(size_t)i];
                if (st.nb[(size_t)v].gen != cg) continue;
                if (st.nb[(size_t)v].dist + ch.up_w[(size_t)i] < d) {
                    stalled = true; break;
                }
            }
            if (stalled) continue;
            st.nb[(size_t)u].set = cg;

            if (st.nf[(size_t)u].gen == cg) {
                int64_t cand = d + st.nf[(size_t)u].dist;
                if (cand < mu) mu = cand;
            }
            for (int32_t i = es; i < ee; ++i) {
                int32_t v = ch.up_to[(size_t)i];
                int64_t nd = d + ch.up_w[(size_t)i];
                if (st.nb[(size_t)v].gen != cg || nd < st.nb[(size_t)v].dist) {
                    st.nb[(size_t)v].dist = nd;
                    st.nb[(size_t)v].gen = cg;
                    st.heap_b.push(nd, v);
                }
            }
        }
    }
    return (mu == INT64_MAX) ? -1 : mu;
}

#endif  // CH disabled

// ---------------------------------------------------------------------------
// Query loops
// ---------------------------------------------------------------------------

static void run_queries_bidir(const CSR& g, const char* qpath, const char* opath) {
    FileIn in;
    if (!in.open_file(qpath)) {
        std::fprintf(stderr, "cannot open query file: %s\n", qpath);
        std::exit(1);
    }
    Parser ps(in.data, in.size);
    int32_t Q = ps.i32();

    OutBuf ob;
    if (!ob.open_file(opath, 1u << 20)) {
        std::fprintf(stderr, "cannot open output file: %s\n", opath);
        std::exit(1);
    }

    BidirState st;
    st.init(g.V);

    // Cyclic bucket queue (Dial's) beats the 8-ary heap when edge weights are
    // small integers (social: max edge weight 100).  Gate on a modest cap so
    // a pathological large-weight HAS_COORDS=0 graph falls back to the heap
    // instead of allocating a huge bucket array.
    constexpr int32_t BUCKET_MAX_W = 4096;
    bool use_bucket = (g.max_w <= BUCKET_MAX_W);
    if (use_bucket) st.init_buckets(g.max_w);

    if (use_bucket) {
        for (int32_t i = 0; i < Q; ++i) {
            int32_t s = ps.i32();
            int32_t t = ps.i32();
            int64_t d = bidir_dijkstra_bucket(g, st, s, t);
            ob.put_int64_nl(d);
        }
    } else {
        for (int32_t i = 0; i < Q; ++i) {
            int32_t s = ps.i32();
            int32_t t = ps.i32();
            int64_t d = bidir_dijkstra(g, st, s, t);
            ob.put_int64_nl(d);
        }
    }
}

static void run_queries_alt(const CSR& g, const ALTIndex& alt,
                            const char* qpath, const char* opath) {
    FileIn in;
    if (!in.open_file(qpath)) {
        std::fprintf(stderr, "cannot open query file: %s\n", qpath);
        std::exit(1);
    }
    Parser ps(in.data, in.size);
    int32_t Q = ps.i32();

    // Pre-parse all (s, t) pairs so we can both probe and bulk-run them.
    vector<int32_t> ss((size_t)Q), tt((size_t)Q);
    for (int32_t i = 0; i < Q; ++i) {
        ss[(size_t)i] = ps.i32();
        tt[(size_t)i] = ps.i32();
    }

    OutBuf ob;
    if (!ob.open_file(opath, 1u << 20)) {
        std::fprintf(stderr, "cannot open output file: %s\n", opath);
        std::exit(1);
    }

    BidirState st;
    st.init(g.V);

    // Phase G8: when the max edge weight is tiny the bidirectional ALT A* can
    // run on cyclic bucket queues instead of the 8-ary heap.  On the scaled
    // reduced costs every live key sits within [current, current + 4*max_w],
    // so a bucket count > 4*max_w gives Dial's no-collision guarantee --
    // init_buckets(4*max_w) sizes them for that.  Only the grid instances
    // qualify (max_w=10); road/cluster (huge max_w and/or huge query
    // distances) keep the heap.
    constexpr int32_t ALT_BUCKET_MAX_W = 256;
    bool bidir_use_bucket = (g.max_w <= ALT_BUCKET_MAX_W);
    if (bidir_use_bucket) st.init_buckets(4 * g.max_w);

    // Phase G13b/c: bitmap-augmented bucket queue for the ALT search.  Unidir
    // f-key spread is bounded by 2*max_w, bidir scaled-key spread by 4*max_w
    // (consistency of h).  Unlike G8's plain BucketQueue, the bitmap top_key
    // keeps the empty-bucket scan cost bounded (O(modulo/64) per call) even
    // when the final f reaches millions -- usable on road (G13b unidir).
    // Grid keeps its G8 BucketQueue path (its modulo is small enough that
    // the bitmap is pure overhead).
    //
    // Cluster (max_w ~250K) doesn't qualify: a 4*max_w-sized bucket array
    // becomes 24 MB of vector headers, larger than L2, and the random-access
    // cache misses make bidir-bbq ~3.5x slower than bidir-heap (measured).
    // The gate keeps bbq for max_w <= 16384 -- catches road but not cluster.
    //
    // bbq_f and bbq_b are sized for the bidir bound (4*max_w) so either
    // run_unidir or run_bidir can use them safely.
    constexpr int32_t ALT_BBQ_MAX_W = 16384;
    bool use_bbq = !bidir_use_bucket && (g.max_w <= ALT_BBQ_MAX_W);
    if (use_bbq) st.init_bm_buckets(4 * g.max_w);

    auto run_unidir = [&](int32_t s, int32_t t) -> int64_t {
        return use_bbq ? alt_unidir_query_bbq(g, alt, st, s, t)
                       : alt_unidir_query    (g, alt, st, s, t);
    };
    auto run_bidir = [&](int32_t s, int32_t t) -> int64_t {
        if (bidir_use_bucket) {
            // Phase N10: grid uses dL16 (int16) for tighter cache footprint.
            // alt.use_int16 is set by alt_build's post-conversion when every
            // landmark distance fits int16 (always true for contest grid).
            if (alt.use_int16) {
#if SOLVER_SOCIAL_UPPER
                if (!g.HAS_COORDS) return alt_query_bucket_int16_social(g, alt, st, s, t);
#endif
                return alt_query_bucket_int16(g, alt, st, s, t);
            }
            return alt_query_bucket(g, alt, st, s, t);
        }
        if (use_bbq)          return alt_query_bbq   (g, alt, st, s, t);
        return                       alt_query       (g, alt, st, s, t);
    };

    bool use_bidir;
#if SOLVER_FORCE_ALT_MODE == 1
    use_bidir = false;
#elif SOLVER_FORCE_ALT_MODE == 2
    use_bidir = true;
#elif SOLVER_FORCE_ALT_MODE == 3
    use_bidir = (g.max_w <= ALT_BUCKET_MAX_W) || (g.max_w > 25000);
#else
    // ---- Dispatcher probe ------------------------------------------------
    // Run the first PROBE queries with each mode, pick the faster mode for
    // the bulk run.  Probe results are discarded -- the chosen mode is re-run
    // for all Q queries below.
    int32_t probe = Q < 30 ? Q : 30;
    using clk = std::chrono::steady_clock;

    auto t0 = clk::now();
    for (int32_t i = 0; i < probe; ++i)
        (void)run_unidir(ss[(size_t)i], tt[(size_t)i]);
    auto t_uni = clk::now() - t0;

    t0 = clk::now();
    for (int32_t i = 0; i < probe; ++i)
        (void)run_bidir(ss[(size_t)i], tt[(size_t)i]);
    auto t_bi = clk::now() - t0;

    use_bidir = t_bi <= t_uni;
#endif

    // ---- Bulk run --------------------------------------------------------
    if (use_bidir) {
        for (int32_t i = 0; i < Q; ++i) {
            int64_t d = run_bidir(ss[(size_t)i], tt[(size_t)i]);
            ob.put_int64_nl(d);
        }
    } else {
        for (int32_t i = 0; i < Q; ++i) {
            int64_t d = run_unidir(ss[(size_t)i], tt[(size_t)i]);
            ob.put_int64_nl(d);
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::fprintf(stderr, "usage: %s <graph_file> <query_file> <output_file>\n", argv[0]);
        return 1;
    }
    CSR g;
    build_csr(argv[1], g);

    if (g.HAS_COORDS || SOLVER_ALT_SOCIAL) {
        // Exact ALT (A* with landmarks).  Landmarks are valid lower bounds
        // for social graphs too; coordinates are not required.
        ALTIndex alt;
        alt_build(g, alt);
        run_queries_alt(g, alt, argv[2], argv[3]);
    } else {
        // Optional fallback for experiments that disable social ALT.
        run_queries_bidir(g, argv[2], argv[3]);
    }
    return 0;
}
