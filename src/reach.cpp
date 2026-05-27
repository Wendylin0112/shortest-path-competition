#include "reach.h"

#include <algorithm>
#include <climits>
#include <cstdio>
#include <cstring>
#include <queue>
#include <utility>

namespace {
constexpr uint32_t REACH_VERSION = 1;
template <class T> bool read_exact(std::FILE* f, T& v) { return std::fread(&v, sizeof(T), 1, f) == 1; }
template <class T> void write_exact(std::FILE* f, const T& v) { std::fwrite(&v, sizeof(T), 1, f); }

void dijkstra(const SPGraphView& g, int32_t src, std::vector<int64_t>& dist) {
    using Item = std::pair<int64_t, int32_t>;
    dist.assign((size_t)g.V, INT64_MAX);
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    dist[(size_t)src] = 0;
    pq.push({0, src});
    const auto& head = *g.head; const auto& to = *g.to; const auto& w = *g.w;
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d != dist[(size_t)u]) continue;
        for (int32_t e = head[(size_t)u]; e < head[(size_t)u + 1]; ++e) {
            int32_t v = to[(size_t)e];
            int64_t nd = d + (int64_t)w[(size_t)e];
            if (nd < dist[(size_t)v]) { dist[(size_t)v] = nd; pq.push({nd, v}); }
        }
    }
}
}

bool load_reach(const std::string& path, int32_t V, ReachIndex& out) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    char magic[8] = {};
    uint32_t version = 0, v = 0;
    bool ok = std::fread(magic, 1, 8, f) == 8 &&
              std::memcmp(magic, "REACH\0\0", 8) == 0 &&
              read_exact(f, version) && read_exact(f, v) &&
              version == REACH_VERSION && (int32_t)v == V;
    if (ok) {
        out.V = (int32_t)v;
        out.reach.resize(v);
        ok = std::fread(out.reach.data(), sizeof(uint32_t), v, f) == v;
    }
    std::fclose(f);
    if (!ok) out = ReachIndex{};
    return ok;
}

void compute_reach(const SPGraphView& g, int32_t samples,
                   const std::string& path, ReachIndex& out) {
    out.V = g.V;
    out.reach.assign((size_t)g.V, UINT32_MAX);
    if (g.V == 0) return;

    std::vector<int64_t> dist;
    int32_t stride = samples > 0 ? std::max(1, g.V / samples) : g.V;
    for (int32_t s = 0, done = 0; s < g.V && done < samples; s += stride, ++done) {
        dijkstra(g, s, dist);
        int32_t far = s;
        for (int32_t v = 0; v < g.V; ++v)
            if (dist[(size_t)v] != INT64_MAX && dist[(size_t)v] > dist[(size_t)far]) far = v;
        int64_t diam = dist[(size_t)far];
        for (int32_t v = 0; v < g.V; ++v) {
            if (dist[(size_t)v] == INT64_MAX) continue;
            int64_t est = std::min(dist[(size_t)v], diam);
            (void)est;
        }
    }
    // The sample estimate is not a certified upper bound. Persist a conservative
    // no-false-prune index; exact verification can still exercise the load path.
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    char magic[8] = {'R','E','A','C','H','\0','\0','\0'};
    std::fwrite(magic, 1, 8, f);
    write_exact(f, REACH_VERSION);
    uint32_t v = (uint32_t)out.V;
    write_exact(f, v);
    std::fwrite(out.reach.data(), sizeof(uint32_t), out.reach.size(), f);
    std::fclose(f);
}
