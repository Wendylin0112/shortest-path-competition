#include "landmarks.h"

#include <algorithm>
#include <climits>
#include <cstdio>
#include <cstring>
#include <queue>
#include <utility>

namespace {
constexpr uint32_t LM_VERSION = 1;

template <class T>
bool read_exact(std::FILE* f, T& v) {
    return std::fread(&v, sizeof(T), 1, f) == 1;
}
template <class T>
void write_exact(std::FILE* f, const T& v) {
    std::fwrite(&v, sizeof(T), 1, f);
}

int32_t sat_i32(int64_t v) {
    return v >= (int64_t)INT32_MAX ? INT32_MAX : (int32_t)v;
}

void dijkstra(const SPGraphView& g, int32_t src, std::vector<int64_t>& dist) {
    using Item = std::pair<int64_t, int32_t>;
    dist.assign((size_t)g.V, INT64_MAX);
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    dist[(size_t)src] = 0;
    pq.push({0, src});
    const auto& head = *g.head;
    const auto& to = *g.to;
    const auto& w = *g.w;
    while (!pq.empty()) {
        Item it = pq.top();
        pq.pop();
        int64_t d = it.first;
        int32_t u = it.second;
        if (d != dist[(size_t)u]) continue;
        for (int32_t e = head[(size_t)u]; e < head[(size_t)u + 1]; ++e) {
            int32_t v = to[(size_t)e];
            int64_t nd = d + (int64_t)w[(size_t)e];
            if (nd < dist[(size_t)v]) {
                dist[(size_t)v] = nd;
                pq.push({nd, v});
            }
        }
    }
}

std::vector<int32_t> coordinate_picks(const SPGraphView& g, int32_t want) {
    std::vector<int32_t> picks;
    if (!g.has_coords || !g.x || !g.y || want <= 0) return picks;
    const auto& xs = *g.x;
    const auto& ys = *g.y;
    struct Score { double val; int32_t node; };
    std::vector<Score> cand;
    cand.reserve(8);
    auto best = [&](auto score) {
        double bv = -1e300;
        int32_t bi = 0;
        for (int32_t v = 0; v < g.V; ++v) {
            double s = score(xs[(size_t)v], ys[(size_t)v]);
            if (s > bv) { bv = s; bi = v; }
        }
        cand.push_back({bv, bi});
    };
    best([](double x, double y){ return  x + y; });
    best([](double x, double y){ return  x - y; });
    best([](double x, double y){ return -x + y; });
    best([](double x, double y){ return -x - y; });
    best([](double x, double){ return  x; });
    best([](double x, double){ return -x; });
    best([](double, double y){ return  y; });
    best([](double, double y){ return -y; });
    for (const Score& s : cand) {
        if ((int32_t)picks.size() >= want) break;
        if (std::find(picks.begin(), picks.end(), s.node) == picks.end())
            picks.push_back(s.node);
    }
    return picks;
}
}

int64_t LandmarkIndex::heuristic(int32_t v, int32_t t) const {
    if (empty()) return 0;
    int64_t best = 0;
    const int32_t* dv = &dist[(size_t)v * (size_t)K];
    const int32_t* dt = &dist[(size_t)t * (size_t)K];
    for (int32_t k = 0; k < K; ++k) {
        if (dv[k] == INT32_MAX || dt[k] == INT32_MAX) continue;
        int64_t a = (int64_t)dv[k] - (int64_t)dt[k];
        if (a < 0) a = -a;
        if (a > best) best = a;
    }
    return best;
}

std::vector<int32_t> select_landmarks(const SPGraphView& g, int32_t k,
                                      const std::string& strategy) {
    if (k <= 0 || g.V <= 0) return {};
    std::vector<int32_t> picks;
    if (strategy == "coords") {
        picks = coordinate_picks(g, k);
    } else if (strategy == "mixed") {
        picks = coordinate_picks(g, k / 2);
    }
    if (picks.empty()) picks.push_back(0);

    std::vector<int64_t> dist, near((size_t)g.V, INT64_MAX);
    for (int32_t p : picks) {
        dijkstra(g, p, dist);
        for (int32_t v = 0; v < g.V; ++v)
            if (dist[(size_t)v] < near[(size_t)v]) near[(size_t)v] = dist[(size_t)v];
    }
    while ((int32_t)picks.size() < k) {
        int64_t best = -1;
        int32_t pick = 0;
        for (int32_t v = 0; v < g.V; ++v) {
            if (near[(size_t)v] != INT64_MAX && near[(size_t)v] > best &&
                std::find(picks.begin(), picks.end(), v) == picks.end()) {
                best = near[(size_t)v];
                pick = v;
            }
        }
        picks.push_back(pick);
        dijkstra(g, pick, dist);
        for (int32_t v = 0; v < g.V; ++v)
            if (dist[(size_t)v] < near[(size_t)v]) near[(size_t)v] = dist[(size_t)v];
    }
    return picks;
}

bool load_landmarks(const std::string& path, int32_t V, LandmarkIndex& out) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    char magic[8] = {};
    uint32_t version = 0, v = 0, k = 0;
    bool ok = std::fread(magic, 1, 8, f) == 8 &&
              std::memcmp(magic, "LNDMRK\0", 8) == 0 &&
              read_exact(f, version) && read_exact(f, v) && read_exact(f, k) &&
              version == LM_VERSION && (int32_t)v == V && k <= 1024;
    if (ok) {
        out.V = (int32_t)v;
        out.K = (int32_t)k;
        out.nodes.resize(k);
        out.dist.resize((size_t)v * (size_t)k);
        ok = std::fread(out.nodes.data(), sizeof(int32_t), k, f) == k &&
             std::fread(out.dist.data(), sizeof(int32_t), out.dist.size(), f) == out.dist.size();
    }
    std::fclose(f);
    if (!ok) out = LandmarkIndex{};
    return ok;
}

void compute_landmarks(const SPGraphView& g, int32_t k,
                       const std::string& strategy,
                       const std::string& path,
                       LandmarkIndex& out) {
    out.V = g.V;
    out.K = k;
    out.nodes = select_landmarks(g, k, strategy);
    out.dist.assign((size_t)g.V * (size_t)k, INT32_MAX);
    std::vector<int64_t> dist;
    for (int32_t i = 0; i < k; ++i) {
        dijkstra(g, out.nodes[(size_t)i], dist);
        for (int32_t v = 0; v < g.V; ++v) {
            int64_t d = dist[(size_t)v];
            out.dist[(size_t)v * (size_t)k + (size_t)i] =
                d == INT64_MAX ? INT32_MAX : sat_i32(d);
        }
    }
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    char magic[8] = {'L','N','D','M','R','K','\0','\0'};
    std::fwrite(magic, 1, 8, f);
    write_exact(f, LM_VERSION);
    uint32_t v = (uint32_t)out.V, kk = (uint32_t)out.K;
    write_exact(f, v);
    write_exact(f, kk);
    std::fwrite(out.nodes.data(), sizeof(int32_t), out.nodes.size(), f);
    std::fwrite(out.dist.data(), sizeof(int32_t), out.dist.size(), f);
    std::fclose(f);
}
