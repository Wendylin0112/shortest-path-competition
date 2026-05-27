#include "arcflags.h"

#include <algorithm>
#include <atomic>
#include <climits>
#include <cstdio>
#include <cstring>
#include <queue>
#include <thread>
#include <utility>

namespace {
constexpr uint32_t AF_VERSION = 1;
template <class T> bool read_exact(std::FILE* f, T& v) { return std::fread(&v, sizeof(T), 1, f) == 1; }
template <class T> void write_exact(std::FILE* f, const T& v) { std::fwrite(&v, sizeof(T), 1, f); }

void assign_regions(const SPGraphView& g, ArcFlagsIndex& out) {
    out.region_of_node.assign((size_t)g.V, 0);
    if (!g.has_coords || !g.x || !g.y || out.rows <= 0 || out.cols <= 0) return;
    const auto& xs = *g.x;
    const auto& ys = *g.y;
    out.min_x = out.max_x = xs.empty() ? 0.0 : xs[0];
    out.min_y = out.max_y = ys.empty() ? 0.0 : ys[0];
    for (int32_t v = 0; v < g.V; ++v) {
        out.min_x = std::min(out.min_x, xs[(size_t)v]);
        out.max_x = std::max(out.max_x, xs[(size_t)v]);
        out.min_y = std::min(out.min_y, ys[(size_t)v]);
        out.max_y = std::max(out.max_y, ys[(size_t)v]);
    }
    double wx = (out.max_x - out.min_x) > 0.0 ? (out.max_x - out.min_x) : 1.0;
    double wy = (out.max_y - out.min_y) > 0.0 ? (out.max_y - out.min_y) : 1.0;
    for (int32_t v = 0; v < g.V; ++v) {
        int32_t c = (int32_t)(((xs[(size_t)v] - out.min_x) / wx) * out.cols);
        int32_t r = (int32_t)(((ys[(size_t)v] - out.min_y) / wy) * out.rows);
        if (c < 0) c = 0;
        if (c >= out.cols) c = out.cols - 1;
        if (r < 0) r = 0;
        if (r >= out.rows) r = out.rows - 1;
        out.region_of_node[(size_t)v] = (uint16_t)(r * out.cols + c);
    }
}
}

int32_t ArcFlagsIndex::region_of(int32_t node) const {
    if (node < 0 || node >= V || region_of_node.empty()) return 0;
    return (int32_t)region_of_node[(size_t)node];
}

bool ArcFlagsIndex::allow(int32_t edge_id, int32_t target_region) const {
    if (!enabled()) return true;
    if (edge_id < 0 || edge_id >= E || target_region < 0 || target_region >= region_count) return true;
    return (flags[(size_t)edge_id] & (1ULL << target_region)) != 0;
}

bool load_arcflags(const std::string& path, const SPGraphView& g,
                   ArcFlagsIndex& out) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    char magic[8] = {};
    uint32_t version = 0, nodes = 0, edges = 0;
    uint16_t regions = 0, rows = 0, cols = 0;
    bool ok = std::fread(magic, 1, 8, f) == 8 &&
              std::memcmp(magic, "ARCFLG\0", 8) == 0 &&
              read_exact(f, version) && read_exact(f, nodes) && read_exact(f, edges) &&
              read_exact(f, regions) && read_exact(f, rows) && read_exact(f, cols) &&
              version == AF_VERSION && (int32_t)nodes == g.V && (int32_t)edges == g.E &&
              regions > 0 && regions <= 64;
    if (ok) {
        out.V = (int32_t)nodes; out.E = (int32_t)edges;
        out.region_count = (int32_t)regions; out.rows = (int32_t)rows; out.cols = (int32_t)cols;
        out.flags.resize(edges);
        ok = std::fread(out.flags.data(), sizeof(uint64_t), edges, f) == edges;
        assign_regions(g, out);
    }
    std::fclose(f);
    if (!ok) out = ArcFlagsIndex{};
    return ok;
}

void compute_arcflags(const SPGraphView& g, int32_t rows, int32_t cols,
                      const std::string& path, ArcFlagsIndex& out) {
    if (rows <= 0) rows = 8;
    if (cols <= 0) cols = 8;
    int32_t regions = rows * cols;
    if (regions <= 0 || regions > 64) {
        std::fprintf(stderr, "[arcflags] grid has %d regions; must be 1..64\n", regions);
        out = ArcFlagsIndex{};
        return;
    }
    uint64_t bytes = (uint64_t)g.E * sizeof(uint64_t);
    if (bytes > (1ULL << 30)) {
        std::fprintf(stderr, "[arcflags] edge flag memory %.1f MB exceeds 1GB cap\n", bytes / 1048576.0);
        out = ArcFlagsIndex{};
        return;
    }
    out.V = g.V; out.E = g.E; out.rows = rows; out.cols = cols; out.region_count = regions;
    out.flags.assign((size_t)g.E, 0);
    assign_regions(g, out);

    std::vector<int32_t> src((size_t)g.E, 0);
    std::vector<std::vector<int32_t>> incoming((size_t)g.V);
    const auto& head = *g.head; const auto& to = *g.to; const auto& w = *g.w;
    for (int32_t u = 0; u < g.V; ++u) {
        for (int32_t e = head[(size_t)u]; e < head[(size_t)u + 1]; ++e) {
            src[(size_t)e] = u;
            incoming[(size_t)to[(size_t)e]].push_back(e);
        }
    }

    int32_t nthreads = (int32_t)std::thread::hardware_concurrency();
    if (nthreads <= 0) nthreads = 1;
    if (nthreads > regions) nthreads = regions;
    std::vector<std::vector<uint64_t>> locals((size_t)nthreads, std::vector<uint64_t>((size_t)g.E, 0));
    std::atomic<int32_t> next_region(0);
    auto worker = [&](int32_t tid) {
        using Item = std::pair<int64_t, int32_t>;
        std::vector<int64_t> dist((size_t)g.V);
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
        while (true) {
            int32_t r = next_region.fetch_add(1);
            if (r >= regions) break;
            std::fill(dist.begin(), dist.end(), INT64_MAX);
            while (!pq.empty()) pq.pop();
            for (int32_t v = 0; v < g.V; ++v) {
                if (out.region_of_node[(size_t)v] == (uint16_t)r) {
                    dist[(size_t)v] = 0;
                    pq.push({0, v});
                }
            }
            uint64_t bit = 1ULL << r;
            for (int32_t e = 0; e < g.E; ++e) {
                if (out.region_of_node[(size_t)to[(size_t)e]] == (uint16_t)r)
                    locals[(size_t)tid][(size_t)e] |= bit;
            }
            while (!pq.empty()) {
                auto [d, v] = pq.top(); pq.pop();
                if (d != dist[(size_t)v]) continue;
                for (int32_t eid : incoming[(size_t)v]) {
                    int32_t u = src[(size_t)eid];
                    int64_t nd = d + (int64_t)w[(size_t)eid];
                    if (nd < dist[(size_t)u]) {
                        dist[(size_t)u] = nd;
                        locals[(size_t)tid][(size_t)eid] |= bit;
                        pq.push({nd, u});
                    }
                }
            }
        }
    };
    std::vector<std::thread> threads;
    for (int32_t t = 0; t < nthreads; ++t) threads.emplace_back(worker, t);
    for (auto& th : threads) th.join();
    for (int32_t t = 0; t < nthreads; ++t)
        for (int32_t e = 0; e < g.E; ++e) out.flags[(size_t)e] |= locals[(size_t)t][(size_t)e];

    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    char magic[8] = {'A','R','C','F','L','G','\0','\0'};
    std::fwrite(magic, 1, 8, f);
    write_exact(f, AF_VERSION);
    uint32_t nodes = (uint32_t)g.V, edges = (uint32_t)g.E;
    uint16_t rr = (uint16_t)regions, r16 = (uint16_t)rows, c16 = (uint16_t)cols;
    write_exact(f, nodes); write_exact(f, edges); write_exact(f, rr); write_exact(f, r16); write_exact(f, c16);
    std::fwrite(out.flags.data(), sizeof(uint64_t), out.flags.size(), f);
    std::fclose(f);
}
