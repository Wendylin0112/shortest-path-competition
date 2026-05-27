#ifndef SPC_ARCFLAGS_H
#define SPC_ARCFLAGS_H

#include "common.h"
#include <cstdint>
#include <string>
#include <vector>

struct ArcFlagsIndex {
    int32_t V = 0;
    int32_t E = 0;
    int32_t rows = 0;
    int32_t cols = 0;
    int32_t region_count = 0;
    double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
    std::vector<uint64_t> flags;
    std::vector<uint16_t> region_of_node;

    bool enabled() const { return !flags.empty() && region_count > 0; }
    int32_t region_of(int32_t node) const;
    bool allow(int32_t edge_id, int32_t target_region) const;
};

bool load_arcflags(const std::string& path, const SPGraphView& g,
                   ArcFlagsIndex& out);
void compute_arcflags(const SPGraphView& g, int32_t rows, int32_t cols,
                      const std::string& path, ArcFlagsIndex& out);

#endif
