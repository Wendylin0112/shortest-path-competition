#ifndef SPC_REACH_H
#define SPC_REACH_H

#include "common.h"
#include <cstdint>
#include <string>
#include <vector>

struct ReachIndex {
    int32_t V = 0;
    std::vector<uint32_t> reach;
    bool enabled() const { return !reach.empty(); }
    uint32_t operator[](int32_t v) const { return reach[(size_t)v]; }
};

bool load_reach(const std::string& path, int32_t V, ReachIndex& out);
void compute_reach(const SPGraphView& g, int32_t samples,
                   const std::string& path, ReachIndex& out);

#endif
