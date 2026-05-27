#ifndef SPC_LANDMARKS_H
#define SPC_LANDMARKS_H

#include "common.h"
#include <string>
#include <vector>

struct LandmarkIndex {
    int32_t V = 0;
    int32_t K = 0;
    std::vector<int32_t> nodes;
    std::vector<int32_t> dist;

    int64_t heuristic(int32_t v, int32_t t) const;
    bool empty() const { return K <= 0 || dist.empty(); }
};

std::vector<int32_t> select_landmarks(const SPGraphView& g, int32_t k,
                                      const std::string& strategy);
bool load_landmarks(const std::string& path, int32_t V, LandmarkIndex& out);
void compute_landmarks(const SPGraphView& g, int32_t k,
                       const std::string& strategy,
                       const std::string& path,
                       LandmarkIndex& out);

#endif
