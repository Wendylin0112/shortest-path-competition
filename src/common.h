#ifndef SPC_COMMON_H
#define SPC_COMMON_H

#include <cstdint>
#include <vector>

struct SPGraphView {
    int32_t V = 0;
    int32_t E = 0;
    int32_t max_w = 0;
    bool has_coords = false;
    const std::vector<int32_t>* head = nullptr;
    const std::vector<int32_t>* to = nullptr;
    const std::vector<int32_t>* w = nullptr;
    const std::vector<double>* x = nullptr;
    const std::vector<double>* y = nullptr;
};

struct Query {
    int32_t s = 0;
    int32_t t = 0;
    int64_t answer = -1;
    bool answered = false;
};

#endif
