#ifndef SPC_QUERY_GROUPING_H
#define SPC_QUERY_GROUPING_H

#include "common.h"
#include <functional>
#include <vector>

using SSSPFn = std::function<void(int32_t, std::vector<int64_t>&)>;
using QueryFn = std::function<int64_t(int32_t, int32_t)>;

void batch_answer_queries(const SPGraphView& g, std::vector<Query>& queries,
                          int32_t threshold, const SSSPFn& forward_sssp,
                          const SSSPFn& reverse_sssp,
                          const QueryFn& fallback_query);

#endif
