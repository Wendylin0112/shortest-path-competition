#include "query_grouping.h"

#include <algorithm>
#include <cstdint>

void batch_answer_queries(const SPGraphView& g, std::vector<Query>& queries,
                          int32_t threshold, const SSSPFn& forward_sssp,
                          const SSSPFn& reverse_sssp,
                          const QueryFn& fallback_query) {
    if (threshold <= 0) threshold = 8;
    std::vector<int32_t> sfreq((size_t)g.V, 0), tfreq((size_t)g.V, 0);
    for (const Query& q : queries) {
        if (q.s >= 0 && q.s < g.V) ++sfreq[(size_t)q.s];
        if (q.t >= 0 && q.t < g.V) ++tfreq[(size_t)q.t];
    }
    std::vector<int64_t> dist;
    for (int32_t s = 0; s < g.V; ++s) {
        if (sfreq[(size_t)s] < threshold) continue;
        forward_sssp(s, dist);
        for (Query& q : queries) {
            if (!q.answered && q.s == s) {
                int64_t d = dist[(size_t)q.t];
                q.answer = d == INT64_MAX ? -1 : d;
                q.answered = true;
            }
        }
    }
    for (int32_t t = 0; t < g.V; ++t) {
        if (tfreq[(size_t)t] < threshold) continue;
        reverse_sssp(t, dist);
        for (Query& q : queries) {
            if (!q.answered && q.t == t) {
                int64_t d = dist[(size_t)q.s];
                q.answer = d == INT64_MAX ? -1 : d;
                q.answered = true;
            }
        }
    }
    for (Query& q : queries) {
        if (!q.answered) {
            q.answer = fallback_query(q.s, q.t);
            q.answered = true;
        }
    }
}
