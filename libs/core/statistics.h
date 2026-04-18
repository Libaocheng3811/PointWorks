#ifndef POINTWORKS_STATISTICS_H
#define POINTWORKS_STATISTICS_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

namespace ct {

struct ScalarFieldStats {
    int total_points = 0;
    int valid_points = 0;
    int nan_points = 0;
    double min_val = 0;
    double max_val = 0;
    double mean_val = 0;
    double std_val = 0;
    double median_val = 0;
};

inline ScalarFieldStats computeStatistics(const std::vector<float>& values)
{
    ScalarFieldStats stats;
    stats.total_points = static_cast<int>(values.size());

    double sum = 0;
    double sq_sum = 0;
    std::vector<float> valid;

    for (float v : values) {
        if (std::isnan(v)) {
            stats.nan_points++;
            continue;
        }
        stats.valid_points++;
        if (v < stats.min_val) stats.min_val = v;
        if (v > stats.max_val) stats.max_val = v;
        sum += v;
        sq_sum += static_cast<double>(v) * v;
        valid.push_back(v);
    }

    if (stats.valid_points == 0) return stats;

    stats.mean_val = sum / stats.valid_points;
    stats.std_val = std::sqrt(sq_sum / stats.valid_points - stats.mean_val * stats.mean_val);

    // Median
    std::sort(valid.begin(), valid.end());
    if (stats.valid_points % 2 == 0) {
        size_t mid = stats.valid_points / 2;
        stats.median_val = (static_cast<double>(valid[mid - 1]) + valid[mid]) / 2.0;
    } else {
        stats.median_val = static_cast<double>(valid[stats.valid_points / 2]);
    }

    return stats;
}

}  // namespace ct

#endif  // POINTWORKS_STATISTICS_H
