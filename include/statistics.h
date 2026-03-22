#pragma once
#include "kalman_filter.h"
#include "target.h"
#include <vector>
#include <string>

namespace tracker {

struct TrackStats {
    double rmse_position;     ///< Root Mean Square Error in position (m)
    double rmse_velocity;     ///< Root Mean Square Error in velocity (m/s)
    double mean_innovation_x; ///< Average innovation in x
    double mean_innovation_y; ///< Average innovation in y
    double peak_error;        ///< Worst-case position error (m)
    int    num_estimates;
};

/// Compute tracking statistics by comparing KF estimates to ground truth.
TrackStats computeStats(const std::vector<KalmanFilter::Estimate>& estimates,
                        const std::vector<TargetState>& truth);

/// Export ground truth, raw measurements, and KF estimates to CSV.
void exportCSV(const std::string& filepath,
               const std::vector<TargetState>& truth,
               const std::vector<KalmanFilter::Estimate>& estimates);

/// Print a formatted summary table to stdout.
void printSummary(const std::string& scenario_name,
                  const TrackStats& stats);

} // namespace tracker
