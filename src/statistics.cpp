#include "statistics.h"
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>

namespace tracker {

TrackStats computeStats(const std::vector<KalmanFilter::Estimate>& estimates,
                        const std::vector<TargetState>& truth) {
    if (estimates.empty()) throw std::runtime_error("No estimates to compare");

    TrackStats stats{};
    stats.num_estimates = static_cast<int>(estimates.size());

    double sum_sq_pos = 0.0;
    double sum_sq_vel = 0.0;
    double sum_inn_x  = 0.0;
    double sum_inn_y  = 0.0;
    double peak       = 0.0;

    for (const auto& est : estimates) {
        // Find closest ground truth time
        auto it = std::min_element(truth.begin(), truth.end(),
            [&](const TargetState& a, const TargetState& b) {
                return std::abs(a.time - est.time) < std::abs(b.time - est.time);
            });

        double ex = est.state(0) - it->x();
        double ey = est.state(1) - it->y();
        double evx = est.state(2) - it->vx();
        double evy = est.state(3) - it->vy();

        double pos_err = std::sqrt(ex*ex + ey*ey);
        sum_sq_pos += pos_err * pos_err;
        sum_sq_vel += evx*evx + evy*evy;

        sum_inn_x += est.innovation_x;
        sum_inn_y += est.innovation_y;
        peak = std::max(peak, pos_err);
    }

    int n = stats.num_estimates;
    stats.rmse_position     = std::sqrt(sum_sq_pos / n);
    stats.rmse_velocity     = std::sqrt(sum_sq_vel / (2.0 * n));
    stats.mean_innovation_x = sum_inn_x / n;
    stats.mean_innovation_y = sum_inn_y / n;
    stats.peak_error        = peak;

    return stats;
}

void exportCSV(const std::string& filepath,
               const std::vector<TargetState>& truth,
               const std::vector<KalmanFilter::Estimate>& estimates) {
    std::ofstream f(filepath);
    if (!f.is_open()) throw std::runtime_error("Cannot open: " + filepath);

    f << "time,truth_x,truth_y,truth_vx,truth_vy,"
         "est_x,est_y,est_vx,est_vy,"
         "prior_x,prior_y,innovation_x,innovation_y\n";

    f << std::fixed << std::setprecision(4);

    // Zip truth and estimates by closest time
    for (const auto& est : estimates) {
        auto it = std::min_element(truth.begin(), truth.end(),
            [&](const TargetState& a, const TargetState& b) {
                return std::abs(a.time - est.time) < std::abs(b.time - est.time);
            });

        f << est.time               << ","
          << it->x()                << "," << it->y()  << ","
          << it->vx()               << "," << it->vy() << ","
          << est.state(0)           << "," << est.state(1) << ","
          << est.state(2)           << "," << est.state(3) << ","
          << est.state_prior(0)     << "," << est.state_prior(1) << ","
          << est.innovation_x       << "," << est.innovation_y << "\n";
    }
}

void printSummary(const std::string& scenario_name, const TrackStats& stats) {
    const int W = 36;
    std::cout << "\n";
    std::cout << std::string(W + 14, '=') << "\n";
    std::cout << "  Scenario: " << scenario_name << "\n";
    std::cout << std::string(W + 14, '-') << "\n";
    std::cout << std::left << std::setw(W) << "  RMSE Position (m)"
              << std::right << std::setw(10) << std::fixed << std::setprecision(3)
              << stats.rmse_position << "\n";
    std::cout << std::left << std::setw(W) << "  RMSE Velocity (m/s)"
              << std::right << std::setw(10) << stats.rmse_velocity << "\n";
    std::cout << std::left << std::setw(W) << "  Peak Position Error (m)"
              << std::right << std::setw(10) << stats.peak_error << "\n";
    std::cout << std::left << std::setw(W) << "  Mean Innovation X (m)"
              << std::right << std::setw(10) << stats.mean_innovation_x << "\n";
    std::cout << std::left << std::setw(W) << "  Mean Innovation Y (m)"
              << std::right << std::setw(10) << stats.mean_innovation_y << "\n";
    std::cout << std::left << std::setw(W) << "  Total Estimates"
              << std::right << std::setw(10) << stats.num_estimates << "\n";
    std::cout << std::string(W + 14, '=') << "\n\n";
}

} // namespace tracker
