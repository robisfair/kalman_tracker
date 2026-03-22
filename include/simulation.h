#pragma once
#include "target.h"
#include "sensor.h"
#include "kalman_filter.h"
#include "statistics.h"
#include <nlohmann/json.hpp>
#include <vector>
#include <memory>
#include <string>

namespace tracker {

struct SimConfig {
    std::string          scenario_name;
    double               duration_s;
    double               dt_s;
    double               process_noise_std;
    Target::MotionModel  motion_model;
    std::vector<Sensor::Config> sensors;

    static SimConfig fromJSON(const nlohmann::json& j);
    static SimConfig fromFile(const std::string& path);
};

struct SimResult {
    std::string                        scenario_name;
    std::vector<TargetState>           truth;
    std::vector<KalmanFilter::Estimate> estimates;
    TrackStats                         stats;
};

/// Runs the full simulation pipeline:
///   1. Generate ground-truth trajectory
///   2. Collect noisy sensor measurements
///   3. Fuse through Kalman Filter
///   4. Compute statistics
SimResult runSimulation(const SimConfig& cfg);

} // namespace tracker
