#include "simulation.h"
#include <fstream>
#include <stdexcept>
#include <iostream>

namespace tracker {

// ── JSON helpers ──────────────────────────────────────────────────────────────

static Target::MotionModel parseMotionModel(const std::string& s) {
    if (s == "linear")  return Target::MotionModel::LINEAR;
    if (s == "curved")  return Target::MotionModel::CURVED;
    if (s == "evasive") return Target::MotionModel::EVASIVE;
    throw std::runtime_error("Unknown motion model: " + s);
}

SimConfig SimConfig::fromJSON(const nlohmann::json& j) {
    SimConfig cfg;
    cfg.scenario_name     = j.at("scenario_name").get<std::string>();
    cfg.duration_s        = j.at("duration_s").get<double>();
    cfg.dt_s              = j.at("dt_s").get<double>();
    cfg.process_noise_std = j.at("process_noise_std").get<double>();
    cfg.motion_model      = parseMotionModel(j.at("motion_model").get<std::string>());

    for (const auto& sj : j.at("sensors")) {
        Sensor::Config sc;
        sc.id             = sj.at("id").get<std::string>();
        sc.pos_std_x      = sj.at("pos_std_x").get<double>();
        sc.pos_std_y      = sj.at("pos_std_y").get<double>();
        sc.detection_prob = sj.at("detection_prob").get<double>();
        sc.dropout_prob   = sj.at("dropout_prob").get<double>();
        cfg.sensors.push_back(sc);
    }
    return cfg;
}

SimConfig SimConfig::fromFile(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) throw std::runtime_error("Cannot open config: " + path);
    nlohmann::json j;
    f >> j;
    return fromJSON(j);
}

// ── Simulation runner ─────────────────────────────────────────────────────────

SimResult runSimulation(const SimConfig& cfg) {
    SimResult result;
    result.scenario_name = cfg.scenario_name;

    // 1. Generate ground-truth trajectory
    Target target("T1", cfg.motion_model);
    result.truth = target.generateTrajectory(cfg.duration_s, cfg.dt_s);
    std::cout << "[Sim] Generated " << result.truth.size()
              << " ground-truth steps for scenario: " << cfg.scenario_name << "\n";

    // 2. Create sensors
    std::vector<Sensor> sensors;
    unsigned seed = 100;
    for (const auto& sc : cfg.sensors) {
        sensors.emplace_back(sc, seed++);
    }

    // 3. Kalman Filter
    KalmanFilter kf(cfg.process_noise_std);

    int valid_meas = 0;
    for (const auto& truth_state : result.truth) {
        // Collect measurements from all sensors, average valid ones
        Eigen::Vector2d fused_pos = Eigen::Vector2d::Zero();
        Eigen::Matrix2d fused_cov = Eigen::Matrix2d::Zero();
        int count = 0;

        for (auto& sensor : sensors) {
            auto meas = sensor.observe(truth_state);
            if (!meas.valid) continue;

            fused_pos += meas.position;
            fused_cov += meas.noise_cov;
            count++;
        }

        if (count == 0) continue;

        // Simple averaging fusion: mean position, averaged covariance
        fused_pos /= count;
        fused_cov /= (count * count);  // variance of mean

        if (!kf.isInitialized()) {
            kf.initialize(truth_state.time, fused_pos, fused_cov);
            continue;
        }

        kf.update(truth_state.time, fused_pos, fused_cov);
        valid_meas++;
    }

    std::cout << "[Sim] KF processed " << valid_meas << " valid fused measurements\n";

    // 4. Gather results
    result.estimates = kf.history();
    result.stats     = computeStats(result.estimates, result.truth);

    return result;
}

} // namespace tracker
