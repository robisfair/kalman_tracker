#pragma once
#include "target.h"
#include <Eigen/Dense>
#include <random>
#include <string>
#include <optional>

namespace tracker {

/// A noisy position measurement from a single sensor.
struct Measurement {
    double time;
    std::string sensor_id;
    Eigen::Vector2d position;   ///< Measured [x, y] with noise applied
    Eigen::Matrix2d noise_cov;  ///< Sensor noise covariance (R matrix)
    bool valid;                 ///< False when sensor is dropped out
};

/// Models a real-world sensor (Radar, IR, Acoustic) with configurable noise.
class Sensor {
public:
    struct Config {
        std::string id;
        double pos_std_x;      ///< 1-sigma position noise in x (meters)
        double pos_std_y;      ///< 1-sigma position noise in y (meters)
        double detection_prob; ///< Probability of detection per timestep [0,1]
        double dropout_prob;   ///< Probability of a full dropout event [0,1]
    };

    explicit Sensor(Config cfg, unsigned seed = 42);

    /// Observe a ground-truth state and return a (potentially noisy/missing) measurement
    Measurement observe(const TargetState& truth) const;

    const std::string& id()     const { return cfg_.id; }
    const Config&      config() const { return cfg_; }

private:
    Config cfg_;
    mutable std::mt19937 rng_;
    mutable std::normal_distribution<double> dist_x_;
    mutable std::normal_distribution<double> dist_y_;
    mutable std::uniform_real_distribution<double> dist_uni_;
};

} // namespace tracker
