#include "sensor.h"

namespace tracker {

Sensor::Sensor(Config cfg, unsigned seed)
    : cfg_(std::move(cfg))
    , rng_(seed)
    , dist_x_(0.0, cfg_.pos_std_x)
    , dist_y_(0.0, cfg_.pos_std_y)
    , dist_uni_(0.0, 1.0)
{}

Measurement Sensor::observe(const TargetState& truth) const {
    Measurement m;
    m.time      = truth.time;
    m.sensor_id = cfg_.id;
    m.valid     = false;

    // Check for dropout event
    if (dist_uni_(rng_) < cfg_.dropout_prob) {
        m.position = Eigen::Vector2d::Zero();
        m.noise_cov = Eigen::Matrix2d::Zero();
        return m;  // valid = false
    }

    // Check detection probability
    if (dist_uni_(rng_) > cfg_.detection_prob) {
        m.position = Eigen::Vector2d::Zero();
        m.noise_cov = Eigen::Matrix2d::Zero();
        return m;  // valid = false — target not detected this timestep
    }

    // Apply Gaussian noise
    m.valid = true;
    m.position(0) = truth.x() + dist_x_(rng_);
    m.position(1) = truth.y() + dist_y_(rng_);

    // Noise covariance matrix (diagonal for independent x,y noise)
    m.noise_cov = Eigen::Matrix2d::Zero();
    m.noise_cov(0, 0) = cfg_.pos_std_x * cfg_.pos_std_x;
    m.noise_cov(1, 1) = cfg_.pos_std_y * cfg_.pos_std_y;

    return m;
}

} // namespace tracker
