#include "kalman_filter.h"
#include <stdexcept>

namespace tracker {

KalmanFilter::KalmanFilter(double process_noise_std)
    : q_std_(process_noise_std)
{
    // Measurement matrix: H maps [px,py,vx,vy] -> [px,py]
    H_ = Eigen::MatrixXd::Zero(2, 4);
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
}

void KalmanFilter::initialize(double time, const Eigen::Vector2d& pos,
                               const Eigen::Matrix2d& R) {
    last_time_ = time;

    // Initialize state: position from measurement, velocity = 0
    x_ = Eigen::Vector4d(pos(0), pos(1), 0.0, 0.0);

    // Initial covariance: high uncertainty on velocity
    P_ = Eigen::Matrix4d::Zero();
    P_(0, 0) = R(0, 0);          // px uncertainty from sensor
    P_(1, 1) = R(1, 1);          // py uncertainty from sensor
    P_(2, 2) = 100.0;            // vx unknown
    P_(3, 3) = 100.0;            // vy unknown

    initialized_ = true;
}

void KalmanFilter::predict(double dt) {
    if (!initialized_) throw std::runtime_error("KF not initialized");
    buildF(dt);
    buildQ(dt);

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

KalmanFilter::Estimate KalmanFilter::update(double time,
                                             const Eigen::Vector2d& z,
                                             const Eigen::Matrix2d& R) {
    if (!initialized_) throw std::runtime_error("KF not initialized");

    double dt = time - last_time_;
    if (dt > 0.0) predict(dt);
    last_time_ = time;

    Estimate est;
    est.time        = time;
    est.state_prior = x_;

    // Innovation
    Eigen::Vector2d y = z - H_ * x_;
    est.innovation_x  = y(0);
    est.innovation_y  = y(1);

    // Innovation covariance
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R;

    // Kalman gain
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    // State & covariance update
    x_ = x_ + K * y;
    P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;

    est.state = x_;
    est.cov   = P_;

    history_.push_back(est);
    return est;
}

void KalmanFilter::buildF(double dt) {
    F_ = Eigen::Matrix4d::Identity();
    F_(0, 2) = dt;   // px += vx*dt
    F_(1, 3) = dt;   // py += vy*dt
}

void KalmanFilter::buildQ(double dt) {
    // Discrete white noise acceleration model
    double dt2 = dt  * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    double q   = q_std_ * q_std_;

    Q_ = Eigen::Matrix4d::Zero();
    Q_(0, 0) = q * dt4 / 4.0;
    Q_(0, 2) = q * dt3 / 2.0;
    Q_(1, 1) = q * dt4 / 4.0;
    Q_(1, 3) = q * dt3 / 2.0;
    Q_(2, 0) = q * dt3 / 2.0;
    Q_(2, 2) = q * dt2;
    Q_(3, 1) = q * dt3 / 2.0;
    Q_(3, 3) = q * dt2;
}

} // namespace tracker
