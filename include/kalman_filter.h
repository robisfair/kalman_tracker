#pragma once
#include <Eigen/Dense>
#include <vector>

namespace tracker {

/// Discrete-time linear Kalman Filter for a constant-velocity target.
///
/// State vector:       x = [px, py, vx, vy]^T
/// Measurement vector: z = [px, py]^T
///
/// Process model (F):  constant-velocity kinematics
/// Measurement model (H): identity on position sub-space
class KalmanFilter {
public:
    struct Estimate {
        double         time;
        Eigen::Vector4d state;    ///< Posterior state estimate
        Eigen::Matrix4d cov;      ///< Posterior covariance
        Eigen::Vector4d state_prior;   ///< Pre-update (predicted) state
        double          innovation_x;  ///< z - H*x_prior  (x component)
        double          innovation_y;  ///< z - H*x_prior  (y component)
    };

    /// @param process_noise_std  std-dev of acceleration noise (m/s^2)
    explicit KalmanFilter(double process_noise_std = 0.5);

    /// Initialize state from first measurement
    void initialize(double time, const Eigen::Vector2d& pos,
                    const Eigen::Matrix2d& R);

    /// Predict step: propagate state forward by dt seconds
    void predict(double dt);

    /// Update step: incorporate a new measurement
    /// @param z   Measured position [px, py]
    /// @param R   Measurement noise covariance 2x2
    Estimate update(double time, const Eigen::Vector2d& z,
                    const Eigen::Matrix2d& R);

    bool isInitialized() const { return initialized_; }

    const Eigen::Vector4d& state() const { return x_; }
    const Eigen::Matrix4d& cov()   const { return P_; }

    /// Returns all recorded estimates
    const std::vector<Estimate>& history() const { return history_; }

private:
    bool            initialized_{false};
    double          q_std_;          ///< Process noise std dev
    double          last_time_{0.0};

    Eigen::Vector4d x_;   ///< State estimate
    Eigen::Matrix4d P_;   ///< Error covariance
    Eigen::Matrix4d F_;   ///< State transition matrix
    Eigen::Matrix4d Q_;   ///< Process noise covariance
    Eigen::MatrixXd H_;   ///< Measurement matrix (2x4)

    std::vector<Estimate> history_;

    void buildF(double dt);
    void buildQ(double dt);
};

} // namespace tracker
