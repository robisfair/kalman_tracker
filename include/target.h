#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace tracker {

/// Represents the ground-truth state of a moving target.
/// State vector: [x, y, vx, vy]  (position in meters, velocity in m/s)
struct TargetState {
    double time;          ///< Simulation time in seconds
    Eigen::Vector4d state; ///< [x, y, vx, vy]

    double x()  const { return state(0); }
    double y()  const { return state(1); }
    double vx() const { return state(2); }
    double vy() const { return state(3); }
};

/// Generates a ground-truth trajectory for a target.
class Target {
public:
    enum class MotionModel { LINEAR, CURVED, EVASIVE };

    explicit Target(std::string id, MotionModel model = MotionModel::LINEAR);

    /// Generate trajectory from t=0 to t=duration_s at given dt
    std::vector<TargetState> generateTrajectory(double duration_s, double dt_s) const;

    const std::string& id() const { return id_; }

private:
    std::string   id_;
    MotionModel   model_;

    TargetState linearStep(const TargetState& s, double dt) const;
    TargetState curvedStep(const TargetState& s, double dt) const;
    TargetState evasiveStep(const TargetState& s, double dt, double t) const;
};

} // namespace tracker
