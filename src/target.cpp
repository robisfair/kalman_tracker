#include "target.h"
#include <cmath>
#include <stdexcept>

namespace tracker {

Target::Target(std::string id, MotionModel model)
    : id_(std::move(id)), model_(model) {}

std::vector<TargetState> Target::generateTrajectory(double duration_s, double dt_s) const {
    std::vector<TargetState> traj;
    int steps = static_cast<int>(duration_s / dt_s) + 1;
    traj.reserve(steps);

    // Initial state: starts at origin moving NE at 50 m/s
    TargetState s0;
    s0.time  = 0.0;
    s0.state = Eigen::Vector4d(0.0, 0.0, 35.0, 35.0); // ~50 m/s NE
    traj.push_back(s0);

    for (int i = 1; i < steps; ++i) {
        double t = i * dt_s;
        TargetState next;
        next.time = t;

        switch (model_) {
            case MotionModel::LINEAR:
                next = linearStep(traj.back(), dt_s);
                break;
            case MotionModel::CURVED:
                next = curvedStep(traj.back(), dt_s);
                break;
            case MotionModel::EVASIVE:
                next = evasiveStep(traj.back(), dt_s, t);
                break;
            default:
                throw std::runtime_error("Unknown motion model");
        }
        traj.push_back(next);
    }
    return traj;
}

TargetState Target::linearStep(const TargetState& s, double dt) const {
    TargetState next;
    next.time    = s.time + dt;
    next.state   = s.state;
    next.state(0) += s.vx() * dt;
    next.state(1) += s.vy() * dt;
    return next;
}

TargetState Target::curvedStep(const TargetState& s, double dt) const {
    // Constant turn rate: omega = 0.05 rad/s
    const double omega = 0.05;
    TargetState next;
    next.time = s.time + dt;
    double vx = s.vx() * std::cos(omega * dt) - s.vy() * std::sin(omega * dt);
    double vy = s.vx() * std::sin(omega * dt) + s.vy() * std::cos(omega * dt);
    next.state(0) = s.x() + vx * dt;
    next.state(1) = s.y() + vy * dt;
    next.state(2) = vx;
    next.state(3) = vy;
    return next;
}

TargetState Target::evasiveStep(const TargetState& s, double dt, double t) const {
    // Sinusoidal lateral maneuvers to simulate evasive action
    const double amp   = 5.0;   // m/s^2 acceleration amplitude
    const double freq  = 0.1;   // Hz
    double ax = 0.0;
    double ay = amp * std::sin(2.0 * M_PI * freq * t);

    TargetState next;
    next.time     = s.time + dt;
    next.state(2) = s.vx() + ax * dt;
    next.state(3) = s.vy() + ay * dt;
    next.state(0) = s.x() + next.state(2) * dt;
    next.state(1) = s.y() + next.state(3) * dt;
    return next;
}

} // namespace tracker
