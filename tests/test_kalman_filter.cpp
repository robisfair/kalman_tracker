#include <gtest/gtest.h>
#include "kalman_filter.h"
#include <cmath>

using namespace tracker;

class KalmanFilterTest : public ::testing::Test {
protected:
    KalmanFilter kf{0.5};
};

TEST_F(KalmanFilterTest, InitializesCorrectly) {
    Eigen::Vector2d pos(100.0, 200.0);
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 25.0;
    kf.initialize(0.0, pos, R);

    EXPECT_TRUE(kf.isInitialized());
    EXPECT_NEAR(kf.state()(0), 100.0, 1e-9);
    EXPECT_NEAR(kf.state()(1), 200.0, 1e-9);
    EXPECT_NEAR(kf.state()(2), 0.0,   1e-9);  // velocity initialized to 0
    EXPECT_NEAR(kf.state()(3), 0.0,   1e-9);
}

TEST_F(KalmanFilterTest, ThrowsIfNotInitialized) {
    EXPECT_THROW(kf.predict(1.0), std::runtime_error);

    Eigen::Vector2d z(0.0, 0.0);
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    EXPECT_THROW(kf.update(1.0, z, R), std::runtime_error);
}

TEST_F(KalmanFilterTest, ConvergesOnStaticTarget) {
    // A static target at (500, 500); after many updates, KF should converge
    Eigen::Vector2d true_pos(500.0, 500.0);
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 100.0; // 10m std dev

    kf.initialize(0.0, true_pos, R);

    for (int i = 1; i <= 50; ++i) {
        kf.update(i * 0.5, true_pos, R);
    }

    EXPECT_NEAR(kf.state()(0), 500.0, 5.0);   // within 5m
    EXPECT_NEAR(kf.state()(1), 500.0, 5.0);
    EXPECT_NEAR(kf.state()(2), 0.0,   2.0);   // velocity near 0
}

TEST_F(KalmanFilterTest, TracksLinearMotion) {
    // Target moving at constant 50 m/s in X
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 25.0;
    kf.initialize(0.0, Eigen::Vector2d(0.0, 0.0), R);

    double vx = 50.0;
    for (int i = 1; i <= 40; ++i) {
        double t = i * 0.5;
        Eigen::Vector2d z(vx * t, 0.0);  // perfect measurement (no noise)
        kf.update(t, z, R);
    }

    // After converging, velocity estimate should be near 50 m/s
    EXPECT_NEAR(kf.state()(2), 50.0, 5.0);
}

TEST_F(KalmanFilterTest, RecordsHistory) {
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 25.0;
    kf.initialize(0.0, Eigen::Vector2d(0.0, 0.0), R);

    for (int i = 1; i <= 10; ++i) {
        kf.update(i * 0.5, Eigen::Vector2d(i * 5.0, 0.0), R);
    }

    EXPECT_EQ(kf.history().size(), 10u);
}

TEST_F(KalmanFilterTest, CovarianceDecreases) {
    // Covariance should reduce as more measurements arrive
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 100.0;
    kf.initialize(0.0, Eigen::Vector2d(0.0, 0.0), R);

    double initial_px = kf.cov()(0, 0);

    for (int i = 1; i <= 20; ++i) {
        kf.update(i * 0.5, Eigen::Vector2d(0.0, 0.0), R);
    }

    double final_px = kf.cov()(0, 0);
    EXPECT_LT(final_px, initial_px);
}
