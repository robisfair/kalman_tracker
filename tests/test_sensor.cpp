#include <gtest/gtest.h>
#include "sensor.h"
#include <cmath>

using namespace tracker;

static TargetState makeTruth(double t, double x, double y) {
    TargetState s;
    s.time = t;
    s.state = Eigen::Vector4d(x, y, 0.0, 0.0);
    return s;
}

TEST(SensorTest, AlwaysDetectsHighProbSensor) {
    Sensor::Config cfg{"TestRadar", 5.0, 5.0, 1.0, 0.0};  // 100% detect, 0% dropout
    Sensor sensor(cfg, 42);

    auto truth = makeTruth(0.0, 100.0, 200.0);
    int valid = 0;
    for (int i = 0; i < 100; ++i) {
        if (sensor.observe(truth).valid) valid++;
    }
    EXPECT_EQ(valid, 100);
}

TEST(SensorTest, NeverDetectsZeroProbSensor) {
    Sensor::Config cfg{"ZeroSensor", 5.0, 5.0, 0.0, 0.0};  // 0% detect
    Sensor sensor(cfg, 42);

    auto truth = makeTruth(0.0, 100.0, 200.0);
    for (int i = 0; i < 50; ++i) {
        EXPECT_FALSE(sensor.observe(truth).valid);
    }
}

TEST(SensorTest, NoiseMagnitudeIsReasonable) {
    // With 10m std dev, 99% of measurements should be within 30m
    Sensor::Config cfg{"NoisySensor", 10.0, 10.0, 1.0, 0.0};
    Sensor sensor(cfg, 123);

    auto truth = makeTruth(0.0, 500.0, 500.0);
    int out_of_range = 0;
    for (int i = 0; i < 1000; ++i) {
        auto m = sensor.observe(truth);
        double err = std::sqrt(std::pow(m.position(0) - 500.0, 2) +
                               std::pow(m.position(1) - 500.0, 2));
        if (err > 50.0) out_of_range++;
    }
    // Expect fewer than 1% outliers beyond 5 sigma
    EXPECT_LT(out_of_range, 10);
}

TEST(SensorTest, NoiseCovMatrixMatchesConfig) {
    Sensor::Config cfg{"MatrixTest", 8.0, 12.0, 1.0, 0.0};
    Sensor sensor(cfg, 99);
    auto truth = makeTruth(0.0, 0.0, 0.0);

    auto m = sensor.observe(truth);
    ASSERT_TRUE(m.valid);
    EXPECT_NEAR(m.noise_cov(0, 0), 64.0,  1e-9);   // 8^2
    EXPECT_NEAR(m.noise_cov(1, 1), 144.0, 1e-9);   // 12^2
    EXPECT_NEAR(m.noise_cov(0, 1), 0.0,   1e-9);   // independent
}
