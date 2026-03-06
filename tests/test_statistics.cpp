#include <gtest/gtest.h>
#include "statistics.h"
#include <cmath>
#include <fstream>

using namespace tracker;

// Helper: make a fake estimate at a given time/position
static KalmanFilter::Estimate makeEst(double t, double x, double y) {
    KalmanFilter::Estimate e;
    e.time          = t;
    e.state         = Eigen::Vector4d(x, y, 0.0, 0.0);
    e.state_prior   = e.state;
    e.cov           = Eigen::Matrix4d::Identity();
    e.innovation_x  = 0.0;
    e.innovation_y  = 0.0;
    return e;
}

static TargetState makeTruth(double t, double x, double y) {
    TargetState s;
    s.time  = t;
    s.state = Eigen::Vector4d(x, y, 10.0, 10.0);
    return s;
}

TEST(StatisticsTest, ZeroErrorWhenPerfectEstimate) {
    std::vector<KalmanFilter::Estimate> ests = { makeEst(0.5, 100.0, 200.0) };
    std::vector<TargetState>            truth = { makeTruth(0.5, 100.0, 200.0) };

    auto stats = computeStats(ests, truth);
    EXPECT_NEAR(stats.rmse_position, 0.0, 1e-6);
    EXPECT_NEAR(stats.peak_error,    0.0, 1e-6);
}

TEST(StatisticsTest, CorrectRMSEForKnownError) {
    // 3-4-5 right triangle: pos error = 5.0
    std::vector<KalmanFilter::Estimate> ests  = { makeEst(1.0, 103.0, 204.0) };
    std::vector<TargetState>            truth = { makeTruth(1.0, 100.0, 200.0) };

    auto stats = computeStats(ests, truth);
    EXPECT_NEAR(stats.rmse_position, 5.0, 1e-4);
    EXPECT_NEAR(stats.peak_error,    5.0, 1e-4);
}

TEST(StatisticsTest, ThrowsOnEmptyEstimates) {
    std::vector<KalmanFilter::Estimate> ests;
    std::vector<TargetState>            truth = { makeTruth(0.0, 0.0, 0.0) };
    EXPECT_THROW(computeStats(ests, truth), std::runtime_error);
}

TEST(StatisticsTest, ExportsCSVFile) {
    std::string path = "/tmp/test_export.csv";
    std::vector<KalmanFilter::Estimate> ests  = { makeEst(1.0, 10.0, 20.0) };
    std::vector<TargetState>            truth = { makeTruth(1.0, 10.0, 20.0) };

    EXPECT_NO_THROW(exportCSV(path, truth, ests));

    std::ifstream f(path);
    ASSERT_TRUE(f.is_open());

    std::string header;
    std::getline(f, header);
    EXPECT_NE(header.find("truth_x"), std::string::npos);
    EXPECT_NE(header.find("est_x"),   std::string::npos);
}
