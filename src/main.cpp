#include "simulation.h"
#include "statistics.h"
#include "real_track.h"
#include "kalman_filter.h"
#include <iostream>
#include <string>
#include <filesystem>
#include <sstream>

namespace fs = std::filesystem;

// ── Default embedded config ───────────────────────────────────────────────────
static const char* DEFAULT_CONFIG = R"({
    "scenario_name": "default_linear",
    "duration_s": 60.0,
    "dt_s": 0.5,
    "process_noise_std": 0.5,
    "motion_model": "linear",
    "sensors": [
        {
            "id": "Radar",
            "pos_std_x": 15.0,
            "pos_std_y": 15.0,
            "detection_prob": 0.95,
            "dropout_prob": 0.02
        },
        {
            "id": "IR",
            "pos_std_x": 25.0,
            "pos_std_y": 10.0,
            "detection_prob": 0.85,
            "dropout_prob": 0.05
        },
        {
            "id": "Acoustic",
            "pos_std_x": 40.0,
            "pos_std_y": 40.0,
            "detection_prob": 0.70,
            "dropout_prob": 0.10
        }
    ]
})";

// ── ADS-B position accuracy ───────────────────────────────────────────────────
// ADS-B NACp (Navigation Accuracy Category for Position):
//   NACp 9 = HPL < 30m   (most modern aircraft)
//   NACp 8 = HPL < 186m  (older equipment)
// We use 100m as a conservative estimate for the measurement noise std dev.
static constexpr double ADSB_POS_STD_M = 100.0;

void printUsage(const char* prog) {
    std::cout << "\nUsage:\n"
              << "  " << prog << "                                   # Simulated (default scenario)\n"
              << "  " << prog << " <config.json>                     # Simulated (custom config)\n"
              << "  " << prog << " <config.json> <out.csv>           # Simulated + export CSV\n"
              << "  " << prog << " --real-track <track.csv>          # Real ADS-B track\n"
              << "  " << prog << " --real-track <track.csv> <out.csv># Real ADS-B track + export\n\n"
              << "Options:\n"
              << "  --real-track <file>   Run filter against real ADS-B CSV track\n"
              << "                        (produced by scripts/fetch_opensky_track.py)\n"
              << "  --process-noise <val> Override process noise std dev (default: 0.5)\n"
              << "  --adsb-noise <val>    Override ADS-B position noise std dev in meters\n"
              << "                        (default: " << ADSB_POS_STD_M << ")\n\n";
}

// ── Real-track mode ───────────────────────────────────────────────────────────
static int runRealTrack(const std::string& track_csv,
                        const std::string& out_csv,
                        double process_noise_std,
                        double adsb_noise_m) {
    // 1. Load track
    tracker::RealTrackMeta meta;
    std::vector<tracker::RealTrackPoint> points;
    try {
        points = tracker::loadRealTrack(track_csv, meta);
    } catch (const std::exception& e) {
        std::cerr << "[Error] " << e.what() << "\n";
        return 1;
    }

    // 2. Print track summary
    std::cout << "\n── Real ADS-B Track ──────────────────────────────\n";
    std::cout << "  Aircraft:     " << meta.callsign
              << " (" << meta.icao24 << ")\n";
    std::cout << "  Track points: " << meta.n_points << "\n";
    std::cout << "  Duration:     " << meta.duration_s << " s ("
              << meta.duration_s / 60.0 << " min)\n";
    std::cout << "  Mean alt:     " << meta.mean_alt_ft << " ft\n";
    std::cout << "  Ref origin:   " << meta.ref_lat << "°N  "
              << meta.ref_lon << "°E\n";
    std::cout << "  ADS-B noise:  ±" << adsb_noise_m << " m (1σ)\n";
    std::cout << "──────────────────────────────────────────────────\n\n";

    // 3. Build measurement noise covariance from ADS-B accuracy
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0, 0) = adsb_noise_m * adsb_noise_m;
    R(1, 1) = adsb_noise_m * adsb_noise_m;

    // 4. Run Kalman filter over real track points
    tracker::KalmanFilter kf(process_noise_std);

    for (const auto& p : points) {
        Eigen::Vector2d z(p.x_m, p.y_m);

        if (!kf.isInitialized()) {
            kf.initialize(p.time, z, R);
            continue;
        }
        kf.update(p.time, z, R);
    }

    const auto& estimates = kf.history();
    std::cout << "[Filter] Processed " << estimates.size()
              << " updates on real track\n";

    // 5. Compute stats
    // For real data: raw ADS-B positions are the reference ("truth")
    // RMSE measures how much the filter smooths vs raw input
    auto truth_states = tracker::realTrackToTargetStates(points);
    auto stats = tracker::computeStats(estimates, truth_states);

    std::string identifier = meta.callsign.empty() ? meta.icao24 : meta.callsign;
    std::string scenario_name = "real_" + identifier;
    tracker::printSummary(scenario_name, stats);

    std::cout << "── Interpretation (Real ADS-B Mode) ─────────────\n";
    std::cout << "  RMSE here measures filter smoothing vs raw ADS-B.\n";
    std::cout << "  Lower RMSE = filter closely follows raw positions.\n";
    std::cout << "  Higher RMSE = filter is smoothing out ADS-B noise.\n";
    std::cout << "  Tune --process-noise to balance responsiveness vs\n";
    std::cout << "  smoothing. Try values: 0.1 (smooth) to 5.0 (tight).\n";
    std::cout << "──────────────────────────────────────────────────\n\n";

    // 6. Export CSV
    std::string csv_path = out_csv.empty()
        ? "data/output_" + scenario_name + ".csv"
        : out_csv;

    try {
        fs::create_directories(fs::path(csv_path).parent_path());
        tracker::exportCSV(csv_path, truth_states, estimates);
        std::cout << "[Main] Results exported to: " << csv_path << "\n\n";
    } catch (const std::exception& e) {
        std::cerr << "[Warning] CSV export failed: " << e.what() << "\n";
    }

    return 0;
}

// ── Entry point ───────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    std::cout << "===========================================\n";
    std::cout << "  Multi-Sensor Kalman Filter Target Tracker\n";
    std::cout << "===========================================\n";

    // Parse args
    bool        real_mode        = false;
    std::string real_track_path;
    std::string out_csv_path;
    std::string config_path;
    double      process_noise    = 0.5;
    double      adsb_noise       = ADSB_POS_STD_M;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--real-track" && i + 1 < argc) {
            real_mode       = true;
            real_track_path = argv[++i];
        } else if (arg == "--process-noise" && i + 1 < argc) {
            process_noise = std::stod(argv[++i]);
        } else if (arg == "--adsb-noise" && i + 1 < argc) {
            adsb_noise = std::stod(argv[++i]);
        } else if (arg[0] != '-' && config_path.empty() && !real_mode) {
            config_path = arg;
        } else if (arg[0] != '-' && out_csv_path.empty()) {
            out_csv_path = arg;
        }
    }

    // ── Real track mode ───────────────────────────────────────────────────────
    if (real_mode) {
        if (real_track_path.empty()) {
            std::cerr << "[Error] --real-track requires a CSV file path\n";
            printUsage(argv[0]);
            return 1;
        }
        std::cout << "[Main] Mode: Real ADS-B Track\n";
        std::cout << "[Main] Track file: " << real_track_path << "\n";
        std::cout << "[Main] Process noise std: " << process_noise << "\n";
        return runRealTrack(real_track_path, out_csv_path,
                            process_noise, adsb_noise);
    }

    // ── Simulated mode ────────────────────────────────────────────────────────
    std::cout << "[Main] Mode: Simulated\n";

    tracker::SimConfig cfg;
    if (!config_path.empty()) {
        try {
            cfg = tracker::SimConfig::fromFile(config_path);
            std::cout << "[Main] Loaded config: " << config_path << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[Error] Failed to load config: " << e.what() << "\n";
            return 1;
        }
    } else {
        nlohmann::json j = nlohmann::json::parse(DEFAULT_CONFIG);
        cfg = tracker::SimConfig::fromJSON(j);
        std::cout << "[Main] Using default scenario\n";
    }

    tracker::SimResult result;
    try {
        result = tracker::runSimulation(cfg);
    } catch (const std::exception& e) {
        std::cerr << "[Error] Simulation failed: " << e.what() << "\n";
        return 1;
    }

    tracker::printSummary(result.scenario_name, result.stats);

    std::string csv_path = out_csv_path.empty()
        ? "data/output_" + result.scenario_name + ".csv"
        : out_csv_path;

    try {
        fs::create_directories(fs::path(csv_path).parent_path());
        tracker::exportCSV(csv_path, result.truth, result.estimates);
        std::cout << "[Main] Results exported to: " << csv_path << "\n\n";
    } catch (const std::exception& e) {
        std::cerr << "[Warning] CSV export failed: " << e.what() << "\n";
    }

    return 0;
}
