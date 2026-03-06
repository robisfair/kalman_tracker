#include "simulation.h"
#include "statistics.h"
#include <iostream>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

// Default config as JSON string (used when no config file is provided)
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

void printUsage(const char* prog) {
    std::cout << "\nUsage:\n"
              << "  " << prog << "                          # Run default scenario\n"
              << "  " << prog << " <config.json>            # Run from config file\n"
              << "  " << prog << " <config.json> <out.csv>  # Run and export CSV\n\n";
}

int main(int argc, char* argv[]) {
    std::cout << "===========================================\n";
    std::cout << "  Multi-Sensor Kalman Filter Target Tracker\n";
    std::cout << "===========================================\n";

    tracker::SimConfig cfg;

    if (argc >= 2) {
        std::string path = argv[1];
        if (path == "--help" || path == "-h") {
            printUsage(argv[0]);
            return 0;
        }
        try {
            cfg = tracker::SimConfig::fromFile(path);
            std::cout << "[Main] Loaded config: " << path << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[Error] Failed to load config: " << e.what() << "\n";
            return 1;
        }
    } else {
        // Use default embedded config
        nlohmann::json j = nlohmann::json::parse(DEFAULT_CONFIG);
        cfg = tracker::SimConfig::fromJSON(j);
        std::cout << "[Main] Using default scenario\n";
    }

    // Run simulation
    tracker::SimResult result;
    try {
        result = tracker::runSimulation(cfg);
    } catch (const std::exception& e) {
        std::cerr << "[Error] Simulation failed: " << e.what() << "\n";
        return 1;
    }

    // Print stats
    tracker::printSummary(result.scenario_name, result.stats);

    // Export CSV
    std::string csv_path = (argc >= 3) ? argv[2] : "data/output_" + result.scenario_name + ".csv";
    try {
        fs::create_directories(fs::path(csv_path).parent_path());
        tracker::exportCSV(csv_path, result.truth, result.estimates);
        std::cout << "[Main] Results exported to: " << csv_path << "\n\n";
    } catch (const std::exception& e) {
        std::cerr << "[Warning] CSV export failed: " << e.what() << "\n";
    }

    return 0;
}
