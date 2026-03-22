#include "real_track.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

namespace tracker {

// ── Flat-earth projection ─────────────────────────────────────────────────────
// Accurate to ~0.1% for tracks under 200 km. Sufficient for ADS-B validation.
static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double EARTH_RADIUS_M = 6371000.0;

static double latToY(double lat, double lat0) {
    return (lat - lat0) * DEG_TO_RAD * EARTH_RADIUS_M;
}

static double lonToX(double lon, double lon0, double lat0) {
    return (lon - lon0) * DEG_TO_RAD * EARTH_RADIUS_M
           * std::cos(lat0 * DEG_TO_RAD);
}

// ── CSV parsing helpers ───────────────────────────────────────────────────────
static std::vector<std::string> splitCSV(const std::string& line) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ','))
        tokens.push_back(tok);
    return tokens;
}

static double safeDouble(const std::string& s, double fallback = 0.0) {
    try { return std::stod(s); }
    catch (...) { return fallback; }
}

// ── Column index lookup ───────────────────────────────────────────────────────
static int colIdx(const std::vector<std::string>& header,
                  const std::string& name) {
    for (int i = 0; i < (int)header.size(); ++i)
        if (header[i] == name) return i;
    return -1;
}

// ── Main loader ───────────────────────────────────────────────────────────────
std::vector<RealTrackPoint> loadRealTrack(const std::string& csv_path,
                                           RealTrackMeta& meta) {
    std::ifstream f(csv_path);
    if (!f.is_open())
        throw std::runtime_error("Cannot open track CSV: " + csv_path);

    // Parse header
    std::string header_line;
    std::getline(f, header_line);
    auto header = splitCSV(header_line);

    // Required columns
    int iTime = colIdx(header, "time");
    int iLat  = colIdx(header, "lat");
    int iLon  = colIdx(header, "lon");
    int iAlt  = colIdx(header, "altitude_ft");
    if (iTime < 0 || iLat < 0 || iLon < 0 || iAlt < 0)
        throw std::runtime_error(
            "CSV missing required columns: time, lat, lon, altitude_ft");

    // Optional columns
    int iHdg  = colIdx(header, "heading_deg");
    int iIcao = colIdx(header, "icao24");
    int iCall = colIdx(header, "callsign");

    std::vector<RealTrackPoint> points;
    std::string line;
    int skipped = 0;

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto cols = splitCSV(line);
        if ((int)cols.size() < (int)header.size()) { ++skipped; continue; }

        // Skip rows with missing critical values
        if (cols[iTime].empty() || cols[iLat].empty() || cols[iLon].empty()) {
            ++skipped; continue;
        }

        RealTrackPoint p;
        p.time        = safeDouble(cols[iTime]);
        double lat    = safeDouble(cols[iLat]);
        double lon    = safeDouble(cols[iLon]);
        p.altitude_ft = safeDouble(cols[iAlt], 0.0);
        p.heading_deg = (iHdg  >= 0) ? safeDouble(cols[iHdg],  0.0) : 0.0;
        p.icao24      = (iIcao >= 0) ? cols[iIcao] : "";
        p.callsign    = (iCall >= 0) ? cols[iCall] : "";

        // Store raw lat/lon temporarily in x_m/y_m — projected below
        p.x_m = lat;
        p.y_m = lon;

        points.push_back(p);
    }

    if (points.empty())
        throw std::runtime_error("No valid track points found in: " + csv_path);

    // Sort by time
    std::sort(points.begin(), points.end(),
              [](const RealTrackPoint& a, const RealTrackPoint& b) {
                  return a.time < b.time;
              });

    // Project lat/lon to local XY centered on first point
    double lat0 = points[0].x_m;
    double lon0 = points[0].y_m;

    for (auto& p : points) {
        double lat = p.x_m;
        double lon = p.y_m;
        p.x_m = lonToX(lon, lon0, lat0);
        p.y_m = latToY(lat, lat0);
    }

    // Build metadata
    meta.icao24    = points[0].icao24;
    meta.callsign  = points[0].callsign;
    meta.ref_lat   = lat0;
    meta.ref_lon   = lon0;
    meta.duration_s = points.back().time - points[0].time;
    meta.n_points  = static_cast<int>(points.size());

    double alt_sum = 0.0;
    for (const auto& p : points) alt_sum += p.altitude_ft;
    meta.mean_alt_ft = alt_sum / points.size();

    if (skipped > 0)
        std::cout << "[RealTrack] Skipped " << skipped
                  << " malformed rows in " << csv_path << "\n";

    std::cout << "[RealTrack] Loaded " << meta.n_points << " points"
              << "  aircraft=" << meta.callsign
              << "  duration=" << meta.duration_s << "s"
              << "  mean_alt=" << meta.mean_alt_ft << " ft\n";

    return points;
}

// ── Convert to TargetState for stats comparison ───────────────────────────────
// For real ADS-B data there is no separate ground truth — the raw ADS-B
// positions ARE the reference. The Kalman filter smooths them.
std::vector<TargetState> realTrackToTargetStates(
    const std::vector<RealTrackPoint>& points) {

    std::vector<TargetState> states;
    states.reserve(points.size());

    for (int i = 0; i < (int)points.size(); ++i) {
        TargetState s;
        s.time      = points[i].time;
        s.state     = Eigen::Vector4d::Zero();
        s.state(0)  = points[i].x_m;
        s.state(1)  = points[i].y_m;

        // Estimate velocity from finite differences
        if (i > 0) {
            double dt = points[i].time - points[i-1].time;
            if (dt > 0.0) {
                s.state(2) = (points[i].x_m - points[i-1].x_m) / dt;
                s.state(3) = (points[i].y_m - points[i-1].y_m) / dt;
            }
        }
        states.push_back(s);
    }
    return states;
}

} // namespace tracker
