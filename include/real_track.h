#pragma once
#include "target.h"
#include <string>
#include <vector>

namespace tracker {

/// A single position fix parsed from a real ADS-B track CSV.
struct RealTrackPoint {
    double time;          ///< Unix timestamp (seconds)
    double x_m;           ///< Local X position in meters (projected from lat/lon)
    double y_m;           ///< Local Y position in meters (projected from lat/lon)
    double altitude_ft;   ///< Barometric altitude in feet
    double heading_deg;   ///< True heading in degrees (0 if unavailable)
    std::string icao24;
    std::string callsign;
};

/// Metadata about the real track (computed on load)
struct RealTrackMeta {
    std::string icao24;
    std::string callsign;
    double      ref_lat;       ///< Latitude of projection origin (first point)
    double      ref_lon;       ///< Longitude of projection origin (first point)
    double      duration_s;    ///< Total track duration in seconds
    double      mean_alt_ft;   ///< Mean barometric altitude
    int         n_points;
};

/// Loads a real ADS-B track CSV produced by scripts/fetch_opensky_track.py.
///
/// Expected columns: time, lat, lon, altitude_ft, heading_deg, icao24, callsign
/// The loader projects lat/lon into local XY (meters) using a flat-earth
/// approximation centered on the first point — accurate to within ~0.1% for
/// tracks under 200 km.
///
/// @param csv_path   Path to the CSV file
/// @param out_meta   Populated with track metadata on success
/// @returns          Vector of RealTrackPoints sorted by time
std::vector<RealTrackPoint> loadRealTrack(const std::string& csv_path,
                                           RealTrackMeta& out_meta);

/// Convert a real track into TargetState objects (for stats comparison).
/// Since we have no ground truth for real data, position is used as both
/// "truth" and measurement — the filter smooths the noisy ADS-B fixes.
std::vector<TargetState> realTrackToTargetStates(
    const std::vector<RealTrackPoint>& points);

} // namespace tracker
