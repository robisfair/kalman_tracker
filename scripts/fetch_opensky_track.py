#!/usr/bin/env python3
"""
fetch_opensky_track.py
----------------------
Fetch a single aircraft's flight track from the OpenSky Network API
and export it as a CSV compatible with the C++ Kalman filter's --real-track mode.

Usage:
    python3 scripts/fetch_opensky_track.py --icao <icao24> [--time <unix_ts>] [--out <path>]

Examples:
    # Fetch the most recent track for a specific aircraft
    python3 scripts/fetch_opensky_track.py --icao a0f0b2

    # Fetch a historical track at a specific time (must be within last 30 days for free tier)
    python3 scripts/fetch_opensky_track.py --icao a0f0b2 --time 1700000000

    # Discover active aircraft over DFW TRACON and pick one
    python3 scripts/fetch_opensky_track.py --discover

Output CSV columns:
    time, lat, lon, altitude_ft, heading_deg, icao24, callsign
"""

import argparse
import csv
import os
import sys
import time
import requests
from datetime import datetime, timezone

# ── OpenSky endpoints ─────────────────────────────────────────────────────────
OPENSKY_BASE     = "https://opensky-network.org/api"
STATES_ENDPOINT  = f"{OPENSKY_BASE}/states/all"
TRACKS_ENDPOINT  = f"{OPENSKY_BASE}/tracks/all"

# DFW TRACON bounding box (default for --discover)
DEFAULT_BBOX = {
    "lamin": 32.0, "lamax": 33.5,
    "lomin": -98.0, "lomax": -96.0,
}

# ── Helpers ───────────────────────────────────────────────────────────────────

def fetch_track(icao24: str, time_s: int = 0) -> dict:
    """Fetch track for icao24. time_s=0 means most recent track."""
    params = {"icao24": icao24.lower()}
    if time_s > 0:
        params["time"] = time_s

    print(f"[OpenSky] Fetching track for {icao24}...")
    resp = requests.get(TRACKS_ENDPOINT, params=params, timeout=20)

    if resp.status_code == 404:
        raise RuntimeError(
            f"No track found for {icao24}. "
            "The aircraft may be on the ground or not recently active."
        )
    resp.raise_for_status()
    return resp.json()


def parse_track(data: dict) -> list[dict]:
    """Parse OpenSky track response into a list of position dicts."""
    icao24   = data.get("icao24", "")
    callsign = (data.get("callsign") or "UNKNOWN").strip()
    path     = data.get("path", []) or []

    if not path:
        raise RuntimeError("Track response contains no path data.")

    rows = []
    for point in path:
        # path element: [time, lat, lon, baro_alt_m, heading, on_ground]
        if point[1] is None or point[2] is None:
            continue
        alt_m = point[3] if point[3] is not None else 0.0
        rows.append({
            "time":         int(point[0]),
            "lat":          float(point[1]),
            "lon":          float(point[2]),
            "altitude_ft":  round(float(alt_m) * 3.28084, 1),
            "heading_deg":  float(point[4]) if point[4] is not None else 0.0,
            "icao24":       icao24,
            "callsign":     callsign,
            "on_ground":    bool(point[5]) if len(point) > 5 else False,
        })

    # Filter out ground segments
    airborne = [r for r in rows if not r["on_ground"] and r["altitude_ft"] > 500]
    print(f"[Parser]   {len(rows)} total points, {len(airborne)} airborne (>500 ft)")
    return airborne


def discover_aircraft(bbox: dict, n: int = 10) -> list[dict]:
    """Fetch current state vectors over a bounding box and return top n aircraft."""
    print(f"[OpenSky] Discovering active aircraft over bbox {bbox}...")
    resp = requests.get(STATES_ENDPOINT, params=bbox, timeout=15)
    resp.raise_for_status()
    data   = resp.json()
    states = data.get("states", []) or []

    aircraft = []
    for sv in states:
        if sv[8]:    # on_ground
            continue
        if sv[5] is None or sv[6] is None or sv[7] is None:
            continue
        aircraft.append({
            "icao24":      sv[0],
            "callsign":    (sv[1] or "UNKNOWN").strip(),
            "lat":         sv[6],
            "lon":         sv[5],
            "altitude_ft": round((sv[7] or 0) * 3.28084, 0),
            "velocity_kts": round((sv[9] or 0) * 1.944, 0),
        })

    # Sort by altitude (higher = more likely en-route = more interesting track)
    aircraft.sort(key=lambda a: a["altitude_ft"], reverse=True)
    return aircraft[:n]


def save_csv(rows: list[dict], out_path: str) -> None:
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    fieldnames = ["time","lat","lon","altitude_ft","heading_deg","icao24","callsign"]
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    print(f"[Output]   Saved {len(rows)} rows -> {out_path}")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Fetch ADS-B track from OpenSky for Kalman filter validation")
    parser.add_argument("--icao",     type=str, help="ICAO24 hex address of aircraft")
    parser.add_argument("--time",     type=int, default=0,
                        help="Unix timestamp for historical track (0 = most recent)")
    parser.add_argument("--out",      type=str, default="",
                        help="Output CSV path (default: data/track_<icao>.csv)")
    parser.add_argument("--discover", action="store_true",
                        help="Discover active aircraft over DFW TRACON and list them")
    args = parser.parse_args()

    # ── Discover mode ─────────────────────────────────────────────────────────
    if args.discover:
        aircraft = discover_aircraft(DEFAULT_BBOX)
        if not aircraft:
            print("No airborne aircraft found in bbox. Try again during peak hours.")
            return

        print(f"\n{'ICAO24':<10} {'Callsign':<12} {'Alt (ft)':<10} {'Speed (kts)'}")
        print("-" * 50)
        for ac in aircraft:
            print(f"{ac['icao24']:<10} {ac['callsign']:<12} "
                  f"{ac['altitude_ft']:<10.0f} {ac['velocity_kts']:.0f}")
        print(f"\nPick an ICAO24 and run:")
        print(f"  python3 scripts/fetch_opensky_track.py --icao <icao24>\n")
        return

    # ── Track fetch mode ──────────────────────────────────────────────────────
    if not args.icao:
        parser.print_help()
        sys.exit(1)

    try:
        data  = fetch_track(args.icao, args.time)
        rows  = parse_track(data)
    except Exception as e:
        print(f"[Error] {e}")
        sys.exit(1)

    if not rows:
        print("[Error] No airborne track points found. Try a different aircraft or time.")
        sys.exit(1)

    # Print summary
    t0  = datetime.fromtimestamp(rows[0]["time"],  tz=timezone.utc)
    t1  = datetime.fromtimestamp(rows[-1]["time"], tz=timezone.utc)
    dur = rows[-1]["time"] - rows[0]["time"]
    print(f"\n── Track Summary ─────────────────────────────────")
    print(f"  Aircraft:   {rows[0]['callsign']} ({args.icao})")
    print(f"  Start:      {t0.strftime('%Y-%m-%d %H:%M:%S')} UTC")
    print(f"  End:        {t1.strftime('%Y-%m-%d %H:%M:%S')} UTC")
    print(f"  Duration:   {dur}s ({dur/60:.1f} min)")
    print(f"  Points:     {len(rows)}")
    print(f"  Alt range:  {min(r['altitude_ft'] for r in rows):.0f} – "
          f"{max(r['altitude_ft'] for r in rows):.0f} ft")
    print(f"──────────────────────────────────────────────────\n")

    # Save
    callsign = rows[0]["callsign"].replace(" ", "_") or args.icao
    out_path = args.out or f"data/track_{callsign}_{args.icao}.csv"
    save_csv(rows, out_path)

    print(f"\nRun Kalman filter against this track:")
    print(f"  ./build/kalman_tracker --real-track {out_path}")
    print(f"  ./build/kalman_tracker --real-track {out_path} --process-noise 1.0")
    print(f"  ./build/kalman_tracker --real-track {out_path} --adsb-noise 50\n")


if __name__ == "__main__":
    main()
