#!/usr/bin/env python3
"""
generate_synthetic_gtfs.py — Minimal GTFS fixture for offline testing.

Usage:
    python3 scripts/generate_synthetic_gtfs.py ./data

Generates a valid 10-station, 3-route GTFS dataset that satisfies all
constraints the parser expects. Use this if all real BMRCL GTFS sources fail.

The generated benchmark numbers are NOT resume-worthy — they reflect a
trivial synthetic topology. Use this only to verify the engine builds and
runs correctly while you hunt for real GTFS data.

Generated files: agency.txt, stops.txt, routes.txt, trips.txt,
                 stop_times.txt, calendar.txt, calendar_dates.txt
"""

import os
import sys
import math

def write(path, content):
    with open(path, 'w') as f:
        f.write(content)
    print(f"  wrote {path}")

def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else "./data"
    os.makedirs(data_dir, exist_ok=True)

    print(f"Generating synthetic GTFS in {data_dir}/")

    # ── agency.txt ───────────────────────────────────────────────────────────
    write(os.path.join(data_dir, "agency.txt"),
        "agency_id,agency_name,agency_url,agency_timezone\n"
        "SYNTHETIC,Synthetic Metro,https://example.com,Asia/Kolkata\n"
    )

    # ── stops.txt ────────────────────────────────────────────────────────────
    # 10 stations laid out along a straight line (lat increases by 0.01 per stop)
    stops_lines = ["stop_id,stop_name,stop_lat,stop_lon,stop_desc"]
    for i in range(10):
        lat = 12.9716 + i * 0.01
        stops_lines.append(f"S{i:02d},Station_{i},{lat:.4f},77.5946,Synthetic stop {i}")
    write(os.path.join(data_dir, "stops.txt"), "\n".join(stops_lines) + "\n")

    # ── routes.txt ───────────────────────────────────────────────────────────
    # Two routes, not one. On a single linear chain the Pareto frontier never
    # exceeds size 2 at any node, so insert_and_dominate's forward-pruning loop
    # is never invoked with more than one label to evict — an implementation that
    # inserts without pruning would pass the oracle silently. R_EXPRESS
    # (S00→S04→S09 via the midpoint) carries a different crowd profile, so the
    # frontier at S09 requires active multi-label forward pruning to be correct.
    write(os.path.join(data_dir, "routes.txt"),
        "route_id,agency_id,route_short_name,route_long_name,route_type\n"
        "R_PURPLE,SYNTHETIC,Purple,Purple Line,1\n"
        "R_EXPRESS,SYNTHETIC,Express,Express Line,1\n"
    )

    # ── trips.txt ────────────────────────────────────────────────────────────
    # 5 Purple Line trips throughout the day: 07:00, 08:00, 09:00, 12:00, 17:00
    # 3 Express trips: 07:30, 08:30, 17:30 — fewer stops, lower crowd off-peak,
    # higher crowd during rush. This creates genuine multi-label frontiers at S09
    # that require insert_and_dominate forward pruning to evict dominated labels.
    departure_hours = [7, 8, 9, 12, 17]
    express_hours = [7, 8, 17]  # rush-hour expresses
    trips_lines = ["route_id,service_id,trip_id,trip_headsign,direction_id"]
    for h in departure_hours:
        trips_lines.append(f"R_PURPLE,WEEKDAY,TRIP_{h:02d}h,Station_9_bound,0")
    for h in express_hours:
        trips_lines.append(f"R_EXPRESS,WEEKDAY,EXPR_{h:02d}h,Station_9_express,0")
    write(os.path.join(data_dir, "trips.txt"), "\n".join(trips_lines) + "\n")

    # ── stop_times.txt ────────────────────────────────────────────────────────
    # Stops carry a 30s dwell rather than arrival == departure: some GTFS
    # validators enforce arrival < departure at intermediate stops.
    #   arrival = base + i*180, departure = base + i*180 + 30
    # This also yields travel_time = arr[i+1] - dep[i] = 180 - 30 = 150s per
    # segment, which is more realistic than 180s.
    #
    # Express trips: stop only at S00 (origin), S04 (midpoint), S09 (terminus).
    # Lower crowd weight than Purple Line off-peak (h=7,8 before Gaussian peak),
    # higher crowd during evening rush (h=17 near peak). This ensures the Pareto
    # frontier at S09 has genuinely non-dominated paths requiring forward pruning.
    stop_times_lines = [
        "trip_id,arrival_time,departure_time,stop_id,stop_sequence,pickup_type,drop_off_type"
    ]

    # Purple Line: all 10 stops with 30s dwell
    for h in departure_hours:
        base_seconds = h * 3600
        for i in range(10):
            arr_s  = base_seconds + i * 180
            dep_s  = arr_s + (30 if 0 < i < 9 else 0)  # no dwell at terminals
            def fmt(s):
                return f"{s//3600:02d}:{(s%3600)//60:02d}:{s%60:02d}"
            stop_times_lines.append(
                f"TRIP_{h:02d}h,{fmt(arr_s)},{fmt(dep_s)},S{i:02d},{i},0,0"
            )

    # Express Line: S00 → S04 → S09 with 8-minute segment travel time
    # secondary_weight injected via stop_times headsign field is not standard —
    # crowd weights come from GraphBuilder using time-of-day Gaussian.
    # We control crowd indirectly via departure time (earlier = lower crowd).
    express_segment_travel = 8 * 60  # 8 min per express segment (vs 6×3=18 for Purple)
    for h in express_hours:
        base_seconds = h * 3600 + 1800  # departs 30min after Purple Line hour
        stops_served = [0, 4, 9]
        for seq, i in enumerate(stops_served):
            arr_s = base_seconds + seq * express_segment_travel
            dep_s = arr_s + (30 if seq == 1 else 0)  # dwell only at midpoint
            def fmt(s):
                return f"{s//3600:02d}:{(s%3600)//60:02d}:{s%60:02d}"
            stop_times_lines.append(
                f"EXPR_{h:02d}h,{fmt(arr_s)},{fmt(dep_s)},S{i:02d},{seq},0,0"
            )

    write(os.path.join(data_dir, "stop_times.txt"),
          "\n".join(stop_times_lines) + "\n")

    # ── calendar.txt ──────────────────────────────────────────────────────────
    write(os.path.join(data_dir, "calendar.txt"),
        "service_id,monday,tuesday,wednesday,thursday,friday,saturday,sunday,"
        "start_date,end_date\n"
        "WEEKDAY,1,1,1,1,1,0,0,20260601,20260831\n"
    )

    # ── calendar_dates.txt ───────────────────────────────────────────────────
    write(os.path.join(data_dir, "calendar_dates.txt"),
        "service_id,date,exception_type\n"
        "WEEKDAY,20260815,2\n"  # Independence Day — service removed
    )

    print("\nDone. Synthetic GTFS generated.")
    print("WARNING: These numbers are NOT resume-worthy.")
    print("         Use only to verify the engine runs while hunting for real GTFS data.")

if __name__ == "__main__":
    main()
