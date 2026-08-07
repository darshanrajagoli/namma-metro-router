#!/usr/bin/env python3
"""
build_namma_metro_gtfs.py — build a routable GTFS feed for the REAL Namma Metro
(Bengaluru / BMRCL) network, in the exact positional layout this project's C++
parser expects.

DATA PROVENANCE (honest labelling)
----------------------------------
Station names, codes, line membership, geo-coordinates and inter-station
distances are the REAL BMRCL network, taken from the CC0 (public-domain) dataset:
    https://github.com/Vinayak-Chinchakhandi/Bengaluru-Metro-Network-Dataset
BMRCL does not publish an open GTFS *timetable*, so the schedule (train
departures + run times) is SYNTHESISED here from a fixed headway and an average
line speed applied to the real distances. So: real topology + real distances +
modelled timetable. State this plainly on your resume/README — it is honest and
still exercises the engine on the actual Bengaluru metro graph.

USAGE
-----
    python3 scripts/build_namma_metro_gtfs.py <network_csv> <output_dir> \
            [--headway 300] [--speed 35] [--dwell 20] [--start 05:00] [--end 23:00]

    python3 scripts/build_namma_metro_gtfs.py raw_namma/bengaluru_metro_network.csv data
    #                                          ^ real BMRCL network      ^ engine reads this

Then: taskset -c 3 ./build/routing_engine_benchmark ./data
"""

import csv
import math
import os
import sys


def die(m):
    print(f"ERROR: {m}", file=sys.stderr)
    sys.exit(1)


def hhmm_to_sec(s):
    h, m = s.split(":")
    return int(h) * 3600 + int(m) * 60


def sec_to_gtfs(t):
    return f"{t // 3600:02d}:{(t % 3600) // 60:02d}:{t % 60:02d}"


def haversine_m(a, b):
    R = 6371000.0
    dl = math.radians(b[0] - a[0])
    do = math.radians(b[1] - a[1])
    x = (math.sin(dl / 2) ** 2
         + math.cos(math.radians(a[0])) * math.cos(math.radians(b[0])) * math.sin(do / 2) ** 2)
    return 2 * R * math.atan2(math.sqrt(x), math.sqrt(1 - x))


def main():
    pos = [a for a in sys.argv[1:] if not a.startswith("--")]
    opt = {}
    it = iter([a for a in sys.argv[1:] if a.startswith("--")])
    # parse --key value pairs from the tail
    raw = sys.argv[1:]
    i = 0
    while i < len(raw):
        if raw[i].startswith("--"):
            key = raw[i][2:]
            val = raw[i + 1] if i + 1 < len(raw) else ""
            opt[key] = val
            i += 2
        else:
            i += 1
    pos = [a for a in raw if not a.startswith("--") and a not in opt.values()]
    if len(pos) < 2:
        die("usage: build_namma_metro_gtfs.py <network_csv> <output_dir> [--headway 300] [--speed 35] [--dwell 20] [--start 05:00] [--end 23:00]")

    net_csv, out = pos[0], pos[1]
    headway = int(opt.get("headway", 300))     # seconds between trains
    speed_kmh = float(opt.get("speed", 35))     # average line speed incl. accel/decel
    dwell = int(opt.get("dwell", 20))           # dwell seconds at each intermediate stop
    start_s = hhmm_to_sec(opt.get("start", "05:00"))
    end_s = hhmm_to_sec(opt.get("end", "23:00"))
    os.makedirs(out, exist_ok=True)

    # ── read the real network ───────────────────────────────────────────────
    with open(net_csv, "r", encoding="utf-8-sig", newline="") as f:
        rows = [ {k.strip(): (v.strip() if v else "") for k, v in r.items()}
                 for r in csv.DictReader(f) ]
    if not rows:
        die(f"empty network csv: {net_csv}")

    # ── merge stations that are physically the same (interchanges) by proximity
    # Node identity = cluster of station rows within 250 m. This guarantees the
    # Purple/Green/Yellow lines actually connect at interchanges even if their
    # per-line station_codes differ.
    raw_stations = {}   # code -> (name, lat, lon)
    for r in rows:
        code = r["station_code"]
        try:
            lat, lon = float(r["latitude"]), float(r["longitude"])
        except ValueError:
            continue
        raw_stations.setdefault(code, (r["station_name"], lat, lon))

    canon = {}          # code -> canonical code
    canon_info = {}     # canonical code -> (name, lat, lon)
    for code, (name, lat, lon) in raw_stations.items():
        merged = None
        for ccode, (cn, cla, clo) in canon_info.items():
            if haversine_m((lat, lon), (cla, clo)) < 250.0:
                merged = ccode
                break
        if merged is None:
            canon[code] = code
            canon_info[code] = (name, lat, lon)
        else:
            canon[code] = merged

    def node(code):
        return canon.get(code, code)

    # ── group rows into per-line ordered sequences ──────────────────────────
    lines = {}
    for r in rows:
        try:
            seq = int(r["sequence"])
        except ValueError:
            continue
        lines.setdefault(r["line"], []).append((seq, r))
    for ln in lines:
        lines[ln].sort(key=lambda x: x[0])

    def seg_seconds(dist_km):
        # run time between two stations from distance + average speed, plus dwell
        run = (dist_km / speed_kmh) * 3600.0 if dist_km > 0 else 30.0
        return int(round(run)) + dwell

    # ── emit files in the engine's exact positional order ───────────────────
    def w(name, header, out_rows):
        with open(os.path.join(out, name), "w", encoding="utf-8", newline="\n") as f:
            f.write(header + "\n")
            for row in out_rows:
                f.write(",".join(str(x) for x in row) + "\n")

    w("agency.txt", "agency_id,agency_name,agency_url,agency_timezone",
      [("METROB", "Namma Metro", "https://english.bmrc.co.in", "Asia/Kolkata")])

    # stops = canonical nodes
    w("stops.txt", "stop_id,stop_name,stop_lat,stop_lon,stop_desc",
      [(c, info[0].replace(",", " "), info[1], info[2], "")
       for c, info in canon_info.items()])

    # routes = one per line
    def route_id(ln):
        return ln.replace(" ", "_")
    w("routes.txt", "route_id,agency_id,route_short_name,route_long_name,route_type",
      [(route_id(ln), "METROB", ln.split()[0], ln, "1") for ln in lines])

    trips_out = []
    st_out = []
    trip_n = 0
    for ln, seq_rows in lines.items():
        ordered = [r for (_, r) in seq_rows]
        # travel seconds between consecutive stations (segment i -> i+1)
        segs = []
        for a, b in zip(ordered, ordered[1:]):
            try:
                dist = float(a.get("distance_to_next_km", "") or 0)
            except ValueError:
                dist = 0.0
            if dist <= 0:  # fall back to geodesic if distance missing
                dist = haversine_m((float(a["latitude"]), float(a["longitude"])),
                                   (float(b["latitude"]), float(b["longitude"]))) / 1000.0
            segs.append(seg_seconds(dist))

        node_seq = [node(r["station_code"]) for r in ordered]

        # both directions
        for direction, stations, seg_times in (
            (0, node_seq, segs),
            (1, list(reversed(node_seq)), list(reversed(segs))),
        ):
            t0 = start_s
            while t0 <= end_s:
                trip_n += 1
                tid = f"{route_id(ln)}_{direction}_{t0}"
                trips_out.append((route_id(ln), "WEEKDAY", tid))
                t = t0
                st_out.append((tid, sec_to_gtfs(t), sec_to_gtfs(t), stations[0], 1))
                for idx in range(1, len(stations)):
                    t += seg_times[idx - 1]
                    arr = t
                    dep = t + (dwell if idx < len(stations) - 1 else 0)
                    st_out.append((tid, sec_to_gtfs(arr), sec_to_gtfs(dep), stations[idx], idx + 1))
                    t = dep
                t0 += headway

    w("trips.txt", "route_id,service_id,trip_id", trips_out)
    w("stop_times.txt", "trip_id,arrival_time,departure_time,stop_id,stop_sequence", st_out)
    w("calendar.txt",
      "service_id,monday,tuesday,wednesday,thursday,friday,saturday,sunday,start_date,end_date",
      [("WEEKDAY", 1, 1, 1, 1, 1, 1, 1, "20200101", "20301231")])

    print("build_namma_metro_gtfs.py — done (REAL BMRCL topology, modelled timetable).")
    print(f"  lines          : {', '.join(lines.keys())}")
    print(f"  stations (nodes): {len(canon_info)}  (merged interchanges within 250 m)")
    print(f"  trips generated : {len(trips_out)}   (headway {headway}s, speed {speed_kmh} km/h, dwell {dwell}s, {opt.get('start','05:00')}-{opt.get('end','23:00')})")
    print(f"  stop_times      : {len(st_out)}")
    print(f"  -> wrote GTFS to: {out}/")
    print(f"  run: taskset -c 3 ./build/routing_engine_benchmark ./{out}")


if __name__ == "__main__":
    main()
