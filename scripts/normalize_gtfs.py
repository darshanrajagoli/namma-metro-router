#!/usr/bin/env python3
"""
normalize_gtfs.py — turn ANY real-world GTFS feed into the exact, clean,
positional layout that this project's C++ parser (src/gtfs_parser.cpp) expects.

WHY THIS EXISTS
---------------
The C++ parser reads GTFS by *fixed column position*, not by header name:
    stops.txt       -> stop_id, stop_name, stop_lat, stop_lon, stop_desc, ...
    routes.txt      -> route_id, agency_id, route_short_name, route_long_name, route_type
    trips.txt       -> route_id, service_id, trip_id, ...
    stop_times.txt  -> trip_id, arrival_time, departure_time, stop_id, stop_sequence
    agency.txt      -> agency_id, agency_name, agency_url, agency_timezone
Real feeds put those columns in different orders and add extra ones, so the raw
feed gets rejected (stops dropped -> every stop_time fails its FK -> synthetic
fallback). This script reads the raw feed *by header* and rewrites those five
files in the precise positional order above, so the engine ingests them cleanly.

It also (by default) ISOLATES THE METRO: keeps only rail routes (route_type in
0,1,2,5,7,11,12 — tram/subway/rail/monorail...), drops buses/ferries, collapses
platform stops onto their parent station, and drops any now-orphaned rows so the
foreign-key drop rate is ~0.

USAGE
-----
    python3 scripts/normalize_gtfs.py  <raw_feed_dir>  <output_dir>
    python3 scripts/normalize_gtfs.py  raw_bart        data          # metro only (default)
    python3 scripts/normalize_gtfs.py  raw_bmtc        data  --all   # keep every mode (e.g. buses)
    python3 scripts/normalize_gtfs.py  raw_feed        data  --no-collapse

<raw_feed_dir> is a folder containing the unzipped .txt files of a real GTFS feed.
<output_dir>   is where the clean files are written (point the engine at this).
"""

import csv
import os
import sys

# GTFS route_type values that are rail / metro-like (everything except bus=3, ferry=4).
RAIL_TYPES = {"0", "1", "2", "5", "7", "11", "12"}


def die(msg):
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def read_table(path):
    """Read a GTFS .txt as a list of dicts keyed by (BOM-stripped, trimmed) header."""
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        rows = []
        for row in reader:
            rows.append({(k.strip() if k else k): (v.strip() if v else "")
                         for k, v in row.items()})
        return rows


def clean(text):
    """Strip commas/quotes/newlines so the C++ positional CSV splitter never
    sees an embedded delimiter (we don't need text fidelity for routing)."""
    if text is None:
        return ""
    return text.replace(",", " ").replace('"', "").replace("\r", "").replace("\n", " ").strip()


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    if len(args) < 2:
        die("usage: normalize_gtfs.py <raw_feed_dir> <output_dir> [--all|--bus] [--no-collapse]")
    raw, out = args[0], args[1]
    keep_all = "--all" in flags or "--bus" in flags
    collapse = "--no-collapse" not in flags
    os.makedirs(out, exist_ok=True)

    stops_r = read_table(os.path.join(raw, "stops.txt"))
    routes_r = read_table(os.path.join(raw, "routes.txt"))
    trips_r = read_table(os.path.join(raw, "trips.txt"))
    st_r = read_table(os.path.join(raw, "stop_times.txt"))
    if stops_r is None or routes_r is None or trips_r is None or st_r is None:
        die("raw feed must contain stops.txt, routes.txt, trips.txt, stop_times.txt")

    # ── 1. Pick the routes we keep (metro-only by default) ──────────────────
    kept_routes = {}
    for r in routes_r:
        rid = r.get("route_id", "")
        rtype = r.get("route_type", "")
        if not rid:
            continue
        if keep_all or rtype in RAIL_TYPES:
            kept_routes[rid] = r
    if not kept_routes:
        die("no routes survived the mode filter. If this is a bus feed, re-run with --all")

    # ── 2. Trips on those routes ────────────────────────────────────────────
    kept_trips = {}
    for t in trips_r:
        if t.get("route_id", "") in kept_routes and t.get("trip_id", ""):
            kept_trips[t["trip_id"]] = t

    # ── 3. Build platform -> parent-station remap (optional collapse) ────────
    parent_of = {}
    if collapse:
        for s in stops_r:
            sid, par = s.get("stop_id", ""), s.get("parent_station", "")
            if sid and par:
                parent_of[sid] = par

    def canon(sid):
        # follow one level of parent (GTFS parent_station is single-level)
        return parent_of.get(sid, sid)

    # ── 4. Keep stop_times on kept trips; remap stop ids; collect used stops ─
    used_stop_ids = set()
    kept_st = []
    for row in st_r:
        tid = row.get("trip_id", "")
        if tid not in kept_trips:
            continue
        sid = canon(row.get("stop_id", ""))
        if not sid:
            continue
        used_stop_ids.add(sid)
        kept_st.append((tid, row.get("arrival_time", ""), row.get("departure_time", ""),
                        sid, row.get("stop_sequence", "0")))

    # ── 5. Emit stops actually used (must have valid coords) ────────────────
    stops_by_id = {s.get("stop_id", ""): s for s in stops_r}
    out_stops = []
    emitted = set()
    dropped_no_coord = 0
    for sid in used_stop_ids:
        s = stops_by_id.get(sid)
        if s is None:
            continue
        lat, lon = s.get("stop_lat", ""), s.get("stop_lon", "")
        try:
            float(lat); float(lon)
        except (ValueError, TypeError):
            dropped_no_coord += 1
            continue
        out_stops.append((sid, clean(s.get("stop_name", sid)) or sid, lat, lon))
        emitted.add(sid)
    if not out_stops:
        die("no usable stops (missing coordinates?). Check the raw feed's stops.txt")

    # Drop stop_times that reference a stop we couldn't emit (keeps FK drop ~0)
    kept_st = [r for r in kept_st if r[3] in emitted]
    # Trips that still have >=2 stop_times are routable
    trip_counts = {}
    for r in kept_st:
        trip_counts[r[0]] = trip_counts.get(r[0], 0) + 1
    routable_trips = {t for t, c in trip_counts.items() if c >= 2}
    kept_st = [r for r in kept_st if r[0] in routable_trips]

    # ── 6. Write the five files in the engine's exact positional order ──────
    def w(name, header, rows):
        with open(os.path.join(out, name), "w", encoding="utf-8", newline="\n") as f:
            f.write(header + "\n")
            for row in rows:
                f.write(",".join(str(x) for x in row) + "\n")

    w("agency.txt", "agency_id,agency_name,agency_url,agency_timezone",
      [("METRO", "Metro", "https://example.org", "Asia/Kolkata")])

    w("stops.txt", "stop_id,stop_name,stop_lat,stop_lon,stop_desc",
      [(sid, name, lat, lon, "") for (sid, name, lat, lon) in out_stops])

    w("routes.txt", "route_id,agency_id,route_short_name,route_long_name,route_type",
      [(rid, "METRO", clean(r.get("route_short_name", rid)) or rid,
        clean(r.get("route_long_name", rid)) or rid, r.get("route_type", "1"))
       for rid, r in kept_routes.items()])

    w("trips.txt", "route_id,service_id,trip_id",
      [(kept_trips[t]["route_id"], "WEEKDAY", t) for t in routable_trips])

    w("stop_times.txt", "trip_id,arrival_time,departure_time,stop_id,stop_sequence",
      kept_st)

    # trivial calendar so the feed is "complete" (engine treats all trips as active)
    w("calendar.txt",
      "service_id,monday,tuesday,wednesday,thursday,friday,saturday,sunday,start_date,end_date",
      [("WEEKDAY", 1, 1, 1, 1, 1, 1, 1, "20200101", "20301231")])

    print("normalize_gtfs.py — done.")
    print(f"  mode filter    : {'ALL modes' if keep_all else 'rail/metro only'}")
    print(f"  parent collapse: {'on' if collapse else 'off'}")
    print(f"  routes kept    : {len(kept_routes)}")
    print(f"  trips  kept    : {len(routable_trips)}")
    print(f"  stops  kept    : {len(out_stops)}   (dropped for bad coords: {dropped_no_coord})")
    print(f"  stop_times kept: {len(kept_st)}")
    print(f"  -> wrote clean feed to: {out}/")
    print(f"  run: taskset -c 3 ./build_release/routing_engine_benchmark ./{out}")


if __name__ == "__main__":
    main()
