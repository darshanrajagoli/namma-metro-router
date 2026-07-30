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
    python3 scripts/normalize_gtfs.py  raw_bart        data_bart --transfers

<raw_feed_dir> is a folder containing the unzipped .txt files of a real GTFS feed.
<output_dir>   is where the clean files are written (point the engine at this).

--transfers
    Keep platforms as separate nodes (implies --no-collapse) and emit transfers.txt
    so that changing lines costs real time and can be counted as a second objective.

    Why this needs generating rather than copying: a feed's transfers.txt is
    sparse. BART's has 34 rows covering 11 stations, but 36 further stations have
    more than one platform and no row at all. Copying it verbatim would leave
    those stations with no way to change platform, silently disconnecting the
    graph. So every ordered same-parent platform pair gets an edge, using the
    feed's min_transfer_time where it exists and --default-transfer elsewhere.

    Transfer types are resolved here, not in the engine:
      type 0 / blank -> recommended  : use --default-transfer
      type 1         -> timed        : use min_transfer_time
      type 2         -> min required : use min_transfer_time
      type 3         -> NOT possible : pair is dropped entirely
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
    argv = sys.argv[1:]
    # --default-transfer takes a value; pull it out before positional parsing.
    default_transfer = 120
    if "--default-transfer" in argv:
        k = argv.index("--default-transfer")
        if k + 1 >= len(argv):
            die("--default-transfer needs a value in seconds")
        try:
            default_transfer = int(argv[k + 1])
        except ValueError:
            die("--default-transfer must be an integer number of seconds")
        del argv[k:k + 2]

    args = [a for a in argv if not a.startswith("--")]
    flags = {a for a in argv if a.startswith("--")}
    if len(args) < 2:
        die("usage: normalize_gtfs.py <raw_feed_dir> <output_dir> "
            "[--all|--bus] [--no-collapse] [--transfers] [--default-transfer SECONDS]")
    raw, out = args[0], args[1]
    keep_all = "--all" in flags or "--bus" in flags
    want_transfers = "--transfers" in flags
    # Transfers are walks BETWEEN platforms. Collapsing platforms onto their
    # parent station merges those endpoints into one node, which would turn every
    # transfer into a self-loop. The two options are mutually exclusive.
    collapse = ("--no-collapse" not in flags) and not want_transfers
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
    # sorted(), not raw set iteration: Python randomises string hashing per
    # process (PYTHONHASHSEED), so iterating the set directly emits stops.txt in a
    # different order on every run. The engine assigns node indices by row order,
    # so that would renumber the entire graph between two runs of this script --
    # changing which (source, departure) pairs the benchmark samples and making
    # every published latency and frontier statistic irreproducible.
    for sid in sorted(used_stop_ids):
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
      [(kept_trips[t]["route_id"], "WEEKDAY", t) for t in sorted(routable_trips)])

    w("stop_times.txt", "trip_id,arrival_time,departure_time,stop_id,stop_sequence",
      kept_st)

    # trivial calendar so the feed is "complete" (engine treats all trips as active)
    w("calendar.txt",
      "service_id,monday,tuesday,wednesday,thursday,friday,saturday,sunday,start_date,end_date",
      [("WEEKDAY", 1, 1, 1, 1, 1, 1, 1, "20200101", "20301231")])

    # ── transfers.txt (only with --transfers) ────────────────────────────────
    n_transfers = n_from_feed = n_defaulted = n_forbidden = 0
    if want_transfers:
        # Feed-supplied times and prohibitions, keyed by ordered platform pair.
        feed_time, forbidden = {}, set()
        raw_tr = read_table(os.path.join(raw, "transfers.txt")) or []
        for r in raw_tr:
            a, b = r.get("from_stop_id", ""), r.get("to_stop_id", "")
            if not a or not b:
                continue
            ttype = (r.get("transfer_type", "") or "").strip()
            if ttype == "3":                       # transfer explicitly not possible
                forbidden.add((a, b))
                continue
            mt = (r.get("min_transfer_time", "") or "").strip()
            if ttype in ("1", "2") and mt.isdigit():
                feed_time[(a, b)] = int(mt)

        # Group the stops we actually emitted by their parent station. Only stops
        # that survived to out_stops can be transfer endpoints.
        by_parent = {}
        for (sid, _name, _lat, _lon) in out_stops:
            s = stops_by_id.get(sid)
            parent = (s.get("parent_station") or "").strip() if s else ""
            by_parent.setdefault(parent or sid, []).append(sid)

        rows = []
        for _parent, plats in by_parent.items():
            if len(plats) < 2:
                continue
            for a in plats:
                for b in plats:
                    if a == b:
                        continue
                    if (a, b) in forbidden:
                        n_forbidden += 1
                        continue
                    if (a, b) in feed_time:
                        secs = feed_time[(a, b)]
                        n_from_feed += 1
                    else:
                        secs = default_transfer
                        n_defaulted += 1
                    rows.append((a, b, secs))
        n_transfers = len(rows)
        w("transfers.txt", "from_stop_id,to_stop_id,min_transfer_time", rows)

    print("normalize_gtfs.py — done.")
    print(f"  mode filter    : {'ALL modes' if keep_all else 'rail/metro only'}")
    print(f"  parent collapse: {'on' if collapse else 'off'}")
    if want_transfers:
        print(f"  transfers      : {n_transfers} emitted "
              f"({n_from_feed} from feed, {n_defaulted} defaulted to {default_transfer}s, "
              f"{n_forbidden} forbidden/skipped)")
    print(f"  routes kept    : {len(kept_routes)}")
    print(f"  trips  kept    : {len(routable_trips)}")
    print(f"  stops  kept    : {len(out_stops)}   (dropped for bad coords: {dropped_no_coord})")
    print(f"  stop_times kept: {len(kept_st)}")
    print(f"  -> wrote clean feed to: {out}/")
    print(f"  run: taskset -c 3 ./build_release/routing_engine_benchmark ./{out}")


if __name__ == "__main__":
    main()
