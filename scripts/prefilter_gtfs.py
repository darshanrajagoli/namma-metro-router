#!/usr/bin/env python3
"""
prefilter_gtfs.py — cut a large multimodal GTFS feed down to its rail layer,
streaming, before scripts/normalize_gtfs.py ever sees it.

WHY THIS EXISTS
---------------
normalize_gtfs.py reads each table fully into memory as a list of dicts. That is
the right shape for a metro feed of a few megabytes and the wrong shape for a
national one: the Netherlands and Ile-de-France feeds carry tens of millions of
stop_time rows, and materialising them as Python dicts needs many gigabytes.

This script does the two reductions that make those feeds tractable — rail modes
only, and one service day rather than all of them — and does both without ever
holding a large table in memory:

    routes.txt     -> streamed; keep rail route_ids
    calendar.txt   -> read fully (small); calendar_dates.txt streamed
    trips.txt      -> streamed TWICE: once to count trips per service_id so a
                      service date can be chosen, once to write the survivors
    stop_times.txt -> streamed; keep rows belonging to a surviving trip
    stops, transfers, feed_info, frequencies, pathways, levels -> copied
    shapes.txt     -> dropped; nothing downstream reads it and it is the
                      largest file in a national feed by a wide margin

It deliberately does NOT do any of normalize_gtfs.py's work — no positional
rewriting, no platform collapsing, no transfer generation. Run it first, then
run normalize_gtfs.py on its output. Feeds small enough not to need it produce
identical results either way, which is what tests it.

SCOPE — WHAT COUNTS AS RAIL, AND WHY IT IS DECIDED HERE
------------------------------------------------------
This script is the SINGLE place the study's mode filter lives. fetch_feeds.py
therefore runs normalize_gtfs.py with --all afterwards: the selection has
already happened, and two filters that can disagree are worse than one.

Basic GTFS route_type kept:
    0 tram / light rail, 1 subway / metro, 2 rail, 5 cable tram,
    7 funicular, 12 monorail
Basic types dropped: 3 bus, 4 ferry, 6 aerial lift, 11 TROLLEYBUS.

Note 11. normalize_gtfs.py's own list includes it, describing the set as
"tram/subway/rail/monorail" — but 11 is trolleybus in the specification and 12
is monorail. A trolleybus is a bus, so it is excluded here. This is the one
place the two lists differ, and the reason the mode filter was consolidated
into this file rather than left in two.

Extended route_types are also handled, which the older list did not do at all.
Entur's Norwegian national feed types every service in the extended space, so a
basic-only filter reports it as having no rail service whatsoever and the feed
silently drops out of the study. Kept ranges:
    100-199  railway service          400-499  urban railway / metro
    900-999  tram service             1400-1499 funicular
Dropped: 200-299 coach, 700-799 bus, 800-899 trolleybus, 1000-1099 water,
1100-1199 air, 1200-1299 ferry, 1300-1399 aerial lift, 1500+ taxi and misc.

ONE SERVICE DAY, NOT ALL OF THEM
--------------------------------
A GTFS feed describes a whole timetable period: weekday, Saturday, Sunday and
holiday variants of the same train all sit in trips.txt, distinguished only by
service_id. The C++ parser does not filter on service day — gtfs_parser.hpp says
so — so loading a feed whole puts every variant into the graph at once.

That is not a small distortion. BART's feed carries three trips departing the
same platform at the same second, one per service pattern, which triples the
edge count and drives the measured headway between consecutive departures to
literally zero. Across a study it is worse than a constant factor, because feeds
differ in how many patterns they publish: one agency ships three, another ships
a separate service_id for every date in the timetable period.

So trips are filtered to a single service date here, resolving calendar.txt and
calendar_dates.txt properly (weekday mask and date range, plus type 1 additions
and type 2 removals). The default picks the WEEKDAY with the most active trips
inside the feed's own service period, which is a normal full-service day rather
than a holiday or a Sunday. The chosen date is printed and recorded.

USAGE
-----
    python3 scripts/prefilter_gtfs.py <raw_feed_dir> <output_dir> [--all]
                                      [--service-date auto|YYYYMMDD|all]

    --all             keep every mode (the script then only reduces by service
                      day, which is still useful for uniformity in a pipeline)
    --service-date    auto (default), an explicit YYYYMMDD, or `all` to keep
                      every service pattern and reproduce the old behaviour
"""

import csv
import datetime
import os
import shutil
import sys

BASIC_RAIL_TYPES = {"0", "1", "2", "5", "7", "12"}
EXTENDED_RAIL_RANGES = ((100, 199), (400, 499), (900, 999), (1400, 1499))


def is_rail(route_type):
    """True when this GTFS route_type is a fixed-guideway mode. See the header."""
    rt = (route_type or "").strip()
    if rt in BASIC_RAIL_TYPES:
        return True
    try:
        value = int(rt)
    except ValueError:
        return False
    return any(lo <= value <= hi for lo, hi in EXTENDED_RAIL_RANGES)


# Tables copied through untouched. stop_times.txt and trips.txt are handled
# specially; everything else a feed may carry is either small or irrelevant.
#
# shapes.txt is deliberately ABSENT. Nothing downstream reads it — the C++
# parser never opens it and the normaliser never emits it — and it is by far the
# largest file in a national feed. Entur's is 2.8 GB uncompressed, against
# 890 MB for the stop_times it actually needs.
PASSTHROUGH = [
    "agency.txt",
    "stops.txt",
    "calendar.txt",
    "calendar_dates.txt",
    "transfers.txt",
    "feed_info.txt",
    "frequencies.txt",
    "pathways.txt",
    "levels.txt",
]


def die(msg):
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def open_text(path):
    """utf-8-sig strips the BOM some agencies emit; newline='' lets csv handle
    embedded newlines inside quoted fields itself."""
    return open(path, "r", encoding="utf-8-sig", newline="", errors="replace")


def read_header(reader):
    """Return the header row with every field name trimmed.

    Not optional politeness: Metra publishes routes.txt with a space after each
    comma in the HEADER line, so the columns are named ' route_type' and ' agency_id'.
    A dict lookup for 'route_type' then misses, every route looks like it has no
    type, the rail filter keeps nothing, and the feed is reported as having no
    rail service. normalize_gtfs.py already trims header names for the same
    reason; this is the streaming equivalent.
    """
    try:
        return [h.strip() for h in next(reader)]
    except StopIteration:
        return None


WEEKDAY_COLUMNS = ["monday", "tuesday", "wednesday", "thursday", "friday",
                   "saturday", "sunday"]


def read_rows(path):
    """Read a whole (small) GTFS table as dicts with trimmed keys and values."""
    if not os.path.exists(path):
        return []
    with open_text(path) as f:
        reader = csv.reader(f)
        header = read_header(reader)
        if header is None:
            return []
        return [{header[i]: (row[i].strip() if i < len(row) else "")
                 for i in range(len(header))} for row in reader]


def parse_calendar(raw):
    """Return (rules, added, removed) ready for repeated date queries.

    `rules` is a list of (service_id, weekday_mask, start, end) with the dates as
    plain YYYYMMDD integers — the form they are compared in, so the conversion
    happens once rather than once per candidate date.
    """
    rules = []
    for row in read_rows(os.path.join(raw, "calendar.txt")):
        sid = row.get("service_id", "")
        if not sid:
            continue
        try:
            start = int(row.get("start_date", "") or 0)
            end = int(row.get("end_date", "") or 0)
        except ValueError:
            continue
        mask = tuple(row.get(day, "0") == "1" for day in WEEKDAY_COLUMNS)
        rules.append((sid, mask, start, end))

    # calendar_dates.txt is streamed rather than read whole: Entur's is 33 MB and
    # around 700,000 rows, and one dict per row is half a gigabyte of Python
    # objects to extract three columns from.
    added, removed = {}, {}
    path = os.path.join(raw, "calendar_dates.txt")
    if os.path.exists(path):
        with open_text(path) as f:
            reader = csv.reader(f)
            header = read_header(reader)
            if header and all(c in header for c in ("service_id", "date", "exception_type")):
                c_sid = header.index("service_id")
                c_date = header.index("date")
                c_kind = header.index("exception_type")
                width = max(c_sid, c_date, c_kind)
                for row in reader:
                    if len(row) <= width:
                        continue
                    sid, day, kind = row[c_sid].strip(), row[c_date].strip(), row[c_kind].strip()
                    if not sid or not day.isdigit():
                        continue
                    (added if kind == "1" else removed).setdefault(int(day), set()).add(sid)
    return rules, added, removed


def active_services_on(date, rules, added, removed):
    """Service ids running on `date` (a datetime.date), per the GTFS rules."""
    stamp = int(date.strftime("%Y%m%d"))
    weekday = date.weekday()
    active = {sid for sid, mask, start, end in rules
              if mask[weekday] and start <= stamp <= end}
    active |= added.get(stamp, set())
    active -= removed.get(stamp, set())
    return active


# Bound on how many candidate weekdays are scanned. Feeds declare periods
# ranging from a fortnight to a decade, and the scan is linear in both the
# number of candidates and the number of calendar rules.
MAX_CANDIDATE_WEEKDAYS = 2000


def choose_service_date(raw, trips_per_service):
    """Pick the weekday with the most active trips inside the feed's own period.

    Returns (date_string_or_None, active_service_ids). None means the feed
    carries no usable calendar at all, in which case the caller keeps every trip
    and says so rather than silently emptying the feed.

    The candidate window is the feed's whole declared period, NOT a fixed slice
    of it. An earlier version clamped to the first 120 days, which looked
    reasonable and quietly broke on any feed carrying one stale calendar row: on
    Entur's Norwegian feed the window landed years before the live timetable, no
    weekday had a single active trip, and the feed fell back to keeping every
    service pattern — the exact distortion this whole step exists to remove. A
    long period is scanned from its END, because the most recent weekday is the
    one the feed is actually about.
    """
    rules, added, removed = parse_calendar(raw)
    if not rules and not added:
        return None, set()

    stamps = [s for _sid, _mask, start, end in rules for s in (start, end) if s > 0]
    stamps.extend(added.keys())
    stamps.extend(removed.keys())
    if not stamps:
        return None, set()

    def to_date(stamp):
        try:
            return datetime.datetime.strptime(str(stamp), "%Y%m%d").date()
        except ValueError:
            return None

    dates = [d for d in (to_date(s) for s in stamps) if d is not None]
    if not dates:
        return None, set()
    first, last = min(dates), max(dates)

    candidates = []
    day = last
    while day >= first and len(candidates) < MAX_CANDIDATE_WEEKDAYS:
        if day.weekday() < 5:  # Monday..Friday: a normal full-service day
            candidates.append(day)
        day -= datetime.timedelta(days=1)
    candidates.reverse()  # earliest first, so ties resolve to the earlier date

    best = None
    for candidate in candidates:
        active = active_services_on(candidate, rules, added, removed)
        count = sum(trips_per_service.get(s, 0) for s in active)
        # Strict >: ties keep the earliest date, so the choice is a
        # deterministic function of the feed.
        if count > 0 and (best is None or count > best[1]):
            best = (candidate, count, active)

    if best is None:
        return None, set()
    return best[0].strftime("%Y%m%d"), best[2]


def main():
    argv = sys.argv[1:]
    service_date = "auto"
    if "--service-date" in argv:
        k = argv.index("--service-date")
        if k + 1 >= len(argv):
            die("--service-date needs a value: auto, YYYYMMDD, or all")
        service_date = argv[k + 1]
        del argv[k:k + 2]

    args = [a for a in argv if not a.startswith("--")]
    flags = {a for a in argv if a.startswith("--")}
    if len(args) < 2:
        die("usage: prefilter_gtfs.py <raw_feed_dir> <output_dir> [--all] "
            "[--service-date auto|YYYYMMDD|all]")
    raw, out = args[0], args[1]
    keep_all = "--all" in flags
    os.makedirs(out, exist_ok=True)

    for required in ("routes.txt", "trips.txt", "stop_times.txt"):
        if not os.path.exists(os.path.join(raw, required)):
            die(f"{raw} is missing {required}")

    # ── routes ───────────────────────────────────────────────────────────────
    # csv.reader with an explicitly trimmed header everywhere, rather than
    # DictReader: DictReader keys on the raw header text, and real feeds pad it.
    kept_route_ids = set()
    with open_text(os.path.join(raw, "routes.txt")) as f:
        reader = csv.reader(f)
        header = read_header(reader)
        if header is None:
            die("routes.txt has no header")
        if "route_id" not in header or "route_type" not in header:
            die(f"routes.txt needs route_id and route_type columns; got {header}")
        c_id, c_type = header.index("route_id"), header.index("route_type")
        with open(os.path.join(out, "routes.txt"), "w", encoding="utf-8", newline="") as g:
            w = csv.writer(g)
            w.writerow(header)
            for row in reader:
                if len(row) <= max(c_id, c_type):
                    continue
                rid, rtype = row[c_id].strip(), row[c_type].strip()
                if not rid:
                    continue
                if keep_all or is_rail(rtype):
                    kept_route_ids.add(rid)
                    w.writerow(row)
    n_routes = len(kept_route_ids)
    if n_routes == 0:
        die("no rail routes in this feed (re-run with --all if it is a bus feed)")

    # ── trips, in two streaming passes ───────────────────────────────────────
    # Pass A counts trips per service_id so a service date can be chosen; pass B
    # writes the survivors. Two passes over trips.txt rather than one pass into
    # memory: Entur's is 66 MB, and materialising it as Python rows costs the
    # better part of a gigabyte for a file that is read twice in seconds.
    trips_path = os.path.join(raw, "trips.txt")
    with open_text(trips_path) as f:
        header = read_header(csv.reader(f))
    if header is None:
        die("trips.txt has no header")
    for required in ("route_id", "trip_id"):
        if required not in header:
            die(f"trips.txt needs route_id and trip_id columns; got {header}")
    c_route, c_trip = header.index("route_id"), header.index("trip_id")
    c_service = header.index("service_id") if "service_id" in header else -1

    trips_per_service = {}
    if c_service >= 0 and service_date != "all":
        with open_text(trips_path) as f:
            reader = csv.reader(f)
            read_header(reader)
            for row in reader:
                if len(row) <= max(c_route, c_service):
                    continue
                if row[c_route].strip() in kept_route_ids:
                    sid = row[c_service].strip()
                    trips_per_service[sid] = trips_per_service.get(sid, 0) + 1

    active_services, chosen_date, service_note = None, "", ""
    if service_date == "all":
        service_note = "every service pattern kept (--service-date all)"
    elif c_service < 0:
        service_note = "trips.txt has no service_id column; every trip kept"
    elif service_date == "auto":
        chosen_date, active_services = choose_service_date(raw, trips_per_service)
        if chosen_date is None:
            active_services = None
            service_note = "no usable calendar.txt or calendar_dates.txt; every trip kept"
        else:
            service_note = f"auto-selected weekday {chosen_date}"
    else:
        if not (len(service_date) == 8 and service_date.isdigit()):
            die("--service-date must be auto, all, or YYYYMMDD")
        try:
            day = datetime.datetime.strptime(service_date, "%Y%m%d").date()
        except ValueError:
            die(f"--service-date {service_date} is not a real date")
        rules, added, removed = parse_calendar(raw)
        active_services = active_services_on(day, rules, added, removed)
        chosen_date = service_date
        service_note = f"service date {service_date} as requested"
        if not any(trips_per_service.get(s, 0) for s in active_services):
            die(f"no trips are active on {service_date}; pick another date or use "
                f"--service-date auto")

    kept_trip_ids = set()
    with open_text(trips_path) as f:
        reader = csv.reader(f)
        read_header(reader)
        with open(os.path.join(out, "trips.txt"), "w", encoding="utf-8", newline="") as g:
            w = csv.writer(g)
            w.writerow(header)
            for row in reader:
                if len(row) <= max(c_route, c_trip):
                    continue
                if row[c_route].strip() not in kept_route_ids:
                    continue
                if active_services is not None and c_service >= 0:
                    if len(row) <= c_service or row[c_service].strip() not in active_services:
                        continue
                tid = row[c_trip].strip()
                if tid:
                    kept_trip_ids.add(tid)
                    w.writerow(row)
    n_trips = len(kept_trip_ids)
    if n_trips == 0:
        die("no trips survived the route and service-day filters")

    # ── stop_times: the streamed pass ────────────────────────────────────────
    # One index lookup per row and no per-row dict: building a dict per row is
    # the allocation that makes the naive approach fall over on a
    # forty-million-row national feed.
    n_st_in = n_st_out = 0
    with open_text(os.path.join(raw, "stop_times.txt")) as f:
        reader = csv.reader(f)
        header = read_header(reader)
        if header is None:
            die("stop_times.txt is empty")
        if "trip_id" not in header:
            die("stop_times.txt has no trip_id column")
        trip_col = header.index("trip_id")
        with open(os.path.join(out, "stop_times.txt"), "w", encoding="utf-8", newline="") as g:
            w = csv.writer(g)
            w.writerow(header)
            for row in reader:
                n_st_in += 1
                if len(row) <= trip_col:
                    continue
                if row[trip_col].strip() in kept_trip_ids:
                    w.writerow(row)
                    n_st_out += 1

    # ── everything else ──────────────────────────────────────────────────────
    copied = []
    for name in PASSTHROUGH:
        src = os.path.join(raw, name)
        if os.path.exists(src):
            shutil.copyfile(src, os.path.join(out, name))
            copied.append(name)

    print("prefilter_gtfs.py — done.")
    print(f"  mode filter : {'ALL modes' if keep_all else 'rail only (basic + extended, see header)'}")
    print(f"  service day : {service_note}")
    print(f"  routes kept : {n_routes}")
    print(f"  trips  kept : {n_trips}")
    print(f"  stop_times  : {n_st_out} of {n_st_in} kept "
          f"({100.0 * n_st_out / n_st_in if n_st_in else 0.0:.1f}%)")
    print(f"  copied      : {', '.join(copied) if copied else '(none)'}")
    print(f"  -> {out}/")


if __name__ == "__main__":
    main()
