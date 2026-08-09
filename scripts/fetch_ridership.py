#!/usr/bin/env python3
"""
fetch_ridership.py — download the measured BMRCL station-hourly ridership data.

    python3 scripts/fetch_ridership.py --out data-ridership

WHAT THIS DOWNLOADS AND WHY IT IS THE RIGHT DATA
------------------------------------------------
Bangalore Metro Rail Corporation publishes daily system-wide ridership but not a
station-level breakdown. The breakdown used here was obtained from BMRCL under
the Right to Information Act and republished as open data: entries per station
per hour, for every station on the network, over a two-month period.

That shape matters. The crowd model this replaces is a Gaussian in time of day
and nothing else, identical at every station in the city, so the second Pareto
objective had nothing to distinguish two routes with — which is exactly what the
engine's own diagnostic measured when it found a single-label frontier at 96-100%
of nodes. A field that varies over STATION as well as hour is the minimum needed
for a route-choice trade-off to be representable at all.

    dataset : github.com/Vonter/bmrcl-ridership-hourly
    upstream: data.opencity.in/dataset/bmrcl-station-wise-ridership-data
    source  : BMRCL, via RTI

WHAT IT IS NOT
--------------
Station entries are not train occupancy. See the caveat block at the top of
include/crowd_model.hpp, which states the modelling assumption plainly rather
than letting the number pass for something it is not.

PINNING
-------
The upstream repository refreshes as new RTI responses arrive, so the checksum
below WILL move. A mismatch is reported loudly and the new checksum recorded —
it is not an error, but it does mean results either side of it are not directly
comparable. Same discipline as scripts/fetch_feeds.py.
"""

import argparse
import hashlib
import json
import os
import sys
import time
import urllib.error
import urllib.request
import zipfile
from datetime import datetime, timezone

URL = "https://raw.githubusercontent.com/Vonter/bmrcl-ridership-hourly/main/data/station-hourly.csv.zip"
SOURCE_PAGE = "https://github.com/Vonter/bmrcl-ridership-hourly"

# Checksum of the archive this project's published numbers were produced from.
# Recorded, verified, and reported when it moves — not enforced, because the
# upstream is a living dataset and failing the build on a data refresh would be
# a worse outcome than saying so.
PINNED_SHA256 = "a0469f3365c53ca25d9aed5396775fc1c6018cfc0b93e245e8bc3e1c9524bd81"

USER_AGENT = "namma-metro-router/1.0 (+https://github.com/darshanrajagoli/namma-metro-router)"
RETRIES = 3


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def download(url, dest):
    last = None
    for attempt in range(1, RETRIES + 1):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=180) as resp, open(dest + ".part", "wb") as f:
                while True:
                    chunk = resp.read(1 << 20)
                    if not chunk:
                        break
                    f.write(chunk)
            os.replace(dest + ".part", dest)
            return
        except (urllib.error.URLError, urllib.error.HTTPError, OSError) as e:
            last = e
            if attempt < RETRIES:
                time.sleep(2 * attempt)
    raise RuntimeError(f"download failed after {RETRIES} attempts: {last}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="data-ridership", help="output directory")
    ap.add_argument("--force", action="store_true", help="re-download even if present")
    args = ap.parse_args()

    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)
    zip_path = os.path.join(out, "station-hourly.csv.zip")

    if args.force or not os.path.exists(zip_path):
        print(f"downloading {URL}")
        download(URL, zip_path)
    else:
        print(f"using existing {zip_path} (pass --force to re-download)")

    digest = sha256_file(zip_path)
    if digest == PINNED_SHA256:
        print(f"sha256 {digest}  (matches the pin)")
    else:
        print(f"sha256 {digest}")
        print(f"  [PIN MOVED] expected {PINNED_SHA256}")
        print("  The upstream dataset has been refreshed. This is normal; it does mean")
        print("  numbers produced before and after are not directly comparable.")

    with zipfile.ZipFile(zip_path) as z:
        names = [n for n in z.namelist() if n.lower().endswith(".csv")]
        if not names:
            print("ERROR: archive contains no .csv", file=sys.stderr)
            return 1
        root = os.path.realpath(out)
        for n in names:
            target = os.path.realpath(os.path.join(out, n))
            if not (target == root or target.startswith(root + os.sep)):
                print(f"ERROR: archive member escapes the output directory: {n}", file=sys.stderr)
                return 1
        z.extractall(out, members=names)
        csv_name = names[0]

    csv_path = os.path.join(out, csv_name)
    lock = {
        "url": URL,
        "source_page": SOURCE_PAGE,
        "provenance": "BMRCL station-wise hourly ridership, obtained under RTI and republished as open data",
        "zip_sha256": digest,
        "csv_sha256": sha256_file(csv_path),
        "csv": csv_name,
        "bytes": os.path.getsize(csv_path),
        "fetched_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "matches_pin": digest == PINNED_SHA256,
    }
    with open(os.path.join(out, "ridership.lock.json"), "w", encoding="utf-8") as f:
        json.dump(lock, f, indent=2, sort_keys=True)
        f.write("\n")

    print(f"\nwrote {csv_path}")
    print(f"wrote {os.path.join(out, 'ridership.lock.json')}")
    print("\nnext:")
    print(f"  ./build/routing_engine_crowd_study ./feeds/norm/namma-metro \\")
    print(f"      --ridership {csv_path} --aliases scripts/station-aliases-bmrcl.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
