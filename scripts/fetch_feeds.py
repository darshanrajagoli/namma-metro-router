#!/usr/bin/env python3
"""
fetch_feeds.py — acquire, pin and normalise every feed in scripts/feeds.json.

WHAT IT PRODUCES
----------------
    <workdir>/zip/<slug>.zip      the byte-for-byte download
    <workdir>/raw/<slug>/         unzipped
    <workdir>/rail/<slug>/        rail modes only (scripts/prefilter_gtfs.py)
    <workdir>/norm/<slug>/        engine-ready feed (scripts/normalize_gtfs.py --transfers)
    <workdir>/feeds.lock.json     slug -> {url, sha256, bytes, fetched_at, ...}

WHY THE LOCK FILE MATTERS
-------------------------
Transit feeds are republished continuously, without versions, at the same URL.
This project has already been bitten by that once: the BART figures in the
README moved because BART added roughly 40% more service between two runs. A
study whose inputs mutate underneath it is not reproducible, so every download
is hashed and the hash is written to feeds.lock.json. `--verify` re-checks the
local zips against the lock and fails on any mismatch; `--refresh` is the only
way to move a pin, and it says loudly which pins moved.

The lock file records what the study was run against. It does not, and cannot,
let someone else re-download the identical bytes months later — agencies do not
keep old feeds. That is a property of the domain, not of this script. What the
lock does give you is the ability to (a) prove your own results came from the
inputs you say they did, and (b) detect the day the upstream feed changed.
See docs/reproducibility.md for how the pinned-snapshot archive closes the gap.

USAGE
-----
    python3 scripts/fetch_feeds.py --workdir feeds
    python3 scripts/fetch_feeds.py --workdir feeds --only bart,mbta-boston
    python3 scripts/fetch_feeds.py --workdir feeds --verify
    python3 scripts/fetch_feeds.py --workdir feeds --refresh
    python3 scripts/fetch_feeds.py --list

Standard library only, on purpose: the reproducibility artifact should not need
a package index to be reachable in ten years' time.
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
import zipfile
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
MANIFEST = os.path.join(HERE, "feeds.json")

# Identifies the project and links to it, so an agency seeing this in their logs
# can tell who is asking and why. Deliberately does NOT claim to be a browser or
# any other client. It also deliberately omits the word "python": MARTA's origin
# rejects urllib's default agent with HTTP 403 while serving this one, and the
# fix for that is an accurate identifier, not a disguise.
USER_AGENT = "namma-metro-router/1.0 (+https://github.com/darshanrajagoli/namma-metro-router)"
# Generous: several agencies serve 100 MB+ feeds from slow origins, and a
# timeout that fires mid-download looks exactly like an unavailable feed.
TIMEOUT_S = 300
RETRIES = 3


def log(msg):
    print(msg, flush=True)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_manifest():
    with open(MANIFEST, "r", encoding="utf-8") as f:
        return json.load(f)


def load_lock(path):
    if not os.path.exists(path):
        return {"schema": 1, "feeds": {}}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_lock(path, lock):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(lock, f, indent=2, sort_keys=True)
        f.write("\n")
    os.replace(tmp, path)


def download(url, dest):
    """Download with retries. Returns (bytes_written, http_headers)."""
    last_err = None
    for attempt in range(1, RETRIES + 1):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=TIMEOUT_S) as resp:
                headers = dict(resp.headers.items())
                tmp = dest + ".part"
                total = 0
                with open(tmp, "wb") as f:
                    while True:
                        chunk = resp.read(1 << 20)
                        if not chunk:
                            break
                        f.write(chunk)
                        total += len(chunk)
                os.replace(tmp, dest)
                return total, headers
        except (urllib.error.URLError, urllib.error.HTTPError, OSError) as e:
            last_err = e
            if attempt < RETRIES:
                time.sleep(2 * attempt)
    raise RuntimeError(f"download failed after {RETRIES} attempts: {last_err}")


def run(cmd, cwd=None):
    """Run a subprocess, surfacing its output on failure and nowhere else."""
    proc = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"{' '.join(cmd)} exited {proc.returncode}\n"
            f"--- stdout ---\n{proc.stdout}\n--- stderr ---\n{proc.stderr}"
        )
    return proc.stdout


def _extract_flat(zip_path, dest):
    """Extract one archive into `dest`, refusing member paths that escape it.

    The traversal check is not paranoia for its own sake: this script downloads
    archives from three dozen third-party hosts and unpacks them, so a malicious
    or merely broken member name must not be able to write outside the workdir.
    """
    os.makedirs(dest, exist_ok=True)
    with zipfile.ZipFile(zip_path) as z:
        members = [m for m in z.namelist() if not m.endswith("/")]
        if not members:
            raise RuntimeError("archive contains no files")
        root = os.path.realpath(dest)
        for m in members:
            target = os.path.realpath(os.path.join(dest, m))
            if not (target == root or target.startswith(root + os.sep)):
                raise RuntimeError(f"archive member escapes the output directory: {m}")
        # shapes.txt is skipped: nothing downstream reads it and it dominates the
        # size of a national feed (2.8 GB of Entur's 3.8 GB). Extracting it costs
        # minutes of disk I/O per feed to produce a file that is deleted unread.
        wanted = [m for m in members if os.path.basename(m).lower() != "shapes.txt"]
        z.extractall(dest, members=wanted)

    # Some feeds are wrapped in one directory; move the .txt files up so every
    # downstream step can assume a flat layout.
    if not os.path.exists(os.path.join(dest, "stop_times.txt")):
        for entry in sorted(os.listdir(dest)):
            inner = os.path.join(dest, entry)
            if os.path.isdir(inner) and os.path.exists(os.path.join(inner, "stop_times.txt")):
                for name in os.listdir(inner):
                    shutil.move(os.path.join(inner, name), os.path.join(dest, name))
                break


def safe_extract(zip_path, dest, inner_name=""):
    """Extract a feed, following one level of archive-inside-archive.

    Two agencies in the manifest do this and they do it differently. SEPTA
    publishes gtfs_public.zip containing google_bus.zip and google_rail.zip;
    PTV publishes one numbered directory per mode, each holding its own
    google_transit.zip. Either way there is no feed at the top level, and
    merging the inner archives is not an option — they would overwrite each
    other's stops.txt. Exactly one is chosen:

      1. the manifest's `inner` field, when the entry names one, matched against
         the archive-relative path so "2/google_transit.zip" works — explicit
         beats clever, and a per-feed decision belongs in the manifest;
      2. otherwise the only member, if there is only one;
      3. otherwise the member whose name mentions rail, since this study's scope
         is the rail layer;
      4. otherwise: fail, naming the candidates, rather than pick arbitrarily.
    """
    _extract_flat(zip_path, dest)
    if os.path.exists(os.path.join(dest, "stop_times.txt")):
        return

    nested = []
    for root, _dirs, files in os.walk(dest):
        for name in files:
            if name.lower().endswith(".zip"):
                nested.append(os.path.relpath(os.path.join(root, name), dest).replace(os.sep, "/"))
    nested.sort()
    if not nested:
        raise RuntimeError("archive has no stop_times.txt and no nested archive")

    chosen = None
    if inner_name:
        if inner_name not in nested:
            raise RuntimeError(f"manifest names inner archive '{inner_name}', "
                               f"which is not present; found {nested}")
        chosen = inner_name
    elif len(nested) == 1:
        chosen = nested[0]
    else:
        rail = [n for n in nested if "rail" in n.lower()]
        if len(rail) == 1:
            chosen = rail[0]
    if chosen is None:
        raise RuntimeError(f"several nested archives and no way to choose: {nested}. "
                           f"Add an \"inner\" field to this feed's manifest entry.")

    log(f"      nested archive: using {chosen}")
    inner_zip = os.path.join(dest, chosen)
    staging = dest + ".inner"
    if os.path.exists(staging):
        shutil.rmtree(staging)
    _extract_flat(inner_zip, staging)
    for name in os.listdir(dest):
        p = os.path.join(dest, name)
        if os.path.isfile(p):
            os.remove(p)
        else:
            shutil.rmtree(p)
    for name in os.listdir(staging):
        shutil.move(os.path.join(staging, name), os.path.join(dest, name))
    shutil.rmtree(staging, ignore_errors=True)
    if not os.path.exists(os.path.join(dest, "stop_times.txt")):
        raise RuntimeError(f"nested archive {chosen} contains no stop_times.txt")


def build_namma(entry, workdir, slug):
    """The Bengaluru entry is a CSV of station topology, not a GTFS zip; the
    repository's own builder turns it into a feed with a modelled timetable."""
    raw_dir = os.path.join(workdir, "raw", slug)
    os.makedirs(raw_dir, exist_ok=True)
    csv_path = os.path.join(raw_dir, "bengaluru_metro_network.csv")
    written, _ = download(entry["url"], csv_path)
    digest = sha256_file(csv_path)
    norm_dir = os.path.join(workdir, "norm", slug)
    if os.path.exists(norm_dir):
        shutil.rmtree(norm_dir)
    run([sys.executable, os.path.join(HERE, "build_namma_metro_gtfs.py"), csv_path, norm_dir])
    return written, digest


def process_feed(entry, workdir, lock, refresh, verify_only):
    slug = entry["slug"]
    zip_dir = os.path.join(workdir, "zip")
    os.makedirs(zip_dir, exist_ok=True)

    record = lock["feeds"].get(slug, {})

    # ── The Bengaluru special case ────────────────────────────────────────────
    if entry.get("builder"):
        if verify_only:
            return {"slug": slug, "status": "skipped", "detail": "builder feed; nothing pinned to verify"}
        written, digest = build_namma(entry, workdir, slug)
        lock["feeds"][slug] = {
            "url": entry["url"],
            "sha256": digest,
            "bytes": written,
            "fetched_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "kind": "builder-input-csv",
            "builder": entry["builder"],
        }
        return {"slug": slug, "status": "ok", "detail": f"built from CSV ({written} B)"}

    zip_path = os.path.join(zip_dir, slug + ".zip")

    # ── Verify mode ───────────────────────────────────────────────────────────
    if verify_only:
        if not record:
            return {"slug": slug, "status": "unpinned", "detail": "no lock entry"}
        if not os.path.exists(zip_path):
            return {"slug": slug, "status": "missing", "detail": "pinned but not downloaded"}
        expected = record.get("sha256") or ""
        actual = sha256_file(zip_path)
        if actual != expected:
            return {"slug": slug, "status": "MISMATCH",
                    "detail": f"lock {expected[:12] or '(absent)'} vs disk {actual[:12]}"}
        return {"slug": slug, "status": "ok", "detail": "checksum matches lock"}

    # ── Download (or reuse a matching pinned copy) ────────────────────────────
    reused = False
    if os.path.exists(zip_path) and record.get("sha256") and not refresh:
        if sha256_file(zip_path) == record["sha256"]:
            reused = True
    if not reused:
        written, headers = download(entry["url"], zip_path)
        digest = sha256_file(zip_path)
        previous = record.get("sha256")
        if previous and previous != digest:
            log(f"  [PIN MOVED] {slug}: upstream changed "
                f"({previous[:12]} -> {digest[:12]}). Results before and after are "
                f"NOT comparable.")
        lock["feeds"][slug] = {
            "url": entry["url"],
            "sha256": digest,
            "bytes": written,
            "fetched_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "http_last_modified": headers.get("Last-Modified", ""),
            "http_etag": headers.get("ETag", ""),
            "kind": "gtfs-zip",
        }

    # ── Unzip -> rail prefilter -> normalise ─────────────────────────────────
    raw_dir = os.path.join(workdir, "raw", slug)
    rail_dir = os.path.join(workdir, "rail", slug)
    norm_dir = os.path.join(workdir, "norm", slug)
    for d in (raw_dir, rail_dir, norm_dir):
        if os.path.exists(d):
            shutil.rmtree(d)

    safe_extract(zip_path, raw_dir, entry.get("inner", ""))
    pre = run([sys.executable, os.path.join(HERE, "prefilter_gtfs.py"), raw_dir, rail_dir,
               "--service-date", entry.get("service_date", "auto")])
    service_day = next((ln.split(":", 1)[1].strip() for ln in pre.splitlines()
                        if ln.strip().startswith("service day")), "")
    lock["feeds"][slug]["service_day"] = service_day
    # --all, deliberately: prefilter_gtfs.py has already applied the study's mode
    # filter, and it is the only place that decision lives. Letting the
    # normaliser filter again would mean two lists that can disagree, and its
    # list does not understand the extended route_type space several European
    # feeds use.
    out = run([sys.executable, os.path.join(HERE, "normalize_gtfs.py"),
               rail_dir, norm_dir, "--all", "--transfers"])

    # The normaliser prints its own counts; pull the two that decide whether a
    # feed is usable at all.
    stops = trips = transfers = "?"
    for line in out.splitlines():
        s = line.strip()
        if s.startswith("stops  kept"):
            stops = s.split(":", 1)[1].split()[0]
        elif s.startswith("trips  kept"):
            trips = s.split(":", 1)[1].strip()
        elif s.startswith("transfers"):
            transfers = s.split(":", 1)[1].strip().split()[0]

    # raw/ is large and now redundant: rail/ and norm/ hold everything the study
    # reads. Deleting it keeps a 40-feed workdir inside a few gigabytes.
    shutil.rmtree(raw_dir, ignore_errors=True)

    return {"slug": slug, "status": "ok",
            "detail": f"{'reused pinned zip' if reused else 'downloaded'}; "
                      f"stops {stops}, trips {trips}, transfers {transfers}"}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workdir", default="feeds", help="where to put zips and feeds (default: feeds)")
    ap.add_argument("--only", default="", help="comma-separated slugs to process")
    ap.add_argument("--skip", default="", help="comma-separated slugs to skip")
    ap.add_argument("--list", action="store_true", help="print the manifest and exit")
    ap.add_argument("--verify", action="store_true", help="check local zips against the lock and exit")
    ap.add_argument("--refresh", action="store_true", help="re-download even when the pin matches")
    args = ap.parse_args()

    manifest = load_manifest()
    feeds = manifest["feeds"]

    if args.list:
        log(f"{len(feeds)} feeds in {MANIFEST} (probed {manifest['probed']}):")
        for e in feeds:
            log(f"  {e['slug']:<22} {e['city']} ({e['country']}) — {e['agency']}")
        log(f"\n{len(manifest.get('unavailable', []))} known-unavailable:")
        for e in manifest.get("unavailable", []):
            log(f"  {e['slug']:<22} {e['reason']}")
        return 0

    only = {s for s in args.only.split(",") if s}
    skip = {s for s in args.skip.split(",") if s}
    selected = [e for e in feeds if (not only or e["slug"] in only) and e["slug"] not in skip]
    if only:
        unknown = only - {e["slug"] for e in feeds}
        if unknown:
            log(f"ERROR: unknown slug(s): {', '.join(sorted(unknown))}")
            return 2

    workdir = os.path.abspath(args.workdir)
    os.makedirs(workdir, exist_ok=True)
    lock_path = os.path.join(workdir, "feeds.lock.json")
    lock = load_lock(lock_path)

    results = []
    for i, entry in enumerate(selected, 1):
        log(f"[{i}/{len(selected)}] {entry['slug']} — {entry['city']}")
        try:
            r = process_feed(entry, workdir, lock, args.refresh, args.verify)
        except Exception as e:  # one bad feed must not end the run
            r = {"slug": entry["slug"], "status": "FAILED", "detail": str(e).split("\n")[0][:300]}
        results.append(r)
        log(f"      {r['status']}: {r['detail']}")
        if not args.verify:
            save_lock(lock_path, lock)  # save incrementally; a crash loses nothing

    ok = [r for r in results if r["status"] == "ok"]
    bad = [r for r in results if r["status"] not in ("ok", "skipped")]
    log("")
    log(f"=== {len(ok)}/{len(results)} feeds ok ===")
    for r in bad:
        log(f"  {r['status']:<9} {r['slug']}: {r['detail']}")
    if not args.verify:
        log(f"lock file: {lock_path}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
