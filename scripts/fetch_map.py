#!/usr/bin/env python3
"""Fetch map tiles for georay and stitch them into one raster.

Like fetch_dem.py, this is the only thing that touches the network and you run
it deliberately; georay reads the result from disk. It also does all the messy
parts - HTTP, PNG decoding, Web Mercator tile arithmetic, stitching - so the
C++ side reads a single trivially parseable file and never learns about any of
it. That is the same division of labour the geoid converter uses.

Typical use:

    scripts/fetch_map.py --origin 52.5163,13.3777 --extent 2000 --out data/berlin.map

--source picks what you get: esri for satellite imagery, basemapde for a calm
greyscale street map (Germany only), osm for OpenStreetMap's standard style.

--source esri fetches satellite imagery instead of a street map, which is what
makes the simulated camera view look like an actual UAV feed. Imagery is also
where resolution runs out first: a camera at 200 m sees about 11 cm per pixel,
finer than any public basemap, so the view magnifies map pixels rather than
resolving more ground. Flying higher is the honest fix.

Tiles come from OpenStreetMap by default. Their tile servers are a donated
resource with a usage policy (https://operations.osmfoundation.org/policies/tiles/):
identify yourself, do not download in bulk, cache what you fetch, and credit
the project. This script does all four - it sets a real User-Agent, rate-limits
itself, never re-downloads a cached tile, refuses runs over --max-tiles, and
records the attribution string inside the raster so georay can display it.

--url points it at any XYZ tile service you are licensed to use, which is the
right way to use aerial imagery here: a photo basemap makes the simulated
camera view look like an actual UAV feed, but almost every imagery provider
requires an account and their own terms.
"""

import argparse
import hashlib
import io
import json
import math
import os
import struct
import sys
import time
import urllib.error
import urllib.request

# Named sources, so the common cases need no URL typing. The template keys
# are {z}/{x}/{y}; note that ESRI orders them z/y/x, which is why the
# placeholders are named rather than positional.
SOURCES = {
    "osm": (
        "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        "(C) OPENSTREETMAP CONTRIBUTORS",
    ),
    "esri": (
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery"
        "/MapServer/tile/{z}/{y}/{x}",
        "IMAGERY (C) ESRI, MAXAR, EARTHSTAR GEOGRAPHICS",
    ),
    # Official German cartography, and far calmer to look at than OSM's
    # standard style: muted greys, no colour-coded land use, labels that stay
    # out of the way. Germany only. Licence: dl-de/by-2-0.
    "basemapde": (
        "https://sgx.geodatenzentrum.de/wmts_basemapde/tile/1.0.0"
        "/de_basemapde_web_raster_grau/default/GLOBAL_WEBMERCATOR/{z}/{y}/{x}.png",
        "(C) GEOBASIS-DE / BKG (DL-DE/BY-2-0)",
    ),
    "basemapde-colour": (
        "https://sgx.geodatenzentrum.de/wmts_basemapde/tile/1.0.0"
        "/de_basemapde_web_raster_farbe/default/GLOBAL_WEBMERCATOR/{z}/{y}/{x}.png",
        "(C) GEOBASIS-DE / BKG (DL-DE/BY-2-0)",
    ),
}
DEFAULT_URL, DEFAULT_ATTRIBUTION = SOURCES["osm"]

# OSM blocks unidentified clients, and is entitled to: a tile server cannot
# tell a simulation from a scraper unless the client says so.
USER_AGENT = ("georay-fetch-map/1.0 (UAV geolocation simulation; "
              "basemap for a local scene; contact: local user)")

MAGIC = b"GEORAYM1"
TILE_PIXELS = 256
ATTRIBUTION_BYTES = 128


def deg2tile(lat, lon, zoom):
    """Fractional slippy-map tile coordinates. The Web Mercator projection.

    y is NOT linear in latitude, and that matters at the scale georay works
    at: treating it as linear across a 4 km scene misplaces the map by over a
    metre on the ground. georay therefore applies this same transform when it
    computes texture coordinates, rather than interpolating in latitude.
    """
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n
    lat_r = math.radians(max(min(lat, 85.05112878), -85.05112878))
    y = (1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r)) / math.pi) / 2.0 * n
    return x, y


def metres_per_pixel(lat, zoom):
    return 156543.03392 * math.cos(math.radians(lat)) / (2.0 ** zoom)


def fetch_tile(url_template, zoom, x, y, cache_dir):
    # Keyed by the URL template as well as the coordinates: two sources serve
    # different pictures of the same tile, and a cache that confused them
    # would quietly mix a street map into satellite imagery.
    #
    # hashlib rather than hash(): Python randomises string hashing per
    # process, so hash() would give a different key every run and the cache
    # would never hit - re-downloading every tile each time, which is the one
    # thing a tile usage policy asks you not to do.
    key = hashlib.md5(url_template.encode("utf-8")).hexdigest()[:8]
    cache_path = os.path.join(cache_dir, "%s_%d_%d_%d.img" % (key, zoom, x, y))
    if os.path.exists(cache_path):
        with open(cache_path, "rb") as handle:
            return handle.read(), True

    url = url_template.format(z=zoom, x=x, y=y)
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(request, timeout=60) as response:
        data = response.read()

    with open(cache_path, "wb") as handle:
        handle.write(data)
    return data, False


def fetch_window(lat, lon, extent, zoom, url, attribution_text, cache_dir,
                 max_tiles, delay, max_pixels, out_path, dry_run=False, quiet=False):
    """Fetch one square window and write it as a georay .map raster.

    Returns the number of tiles actually downloaded, or -1 if the window was
    refused or produced nothing.
    """
    def say(*parts):
        if not quiet:
            print(*parts)

    # Window bounds. Longitude degrees shrink towards the poles, hence the
    # cosine; the guard keeps a polar window from asking for the whole world.
    d_lat = extent / 110574.0
    d_lon = extent / max(111320.0 * math.cos(math.radians(lat)), 1.0)

    # North-west and south-east corners, in fractional tile space. Note y
    # grows southwards, so the northern edge gives the smaller y.
    x_nw, y_nw = deg2tile(lat + d_lat, lon - d_lon, zoom)
    x_se, y_se = deg2tile(lat - d_lat, lon + d_lon, zoom)

    tile_x0, tile_y0 = int(math.floor(x_nw)), int(math.floor(y_nw))
    tile_x1, tile_y1 = int(math.floor(x_se)), int(math.floor(y_se))
    tiles_x = tile_x1 - tile_x0 + 1
    tiles_y = tile_y1 - tile_y0 + 1
    total = tiles_x * tiles_y

    width, height = tiles_x * TILE_PIXELS, tiles_y * TILE_PIXELS
    native_resolution = metres_per_pixel(lat, zoom)

    # Downsampling after stitching rather than fetching a lower zoom: the
    # extra detail is averaged in rather than thrown away, and the tile grid
    # still lines up with the Mercator maths georay uses.
    longest = max(width, height)
    shrink = 1.0
    if max_pixels > 0 and longest > max_pixels:
        shrink = max_pixels / float(longest)

    say("  %d x %d tiles = %d, raster %d x %d, %.2f m/px native, %.2f m/px stored"
        % (tiles_x, tiles_y, total, int(width * shrink), int(height * shrink),
           native_resolution, native_resolution / shrink))

    if total > max_tiles:
        say("  refusing: %d tiles exceeds --max-tiles %d. Lower --zoom or the window size.\n"
            "  OpenStreetMap's policy treats runs over 250 tiles as bulk downloading,\n"
            "  and their servers are donated resources not intended for it."
            % (total, max_tiles))
        return -1

    if dry_run:
        return 0

    try:
        from PIL import Image
    except ImportError:
        raise SystemExit("this script needs Pillow to decode tiles: pip install pillow")

    os.makedirs(cache_dir, exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    canvas = Image.new("RGB", (width, height), (128, 128, 128))
    fetched = cached = failed = 0
    for row, ty in enumerate(range(tile_y0, tile_y1 + 1)):
        for col, tx in enumerate(range(tile_x0, tile_x1 + 1)):
            try:
                data, from_cache = fetch_tile(url, zoom, tx, ty, cache_dir)
            except urllib.error.HTTPError as error:
                say("  tile %d/%d HTTP %s" % (tx, ty, error.code))
                failed += 1
                continue
            except Exception as error:  # noqa: BLE001
                say("  tile %d/%d failed: %s" % (tx, ty, error))
                failed += 1
                continue

            if from_cache:
                cached += 1
            else:
                fetched += 1
                # Only sleep for requests actually made; replaying from cache
                # should be instant.
                time.sleep(delay)

            tile_image = Image.open(io.BytesIO(data)).convert("RGB")
            if tile_image.size != (TILE_PIXELS, TILE_PIXELS):
                tile_image = tile_image.resize((TILE_PIXELS, TILE_PIXELS))
            canvas.paste(tile_image, (col * TILE_PIXELS, row * TILE_PIXELS))

        if not quiet:
            print("  row %d/%d" % (row + 1, tiles_y), end="\r", flush=True)
    if not quiet:
        print()

    if failed == total:
        say("  no tiles were fetched for this window; check --url and your connection")
        return -1

    if shrink < 1.0:
        canvas = canvas.resize((int(width * shrink), int(height * shrink)), Image.LANCZOS)
        width, height = canvas.size

    attribution = attribution_text.encode("ascii", "replace")[: ATTRIBUTION_BYTES - 1]
    attribution += b"\0" * (ATTRIBUTION_BYTES - len(attribution))

    # georay map raster, little-endian:
    #   magic[8], zoom, tile_x, tile_y, tiles_x, tiles_y, width, height,
    #   attribution[128], then width*height RGB triples, row 0 northernmost.
    #
    # Written to a temporary name and renamed, so an interrupted run never
    # leaves a half-written window that looks complete to georay.
    temp_path = out_path + ".partial"
    with open(temp_path, "wb") as handle:
        handle.write(MAGIC)
        handle.write(struct.pack("<iiiiiii", zoom, tile_x0, tile_y0, tiles_x, tiles_y,
                                 width, height))
        handle.write(attribution)
        handle.write(canvas.tobytes())
    os.replace(temp_path, out_path)

    say("  %d fetched, %d from cache, %d failed -> %s (%.1f MB)"
        % (fetched, cached, failed, out_path, os.path.getsize(out_path) / 1e6))
    if failed:
        say("  WARNING: %d tiles are missing and show as flat grey." % failed)
    return fetched


def read_route(path):
    """Waypoint positions from a QGC .plan or a QGC WPL list.

    A deliberately small reader: a corridor only needs latitudes and
    longitudes, so it skips the frames, altitudes and command semantics that
    georay's own loader cares about.
    """
    text = open(path, encoding="utf-8", errors="replace").read()
    stripped = text.lstrip()

    # Navigation commands only. A speed change or a camera trigger carries no
    # position, and routing a corridor through one would send it to 0,0.
    NAV = {16, 17, 18, 19, 21, 22}
    points = []

    if stripped.startswith("{"):
        plan = json.loads(text)
        for item in plan.get("mission", {}).get("items", []):
            if item.get("type") == "ComplexItem":
                continue
            if int(item.get("command", 16)) not in NAV:
                continue
            params = item.get("params") or []
            if len(params) < 6:
                continue
            lat, lon = params[4], params[5]
            if lat and lon:
                points.append((float(lat), float(lon)))
    else:
        for line in text.splitlines()[1:]:
            fields = line.split()
            if len(fields) < 12:
                continue
            seq, command = int(float(fields[0])), int(float(fields[3]))
            lat, lon = float(fields[8]), float(fields[9])
            # Sequence 0 is the home position by convention, not a leg.
            if seq == 0 or command not in NAV or (not lat and not lon):
                continue
            points.append((lat, lon))

    if not points:
        raise SystemExit("%s: no navigable waypoints found" % path)
    return points


def region_windows(south, west, north, east, window_m, overlap=0.75):
    """Window centres tiling a bounding box.

    For covering a whole region rather than a flight path. The overlap is the
    same as the corridor's and for the same reason: georay holds one window
    at a time, so without it the aircraft crosses a seam with nothing on
    either side of it.

    Overlapping raw windows do duplicate ground - about 1.8x at 0.75 - which
    is the dominant cost of covering an area this way. It is the price of
    being able to fly anywhere in it without a seam.
    """
    step_m = window_m * 2.0 * overlap
    mid_lat = math.radians((south + north) * 0.5)
    step_lat = step_m / 110574.0
    step_lon = step_m / max(111320.0 * math.cos(mid_lat), 1.0)

    centres = []
    lat = south + step_lat * 0.5
    while lat - step_lat * 0.5 < north:
        lon = west + step_lon * 0.5
        while lon - step_lon * 0.5 < east:
            centres.append((lat, lon))
            lon += step_lon
        lat += step_lat
    return centres


def corridor_windows(points, window_m, overlap=0.75):
    """Window centres spaced along a route.

    Fetching the route's bounding box would be the obvious thing and is
    wildly wasteful: a survey running 40 km east and 2 km north has a box
    twenty times the area actually flown. Walking the legs and placing
    overlapping windows along them fetches the corridor instead.

    The overlap matters. georay holds one window at a time, so without it the
    aircraft would cross a seam with nothing on either side of it.
    """
    step = window_m * 2.0 * overlap
    centres = []

    def push(lat, lon):
        # Skip a centre already covered, so legs that double back over the
        # same ground do not fetch it twice.
        for clat, clon in centres:
            dy = (lat - clat) * 110574.0
            dx = (lon - clon) * 111320.0 * math.cos(math.radians(lat))
            if math.hypot(dx, dy) < step * 0.5:
                return
        centres.append((lat, lon))

    push(*points[0])
    for i in range(len(points) - 1):
        (lat0, lon0), (lat1, lon1) = points[i], points[i + 1]
        mid_lat = math.radians((lat0 + lat1) * 0.5)
        dy = (lat1 - lat0) * 110574.0
        dx = (lon1 - lon0) * 111320.0 * math.cos(mid_lat)
        length = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(length / step)))
        for k in range(1, steps + 1):
            t = k / float(steps)
            push(lat0 + (lat1 - lat0) * t, lon0 + (lon1 - lon0) * t)
    return centres


def run_windows(centres, args, url, attribution_text, prefix):
    """Fetch a list of window centres into args.out_dir."""
    written = skipped = refused = 0
    for i, (clat, clon) in enumerate(centres):
        # The position is in the name, so a directory of windows is
        # self-describing and a partial fetch can be resumed by re-running.
        name = "%s%03d_%+.4f_%+.4f" % (prefix, i, clat, clon)
        out = os.path.join(args.out_dir, name + ".map")
        if os.path.exists(out) and not args.force:
            skipped += 1
            continue
        print("[%d/%d] %s" % (i + 1, len(centres), name))
        if fetch_window(clat, clon, args.window, args.zoom, url, attribution_text,
                        args.cache, args.max_tiles, args.delay, args.max_pixels,
                        out, args.dry_run, quiet=True) < 0:
            refused += 1
        else:
            written += 1

    print("\n%d written, %d already present, %d refused -> %s"
          % (written, skipped, refused, args.out_dir))
    if not args.dry_run and (written or skipped):
        total = sum(os.path.getsize(os.path.join(args.out_dir, f))
                    for f in os.listdir(args.out_dir) if f.endswith(".map"))
        print("%s now holds %.2f GB" % (args.out_dir, total / 1e9))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--origin", default="52.5163,13.3777",
                        help="scene centre as LAT,LON. For negative values use --origin=-33.9,18.4")
    parser.add_argument("--extent", type=float, default=2000.0,
                        help="half-extent of the scene in metres")
    parser.add_argument("--zoom", type=int, default=16,
                        help="tile zoom level (default 16, about 1.5 m per pixel at mid latitudes)")
    parser.add_argument("--out", default="data/map.bin", help="raster to write")
    parser.add_argument("--source", choices=sorted(SOURCES), default="osm",
                        help="named tile source: osm or basemapde (street maps) or esri (imagery)")
    parser.add_argument("--url", help="XYZ tile URL template, overriding --source")
    parser.add_argument("--max-pixels", type=int, default=4096,
                        help="downsample the stitched raster to at most this on its longest "
                             "side (default 4096). Caps file size and stays inside the GPU's "
                             "texture limit.")
    parser.add_argument("--attribution",
                        help="credit line georay displays while the map is shown; "
                             "defaults to the one the chosen --source requires")
    parser.add_argument("--cache", default="data/tiles", help="per-tile download cache")
    parser.add_argument("--max-tiles", type=int, default=250,
                        help="refuse windows larger than this (OSM treats 250+ as bulk)")
    parser.add_argument("--delay", type=float, default=0.12,
                        help="seconds between requests, to stay a polite client")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--force", action="store_true",
                        help="re-fetch windows that already exist")
    parser.add_argument("--along", metavar="MISSION",
                        help="fetch a corridor along a .plan or .waypoints route instead of a "
                             "single square, writing one window per stretch")
    parser.add_argument("--window", type=float, default=1200.0,
                        help="half-extent of each corridor window in metres (default 1200)")
    parser.add_argument("--out-dir", default="data/windows",
                        help="directory to write corridor or region windows into")
    parser.add_argument("--region", metavar="S,W,N,E",
                        help="tile a bounding box with windows instead of fetching one "
                             "square, e.g. --region 51.36,11.27,53.56,14.77 for Berlin "
                             "and Brandenburg")
    args = parser.parse_args()

    # A source carries its own attribution. Overriding the URL without saying
    # who to credit would silently keep the previous source's credit line,
    # which is worse than no credit at all.
    source_url, source_attribution = SOURCES[args.source]
    url = args.url or source_url
    attribution_text = args.attribution or (source_attribution if not args.url
                                            else "UNKNOWN TILE SOURCE")

    if not 1 <= args.zoom <= 19:
        raise SystemExit("--zoom must be between 1 and 19")

    # --- a grid of windows over a region ---------------------------------
    if args.region:
        try:
            south, west, north, east = (float(v) for v in args.region.split(","))
        except ValueError:
            raise SystemExit("--region wants S,W,N,E")
        if not (north > south and east > west):
            raise SystemExit("--region bounds are inverted")

        centres = region_windows(south, west, north, east, args.window)
        km_lat = (north - south) * 110.574
        km_lon = (east - west) * 111.320 * math.cos(math.radians((south + north) * 0.5))
        per_window = metres_per_pixel((south + north) * 0.5, args.zoom)
        print("region %.0f x %.0f km -> %d window(s) of %.0f m half-extent at zoom %d "
              "(%.1f m/px)" % (km_lon, km_lat, len(centres), args.window, args.zoom,
                               per_window))
        return run_windows(centres, args, url, attribution_text, "r")

    # --- a corridor along a flight plan ----------------------------------
    if args.along:
        points = read_route(args.along)
        centres = corridor_windows(points, args.window)
        print("%s: %d waypoints -> %d window(s) of %.0f m half-extent at zoom %d"
              % (args.along, len(points), len(centres), args.window, args.zoom))

        return run_windows(centres, args, url, attribution_text, "w")

    # --- a single square window ------------------------------------------
    try:
        lat_text, lon_text = args.origin.split(",")
        lat, lon = float(lat_text), float(lon_text)
    except ValueError:
        raise SystemExit("--origin wants LAT,LON")

    print("scene %.5f,%.5f +/- %.0f m at zoom %d" % (lat, lon, args.extent, args.zoom))
    return 0 if fetch_window(lat, lon, args.extent, args.zoom, url, attribution_text,
                             args.cache, args.max_tiles, args.delay, args.max_pixels,
                             args.out, args.dry_run) >= 0 else 1


if __name__ == "__main__":
    sys.exit(main())
