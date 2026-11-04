#!/usr/bin/env python3
"""Pre-fetch map tiles into georay's cache.

georay stitches its map from tiles at run time, choosing the zoom from how
close the view is, so what it needs on disk is tiles - not a pre-assembled
image. This script fills that cache ahead of time: before a demo, for an area
with no connectivity, or simply to avoid waiting while flying.

It is not required. With --online georay fetches what it needs itself. This is
for getting a region in one deliberate go rather than a window at a time.

    scripts/fetch_map.py --origin 52.5163,13.3777 --extent 2000 --zoom 17
    scripts/fetch_map.py --along missions/berlin-survey.plan --window 900 --zoom 17
    scripts/fetch_map.py --region 51.36,11.27,53.56,14.77 --zoom 12 --source basemapde

Tile servers have usage policies. OpenStreetMap's in particular is a donated
volunteer resource and treats runs over 250 tiles as bulk downloading;
basemap.de is a government open-data service and far more tolerant. This
script identifies itself, rate-limits, caches per tile, and refuses runs over
--max-tiles, which defaults to OSM's line.

No image decoding happens here - tiles are stored exactly as served, and
georay decodes them. Pillow is not needed.
"""

import argparse
import hashlib
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request

# Must match src/map/tilesource.cpp, so the script and georay's live fetcher
# share one cache rather than each filling their own.
SOURCES = {
    "esri": ("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery"
             "/MapServer/tile/{z}/{y}/{x}", 19),
    "basemapde": ("https://sgx.geodatenzentrum.de/wmts_basemapde/tile/1.0.0"
                  "/de_basemapde_web_raster_grau/default/GLOBAL_WEBMERCATOR/{z}/{y}/{x}.png", 18),
    "basemapde-colour": ("https://sgx.geodatenzentrum.de/wmts_basemapde/tile/1.0.0"
                         "/de_basemapde_web_raster_farbe/default/GLOBAL_WEBMERCATOR"
                         "/{z}/{y}/{x}.png", 18),
    "osm": ("https://tile.openstreetmap.org/{z}/{x}/{y}.png", 19),
}

USER_AGENT = ("georay-fetch-map/1.0 (UAV geolocation simulation; "
              "basemap for a local scene)")


def deg2tile(lat, lon, zoom):
    """Fractional slippy-map tile coordinates: the Web Mercator projection.

    y is not linear in latitude, and at the scale georay works at that
    matters - treating it as linear misplaces the map by over a metre across
    a 4 km scene. georay applies the same transform when it computes texture
    coordinates.
    """
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n
    lat_r = math.radians(max(min(lat, 85.05112878), -85.05112878))
    y = (1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r)) / math.pi) / 2.0 * n
    return x, y


def metres_per_pixel(lat, zoom):
    return 156543.03392 * math.cos(math.radians(lat)) / (2.0 ** zoom)


def cache_key(source_name, url_template):
    """Cache key for a source.

    The source's own name when it has one, so the cache is readable and so
    georay's live fetcher - which knows sources by name - lands on exactly the
    same files. A custom --url has no name and gets a stable digest instead.

    Keyed by source at all because two of them serve different pictures of the
    same tile, and a cache that confused them would quietly mix a street map
    into satellite imagery.
    """
    if source_name:
        return source_name
    return "url" + hashlib.md5(url_template.encode("utf-8")).hexdigest()[:8]


def tiles_for_box(lat, lon, half_extent_m, zoom):
    """Inclusive tile range covering a square around a position."""
    d_lat = half_extent_m / 110574.0
    # Longitude degrees shrink towards the poles; the guard keeps a polar box
    # from spanning the world.
    d_lon = half_extent_m / max(111320.0 * math.cos(math.radians(lat)), 1.0)
    x0, y0 = deg2tile(lat + d_lat, lon - d_lon, zoom)   # north-west
    x1, y1 = deg2tile(lat - d_lat, lon + d_lon, zoom)   # south-east
    return (int(math.floor(x0)), int(math.floor(y0)),
            int(math.floor(x1)), int(math.floor(y1)))


def read_route(path):
    """Waypoint positions from a QGC .plan or a QGC WPL list.

    Deliberately small: a corridor needs latitudes and longitudes only, so it
    skips the frames, altitudes and command semantics georay's own loader
    cares about.
    """
    text = open(path, encoding="utf-8", errors="replace").read()
    # Navigation commands only. A speed change or camera trigger carries no
    # position, and routing a corridor through one would send it to 0,0.
    NAV = {16, 17, 18, 19, 21, 22}
    points = []

    if text.lstrip().startswith("{"):
        for item in json.loads(text).get("mission", {}).get("items", []):
            if item.get("type") == "ComplexItem":
                continue
            if int(item.get("command", 16)) not in NAV:
                continue
            params = item.get("params") or []
            if len(params) >= 6 and params[4] and params[5]:
                points.append((float(params[4]), float(params[5])))
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


def corridor_tiles(points, half_width_m, zoom):
    """Tiles along a route.

    Fetching the route's bounding box would be the obvious thing and is
    wildly wasteful: a survey running 40 km east and 2 km north has a box
    twenty times the area actually flown.
    """
    wanted = set()
    step_m = half_width_m
    for i in range(len(points) - 1):
        (lat0, lon0), (lat1, lon1) = points[i], points[i + 1]
        mid = math.radians((lat0 + lat1) * 0.5)
        dy = (lat1 - lat0) * 110574.0
        dx = (lon1 - lon0) * 111320.0 * math.cos(mid)
        steps = max(1, int(math.ceil(math.hypot(dx, dy) / max(step_m, 1.0))))
        for k in range(steps + 1):
            t = k / float(steps)
            lat = lat0 + (lat1 - lat0) * t
            lon = lon0 + (lon1 - lon0) * t
            x0, y0, x1, y1 = tiles_for_box(lat, lon, half_width_m, zoom)
            for x in range(x0, x1 + 1):
                for y in range(y0, y1 + 1):
                    wanted.add((x, y))
    return sorted(wanted)


def region_tiles(south, west, north, east, zoom):
    x0, y0 = deg2tile(north, west, zoom)
    x1, y1 = deg2tile(south, east, zoom)
    return [(x, y)
            for x in range(int(math.floor(x0)), int(math.floor(x1)) + 1)
            for y in range(int(math.floor(y0)), int(math.floor(y1)) + 1)]


def fetch(tiles, zoom, url_template, key, cache_dir, delay, max_tiles, dry_run):
    print("%d tile(s) at zoom %d, about %.2f m per pixel"
          % (len(tiles), zoom, metres_per_pixel(52.5, zoom)))
    if len(tiles) > max_tiles:
        raise SystemExit(
            "refusing: %d tiles exceeds --max-tiles %d.\n"
            "Lower --zoom or the area. OpenStreetMap's policy treats runs over 250\n"
            "tiles as bulk downloading; their servers are donated resources not\n"
            "intended for it. A government open-data service is far more tolerant -\n"
            "raise --max-tiles deliberately if that is what you are using."
            % (len(tiles), max_tiles))
    if dry_run:
        return 0

    os.makedirs(cache_dir, exist_ok=True)
    fetched = cached = failed = 0
    for index, (x, y) in enumerate(tiles):
        path = os.path.join(cache_dir, "%s_%d_%d_%d.img" % (key, zoom, x, y))
        if os.path.exists(path) and os.path.getsize(path) > 0:
            cached += 1
            continue

        url = url_template.format(z=zoom, x=x, y=y)
        request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                data = response.read()
        except urllib.error.HTTPError as error:
            # 404 over ocean or outside a national service's coverage is
            # expected, not a fault.
            if error.code != 404:
                print("  %d/%d HTTP %s" % (x, y, error.code))
            failed += 1
            continue
        except Exception as error:  # noqa: BLE001
            print("  %d/%d failed: %s" % (x, y, error))
            failed += 1
            continue

        # Written to a temporary name and renamed, so an interrupted run never
        # leaves a half-written tile that looks complete to georay.
        with open(path + ".part", "wb") as handle:
            handle.write(data)
        os.replace(path + ".part", path)
        fetched += 1
        time.sleep(delay)

        if (index + 1) % 50 == 0:
            print("  %d/%d" % (index + 1, len(tiles)), end="\r", flush=True)

    print("\n%d fetched, %d already cached, %d failed -> %s"
          % (fetched, cached, failed, cache_dir))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--origin", default="52.5163,13.3777",
                        help="centre as LAT,LON. For negative values use --origin=-33.9,18.4")
    parser.add_argument("--extent", type=float, default=2000.0,
                        help="half-extent of the area in metres")
    parser.add_argument("--zoom", type=int, default=16)
    parser.add_argument("--source", choices=sorted(SOURCES), default="basemapde")
    parser.add_argument("--url", help="XYZ tile URL template, overriding --source")
    parser.add_argument("--cache", default="data/tiles", help="tile cache directory")
    parser.add_argument("--max-tiles", type=int, default=250,
                        help="refuse runs larger than this (OSM treats 250+ as bulk)")
    parser.add_argument("--delay", type=float, default=0.1,
                        help="seconds between requests, to stay a polite client")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--along", metavar="MISSION",
                        help="fetch a corridor along a .plan or .waypoints route")
    parser.add_argument("--window", type=float, default=1200.0,
                        help="half-width of the corridor in metres")
    parser.add_argument("--region", metavar="S,W,N,E",
                        help="fetch a bounding box, e.g. 51.36,11.27,53.56,14.77")
    args = parser.parse_args()

    source_url, source_max_zoom = SOURCES[args.source]
    url = args.url or source_url
    key = cache_key(None if args.url else args.source, url)
    if not args.url and args.zoom > source_max_zoom:
        raise SystemExit("%s serves up to zoom %d; asking for %d returns 404s"
                         % (args.source, source_max_zoom, args.zoom))

    if args.region:
        try:
            south, west, north, east = (float(v) for v in args.region.split(","))
        except ValueError:
            raise SystemExit("--region wants S,W,N,E")
        if not (north > south and east > west):
            raise SystemExit("--region bounds are inverted")
        tiles = region_tiles(south, west, north, east, args.zoom)
    elif args.along:
        tiles = corridor_tiles(read_route(args.along), args.window, args.zoom)
    else:
        try:
            lat, lon = (float(v) for v in args.origin.split(","))
        except ValueError:
            raise SystemExit("--origin wants LAT,LON")
        x0, y0, x1, y1 = tiles_for_box(lat, lon, args.extent, args.zoom)
        tiles = [(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]

    return fetch(tiles, args.zoom, url, key, args.cache, args.delay, args.max_tiles,
                 args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
