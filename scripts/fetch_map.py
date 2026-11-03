#!/usr/bin/env python3
"""Fetch map tiles for georay and stitch them into one raster.

Like fetch_dem.py, this is the only thing that touches the network and you run
it deliberately; georay reads the result from disk. It also does all the messy
parts - HTTP, PNG decoding, Web Mercator tile arithmetic, stitching - so the
C++ side reads a single trivially parseable file and never learns about any of
it. That is the same division of labour the geoid converter uses.

Typical use:

    scripts/fetch_map.py --origin 52.5163,13.3777 --extent 2000 --out data/berlin.map

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
    key = "%08x" % (hash(url_template) & 0xFFFFFFFF)
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
                        help="named tile source: osm (street map) or esri (satellite imagery)")
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
                        help="refuse runs larger than this (OSM treats 250+ as bulk)")
    parser.add_argument("--delay", type=float, default=0.12,
                        help="seconds between requests, to stay a polite client")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    # A source carries its own attribution. Overriding the URL without saying
    # who to credit would silently keep the previous source's credit line,
    # which is worse than no credit at all.
    source_url, source_attribution = SOURCES[args.source]
    url = args.url or source_url
    attribution_text = args.attribution or (source_attribution if not args.url
                                            else "UNKNOWN TILE SOURCE")

    try:
        lat_text, lon_text = args.origin.split(",")
        lat, lon = float(lat_text), float(lon_text)
    except ValueError:
        raise SystemExit("--origin wants LAT,LON")
    if not 1 <= args.zoom <= 19:
        raise SystemExit("--zoom must be between 1 and 19")

    # Scene bounds. Longitude degrees shrink towards the poles, hence the
    # cosine; the guard keeps a polar scene from asking for the whole world.
    d_lat = args.extent / 110574.0
    d_lon = args.extent / max(111320.0 * math.cos(math.radians(lat)), 1.0)

    # North-west and south-east corners, in fractional tile space. Note y
    # grows southwards, so the northern edge gives the smaller y.
    x_nw, y_nw = deg2tile(lat + d_lat, lon - d_lon, args.zoom)
    x_se, y_se = deg2tile(lat - d_lat, lon + d_lon, args.zoom)

    tile_x0, tile_y0 = int(math.floor(x_nw)), int(math.floor(y_nw))
    tile_x1, tile_y1 = int(math.floor(x_se)), int(math.floor(y_se))
    tiles_x = tile_x1 - tile_x0 + 1
    tiles_y = tile_y1 - tile_y0 + 1
    total = tiles_x * tiles_y

    width, height = tiles_x * TILE_PIXELS, tiles_y * TILE_PIXELS
    print("scene %.5f,%.5f +/- %.0f m at zoom %d" % (lat, lon, args.extent, args.zoom))
    print("  %d x %d tiles = %d, raster %d x %d, %.1f MB on disk"
          % (tiles_x, tiles_y, total, width, height, width * height * 3 / 1e6))
    native_resolution = metres_per_pixel(lat, args.zoom)
    # Downsampling after stitching rather than fetching a lower zoom: the
    # extra detail is averaged in rather than thrown away, and the tile grid
    # still lines up with the Mercator maths georay uses.
    longest = max(width, height)
    shrink = 1.0
    if args.max_pixels > 0 and longest > args.max_pixels:
        shrink = args.max_pixels / float(longest)
    print("  ground resolution %.2f m/px native, %.2f m/px after scaling"
          % (native_resolution, native_resolution / shrink))
    if shrink < 1.0:
        print("  raster will be scaled to %d x %d (%.1f MB)"
              % (int(width * shrink), int(height * shrink),
                 int(width * shrink) * int(height * shrink) * 3 / 1e6))

    if total > args.max_tiles:
        raise SystemExit(
            "refusing: %d tiles exceeds --max-tiles %d.\n"
            "Lower --zoom or --extent. OpenStreetMap's tile policy treats runs over\n"
            "250 tiles as bulk downloading, which their servers are donated resources\n"
            "and not intended for." % (total, args.max_tiles))

    if args.dry_run:
        return 0

    try:
        from PIL import Image
    except ImportError:
        raise SystemExit("this script needs Pillow to decode PNG tiles: pip install pillow")

    os.makedirs(args.cache, exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)

    canvas = Image.new("RGB", (width, height), (128, 128, 128))
    fetched = cached = failed = 0
    for row, ty in enumerate(range(tile_y0, tile_y1 + 1)):
        for col, tx in enumerate(range(tile_x0, tile_x1 + 1)):
            try:
                data, from_cache = fetch_tile(url, args.zoom, tx, ty, args.cache)
            except urllib.error.HTTPError as error:
                print("  tile %d/%d HTTP %s" % (tx, ty, error.code))
                failed += 1
                continue
            except Exception as error:  # noqa: BLE001
                print("  tile %d/%d failed: %s" % (tx, ty, error))
                failed += 1
                continue

            if from_cache:
                cached += 1
            else:
                fetched += 1
                # Only sleep for requests actually made; replaying from cache
                # should be instant.
                time.sleep(args.delay)

            import io
            tile_image = Image.open(io.BytesIO(data)).convert("RGB")
            if tile_image.size != (TILE_PIXELS, TILE_PIXELS):
                tile_image = tile_image.resize((TILE_PIXELS, TILE_PIXELS))
            canvas.paste(tile_image, (col * TILE_PIXELS, row * TILE_PIXELS))

        print("  row %d/%d" % (row + 1, tiles_y), end="\r", flush=True)
    print()

    if failed == total:
        raise SystemExit("no tiles were fetched; check --url and your connection")

    if shrink < 1.0:
        canvas = canvas.resize((int(width * shrink), int(height * shrink)), Image.LANCZOS)
        width, height = canvas.size

    attribution = attribution_text.encode("ascii", "replace")[: ATTRIBUTION_BYTES - 1]
    attribution += b"\0" * (ATTRIBUTION_BYTES - len(attribution))

    # georay map raster, little-endian:
    #   magic[8], zoom, tile_x, tile_y, tiles_x, tiles_y, width, height,
    #   attribution[128], then width*height RGB triples, row 0 northernmost.
    temp_path = args.out + ".partial"
    with open(temp_path, "wb") as handle:
        handle.write(MAGIC)
        handle.write(struct.pack("<iiiiiii", args.zoom, tile_x0, tile_y0, tiles_x, tiles_y,
                                 width, height))
        handle.write(attribution)
        handle.write(canvas.tobytes())
    os.replace(temp_path, args.out)

    print("%d fetched, %d from cache, %d failed -> %s (%.1f MB)"
          % (fetched, cached, failed, args.out, os.path.getsize(args.out) / 1e6))
    if failed:
        print("WARNING: %d tiles are missing and show as flat grey in the scene." % failed)
    return 0


if __name__ == "__main__":
    sys.exit(main())
