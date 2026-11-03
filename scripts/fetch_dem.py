#!/usr/bin/env python3
"""Fetch SRTM elevation tiles (and optionally a geoid grid) for georay.

georay itself never touches the network. This script is the only thing that
does, and you run it deliberately: it writes plain files into a directory that
georay then reads. That keeps the binary free of libcurl and TLS, keeps
--selftest runnable offline, and keeps bulk downloading something you asked
for rather than something a render loop does on your behalf.

Typical use:

    scripts/fetch_dem.py --origin 48.1375,11.5755 --extent 3000
    ./build/georay --dem data/dem --origin 48.1375,11.5755 --extent 3000 \
                   --geoid-separation 47.0

Elevation comes from the AWS "elevation-tiles-prod" open-data bucket, which
serves SRTM in skadi layout: gzipped .hgt, no account, no API key. Tiles are
1 degree square, 3601x3601 at 1 arc-second (~30 m), about 7 MB compressed.
"""

import argparse
import gzip
import math
import os
import struct
import sys
import urllib.error
import urllib.request

SRTM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/skadi/{ns}/{tile}.hgt.gz"

# Identify ourselves. Open-data buckets and tile servers alike are within
# their rights to block anonymous bulk traffic, and a script that hides what
# it is makes that their problem instead of ours.
USER_AGENT = "georay-fetch-dem/1.0 (+https://github.com/; elevation data for a local simulation)"


def tile_name(lat, lon):
    """Name of the 1-degree tile containing a position.

    Floor, not truncation: at latitude -0.5 the tile is S01, and int() would
    call it N00 and fetch the wrong side of the equator.
    """
    la, lo = math.floor(lat), math.floor(lon)
    return "%s%02d%s%03d" % ("N" if la >= 0 else "S", abs(la),
                             "E" if lo >= 0 else "W", abs(lo))


def tiles_covering(lat, lon, half_extent_m):
    """Every tile a square scene of the given half-extent can touch.

    Uses a generous metres-per-degree so the bound is never short; fetching
    one tile more than strictly needed is far cheaper than a scene with a
    missing corner.
    """
    d_lat = half_extent_m / 110574.0
    # Longitude degrees shrink towards the poles; guard the cosine so a scene
    # near a pole asks for a sane span rather than dividing by zero.
    d_lon = half_extent_m / max(111320.0 * math.cos(math.radians(lat)), 1.0)

    names = []
    la = math.floor(lat - d_lat)
    while la <= math.floor(lat + d_lat):
        lo = math.floor(lon - d_lon)
        while lo <= math.floor(lon + d_lon):
            names.append(tile_name(la + 0.5, lo + 0.5))
            lo += 1
        la += 1
    return sorted(set(names))


def fetch_tile(name, directory, force=False):
    out_path = os.path.join(directory, name + ".hgt")
    if os.path.exists(out_path) and not force:
        print("  %s already present (%.1f MB)" % (name, os.path.getsize(out_path) / 1e6))
        return True

    url = SRTM_URL.format(ns=name[:3], tile=name)
    print("  %s <- %s" % (name, url))
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            compressed = response.read()
    except urllib.error.HTTPError as error:
        if error.code == 404:
            # Expected over open ocean, where SRTM simply has no tile. Not an
            # error worth stopping for.
            print("    no tile published (ocean, or outside SRTM's 60N-56S coverage)")
            return False
        print("    HTTP %s: %s" % (error.code, error.reason))
        return False
    except Exception as error:  # noqa: BLE001 - report and continue to the next tile
        print("    failed: %s" % error)
        return False

    raw = gzip.decompress(compressed)

    # Sanity-check the geometry before writing: a .hgt has no header, so a
    # truncated or redirected download is otherwise indistinguishable from
    # real data until georay renders a cliff.
    side = int(round(math.sqrt(len(raw) / 2)))
    if side * side * 2 != len(raw):
        print("    rejected: %d bytes is not a square array of int16" % len(raw))
        return False

    # Write to a temporary name and rename, so an interrupted run never leaves
    # a half-written tile that looks complete to georay.
    temp_path = out_path + ".partial"
    with open(temp_path, "wb") as handle:
        handle.write(raw)
    os.replace(temp_path, out_path)
    print("    %d x %d posts, %.1f arc-second, %.1f MB" %
          (side, side, 3600.0 / (side - 1), len(raw) / 1e6))
    return True


def convert_geoid_pgm(pgm_path, out_path):
    """Convert a GeographicLib geoid .pgm (e.g. egm96-5.pgm) to georay's grid.

    The messy part of geoid data is its packaging - PGM with the scale and
    offset hidden in comment lines, or NGA's ASCII grid, or a set of spherical
    harmonic coefficients. All of that is handled here, once, so the C++ side
    reads one trivially parseable binary and never learns about any of it.

    Download the source file yourself from
    https://geographiclib.sourceforge.io/html/geoid.html - it is a separate,
    larger download and not everyone needs it. For a scene a few tens of
    kilometres across, --geoid-separation with a single constant is accurate
    to a few centimetres and needs nothing at all.
    """
    with open(pgm_path, "rb") as handle:
        data = handle.read()

    if not data.startswith(b"P5"):
        raise SystemExit("%s: not a binary PGM" % pgm_path)

    # Header: "P5", then whitespace-separated width, height, maxval, with
    # '#' comment lines that carry Offset and Scale.
    offset = scale = None
    position = 2
    fields = []
    while len(fields) < 3:
        while position < len(data) and data[position : position + 1].isspace():
            position += 1
        if data[position : position + 1] == b"#":
            end = data.index(b"\n", position)
            comment = data[position:end].decode("latin-1")
            parts = comment.split()
            if len(parts) >= 3 and parts[1] == "Offset":
                offset = float(parts[2])
            if len(parts) >= 3 and parts[1] == "Scale":
                scale = float(parts[2])
            position = end + 1
            continue
        end = position
        while end < len(data) and not data[end : end + 1].isspace():
            end += 1
        fields.append(int(data[position:end]))
        position = end
    position += 1  # the single whitespace byte after maxval

    width, height, maxval = fields
    if maxval != 65535:
        raise SystemExit("%s: expected a 16-bit PGM" % pgm_path)
    if offset is None or scale is None:
        raise SystemExit("%s: missing Offset/Scale comments" % pgm_path)

    pixels = struct.unpack(">%dH" % (width * height), data[position : position + width * height * 2])

    # GeographicLib's grid spans the whole globe with the first row at +90 and
    # the first column at -180, and the last column repeating the first.
    lat_max, lat_min = 90.0, -90.0
    lon_min, lon_max = -180.0, 180.0

    # Re-quantise to decimetres in int16, which covers the geoid's full range
    # (about -107 m to +86 m) to 0.1 m - an order of magnitude finer than the
    # DEM's own vertical accuracy, so nothing is lost.
    out_scale, out_offset = 0.1, 0.0
    packed = bytearray()
    for raw_value in pixels:
        metres = raw_value * scale + offset
        packed += struct.pack("<h", int(round(metres / out_scale)))

    with open(out_path, "wb") as handle:
        handle.write(b"GEORAYG1")
        handle.write(struct.pack("<ii", height, width))
        handle.write(struct.pack("<dddd", lat_min, lat_max, lon_min, lon_max))
        handle.write(struct.pack("<dd", out_scale, out_offset))
        handle.write(bytes(packed))

    print("geoid grid: %d x %d posts -> %s (%.1f MB)" %
          (height, width, out_path, os.path.getsize(out_path) / 1e6))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    # Note the "=" in the help: argparse reads a leading "-" as a flag, so a
    # southern or western origin has to be written --origin=-33.9,18.4.
    parser.add_argument("--origin", default="48.1375,11.5755",
                        help="scene centre as LAT,LON (default: Munich). "
                             "For negative values use --origin=-33.9,18.4")
    parser.add_argument("--extent", type=float, default=3000.0,
                        help="half-extent of the scene in metres (default: 3000)")
    parser.add_argument("--out", default="data/dem", help="directory to write tiles into")
    parser.add_argument("--force", action="store_true", help="re-download tiles already present")
    parser.add_argument("--geoid-pgm", metavar="PATH",
                        help="convert a GeographicLib geoid .pgm into georay's grid format")
    parser.add_argument("--geoid-out", default="data/geoid.bin",
                        help="where to write the converted geoid grid")
    parser.add_argument("--dry-run", action="store_true",
                        help="list the tiles that would be fetched, and stop")
    args = parser.parse_args()

    try:
        lat_text, lon_text = args.origin.split(",")
        lat, lon = float(lat_text), float(lon_text)
    except ValueError:
        raise SystemExit("--origin wants LAT,LON, e.g. 48.1375,11.5755")
    if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
        raise SystemExit("--origin is not a valid position")

    if args.geoid_pgm:
        os.makedirs(os.path.dirname(os.path.abspath(args.geoid_out)), exist_ok=True)
        convert_geoid_pgm(args.geoid_pgm, args.geoid_out)

    names = tiles_covering(lat, lon, args.extent)
    print("scene centred on %.5f, %.5f with a %.0f m half-extent needs %d tile(s): %s"
          % (lat, lon, args.extent, len(names), " ".join(names)))
    if args.dry_run:
        return 0

    os.makedirs(args.out, exist_ok=True)
    fetched = sum(1 for name in names if fetch_tile(name, args.out, args.force))

    if fetched == 0:
        print("\nNo tiles were obtained. SRTM covers 60N to 56S and has no tiles over "
              "open ocean; check the origin.")
        return 1

    print("\n%d of %d tile(s) in %s" % (fetched, len(names), args.out))
    print("Now run:\n  ./build/georay --dem %s --origin %s --extent %.0f"
          % (args.out, args.origin, args.extent))
    if not args.geoid_pgm:
        print("\nSRTM heights are above the geoid, georay's are above the ellipsoid.")
        print("Pass --geoid-separation with the value for your area (about 47 m in")
        print("southern Germany, 34 m in Colorado, -30 m in southern India) or")
        print("georay will report heights that are off by that much. A single")
        print("constant is good to a few centimetres across a scene this size.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
