# georay

A prototype for UAV camera pixel geolocation. See README.md for what it does.

## Design authority

`src/geo/` is the part that matters; everything else exists to exercise and
display it. Two rules follow from that:

- **Keep GL out of `src/geo/`.** It is a separate CMake target (`georay_geo`)
  with no GL dependency, and `--selftest` runs headless because of it. If
  something in there needs a GL header, it is in the wrong directory.
- **The maths is double precision.** Floats appear only after coordinates have
  been made local (ENU) for rendering. A float ECEF coordinate quantises to
  about half a metre, which is larger than the errors this tool studies.

## Frames

Every geolocation bug is a frame or sign error, and they are all silent. The
conventions are documented at the top of `src/geo/pose.hpp` and pinned down by
`--selftest`. Read that header before touching the rotation chain; do not
re-derive it from the code.

## Network

georay never makes a network request. `scripts/fetch_dem.py` is the only thing
in the repo that does, and the user runs it deliberately. Keep it that way: it
is what lets `--selftest` run offline, keeps the binary free of libcurl and
TLS, and keeps bulk downloading an explicit act rather than something a render
loop does on someone's behalf.

Messy third-party formats (PNG map tiles, geoid PGMs, and GeoTIFF if it is
ever added) are parsed in Python at fetch time and written out as simple
arrays. The C++ side reads a handful of trivially parseable formats and never
learns about the others - which is also why georay needs no image decoder.

Tile servers have usage policies. fetch_map.py identifies itself, rate-limits,
caches per tile and refuses bulk runs; keep it that way, and keep the
attribution travelling inside the raster so georay displays whatever the
source requires.

## Height systems

Three different heights are in play and mixing them is silent:

- `Geodetic::altitude_m` - above the WGS84 ellipsoid. What GPS reports.
- DEM heights - above the geoid. What a map calls "metres above sea level".
- `Terrain` heights - ENU, i.e. relative to the frame origin's altitude.

And one horizontal projection: map rasters are Web Mercator, not latitude and
longitude. `MapRaster::uvFor` owns that transform and must stay identical to
the one fetch_map.py lays the raster out with.

`Terrain::loadFromDem` is the one place all three meet. Before changing it,
read the header comments in `geo/geoid.hpp` and `geo/terrain.hpp`. A plain
subtraction instead of a conversion through the ellipsoid drops the
tangent-plane curvature term and bakes a dome into the terrain; the selftest
checks for exactly that, against the closed-form `d^2 / 2R`.

## Mission files

`src/geo/mission.hpp` reads QGroundControl `.plan` and `QGC WPL` files. The
altitude frame is the part to be careful with: MAV_FRAME_GLOBAL_RELATIVE_ALT
is measured from the home position, not from sea level, and confusing the two
is a 38 m error over Berlin. `resolveAltitude` is the single place that
decides; the selftest checks the frames are not interchangeable.

`src/geo/json.hpp` is a minimal JSON reader written for this rather than a
library, because the subset needed is a handful of numbers and a
general-purpose header would be the largest dependency in the project. It is
still a parser reading files from disk, so keep it failing rather than
misbehaving: bounded depth and length, checked conversions, no partially-built
values on error.

## Text rendering

The HUD font is a bitmap atlas baked from a TTF by `scripts/bake_font.py` and
committed. Keep it that way: it is what lets georay draw a real programming
font without linking a font library, and baking at the exact draw sizes is
what keeps it crisp.

Faces are named sizes, not scale factors - scaling a baked face just blurs it.
Lay HUD panels out from `TextRenderer::lineHeight()` and `advance()` rather
than from constants; hard-coded offsets collided the moment the font changed.

## The floating origin

The ENU frame re-anchors to the aircraft as it flies, because one tangent
plane cannot span more than a few tens of kilometres (`d^2/2R` is 4.4 km
across Brandenburg). The geolocation maths is unaware of this and must stay
that way.

Anything held in ENU metres is silently relocated by a re-anchor. The ones
that bit: the flight pattern's own centre, the gimbal's track target, the view
camera's target, and the mission's progress along its route. If you add state
in ENU, keep the geodetic truth beside it and restore it in
`App::reanchorFrame`, or the thing it describes will drift away from the
ground it was supposed to be attached to.

`testFloatingOrigin` in the selftest is the invariant: the same pixel, from
the same aircraft, must geolocate to the same place whatever the anchor. Note
the aircraft in that test is deliberately well above the terrain - with the
camera underground the ray marcher returns its own position, which is
frame-independent for trivial reasons and made the test vacuous the first time
it was written.

## Streaming

Three caches, all bounded, all for the same reason - the aircraft travels and
does not come back:

- `DemSource` holds at most `kDefaultTileLimit` one-degree tiles (26 MB each),
  least-recently-used first out.
- `MapLibrary` holds exactly one window. More would need a texture array and
  a redirect table in the shader; one window plus the re-anchor that rebuilds
  the terrain mesh anyway is the same machinery for a fraction of the cost.
- `MapLibrary::open` reads headers only. A region-sized library is dozens of
  windows of tens of megabytes; reading them in full to learn four numbers
  each would be a gigabyte of disk traffic before the first frame.

The map window must change in lockstep with the terrain mesh rebuild, because
the mesh carries baked texture coordinates. Swapping one without the other
misregisters the map.

## Process

- Verify every change: clean rebuild + `./build/georay --selftest` + a runtime
  smoke test (`DISPLAY=:1 timeout 20 ./build/georay`, termination by timeout is
  expected). For anything that changes what is drawn, also
  `DISPLAY=:1 ./build/georay --screenshot /tmp/shot.bmp` and look at it.
- The DEM path has no committed test data on purpose. To exercise it without
  the network, write a full-size tile (3601x3601 big-endian int16, no header,
  25,934,402 bytes) with numpy and point `--dem` at the directory.
- Prove new tests non-vacuous: break the thing they cover, confirm the test
  fails, then restore it, before committing.
- When a test fails, work out whether the code or the expectation is wrong
  before changing either. Several of the existing expectations are
  counterintuitive and correct - the rolled-nadir case in `testPose` is the
  clearest example.
- Commit locally when a fix is verified; don't push unless asked.
