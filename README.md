# georay

A simulation and visualisation prototype for **UAV pixel geolocation**: given a
camera's intrinsic and extrinsic parameters, the aircraft's GPS fix, altitude
and attitude, and a ground model — where on Earth is the thing in this pixel?

It is a prototype for a photogrammetry pipeline, built so the answer can be
checked rather than trusted. The window shows a 3D scene you can fly around,
and inset into it the image the simulated UAV camera is producing. Click a
pixel in that inset and georay traces the ray, intersects it with the terrain,
and reports a latitude and longitude — drawing the ray in the 3D view as it
goes. If the ray misses the feature you aimed at, you can see it.

![georay](docs/screenshot.png)

## Building

Needs a C++17 compiler, CMake 3.16+, OpenGL, SDL2, GLEW and glm. SDL2, GLEW
and glm are used from the system if present and fetched from source otherwise,
so a bare checkout builds either way.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build -j
```

On Debian/Ubuntu, installing the system packages first makes configuring much
faster:

```bash
sudo apt install build-essential cmake libsdl2-dev libglew-dev libglm-dev
```

## Running

```bash
./build/georay --scene berlin
```

That flies a survey orbit over the Brandenburg Gate on real elevation data
with an OpenStreetMap basemap draped over it — click anywhere in the camera
view and you get a coordinate you can check on a real map. `scenes/` also has
`munich` and `zugspitze`. Fetch a scene's data once before first use:

```bash
scripts/fetch_dem.py --origin 52.5163,13.3777 --extent 2000
```
```bash
scripts/fetch_map.py --origin 52.5163,13.3777 --extent 2000 --out data/berlin.map
```

Without a scene you get synthetic terrain, which needs no files:

```bash
./build/georay
```

Other modes:

| Command | What it does |
| --- | --- |
| `./build/georay --selftest` | Runs the numeric verification suite and exits. No window, no GL — works headless. |
| `./build/georay --screenshot out.bmp` | Renders a few frames, saves a BMP, exits. |
| `./build/georay --size 1920x1080` | Window size. |
| `./build/georay --scene NAME` | Load `scenes/NAME.scene`. Later flags override it. |
| `./build/georay --origin LAT,LON --extent M` | Where the scene is, and how big. |

## Real maps

A map raster draped over the terrain is what makes a flat city worth flying.
Berlin is 36–40 m above sea level and essentially level, so elevation data
alone shows you nothing there; with the basemap, the simulated camera looks at
real streets and a geolocated pixel can be checked against a place you can
name.

```bash
scripts/fetch_map.py --origin 52.5163,13.3777 --extent 2000 --zoom 16 --out data/berlin.map
```

Tiles come from OpenStreetMap by default. Their servers are a donated resource
with a [usage policy](https://operations.osmfoundation.org/policies/tiles/):
identify yourself, don't bulk download, cache what you fetch, and credit the
project. The script does all four — real User-Agent, rate limited, per-tile
cache, and it refuses runs over `--max-tiles` (250, OSM's bulk threshold). The
attribution string travels *inside* the raster so georay displays it whenever
the map is on screen. `--url` points it at any XYZ service you are licensed to
use; aerial imagery makes the camera view look like a real UAV feed, but
essentially every imagery provider wants an account and their own terms.

**Web Mercator is handled properly, not approximated.** Map tiles are not in
latitude and longitude — their y axis is the Mercator projection, which is
distinctly non-linear in latitude. Treating it as linear would slide the map
off the ground by over a metre across a 4 km scene: small enough to look fine,
large enough to matter for a tool that reports positions in metres. georay
applies the same forward transform the fetch script laid the raster out with,
once per terrain post.

### Verified against real landmarks

Three independently-known Berlin coordinates were projected through the full
chain — pose, intrinsics, projection, Mercator UV, render — and checked against
where they land in the rendered camera view:

| Landmark | Projected to |
| --- | --- |
| Brandenburg Gate (52.51628, 13.37771) | the gate monument at the head of Pariser Platz |
| Holocaust Memorial (52.51390, 13.37859) | its stelae grid |
| Reichstag (52.51864, 13.37612) | the Reichstag building |

## Real elevation data

georay runs on synthetic terrain out of the box, and on real SRTM elevation
data when you give it some. Fetch tiles for an area:

```bash
scripts/fetch_dem.py --origin 48.1375,11.5755 --extent 3000
./build/georay --dem data/dem --origin 48.1375,11.5755 --extent 3000 --geoid-separation 47
```

Press `0` for the real terrain and `1`–`4` for the synthetic generators, on the
same scene — switching back and forth is the most direct way to see how much
the ground model actually changes an answer.

![georay over the Zugspitze massif](docs/screenshot-dem.png)

*Real SRTM data over the Zugspitze massif (`--origin 47.435,10.99
--geoid-separation 48`): two tiles across the 11°E seam, 2010 m of relief in a
6 km box. The flat-earth error on the picked pixel is 59 m there, against 4 m
over the Munich plain — which is the whole argument for terrain-aware
geolocation in one number.*

### Verified against published elevations

The format handling was checked against ground truth rather than only against
itself, because a misread `.hgt` produces terrain that still looks like
terrain:

| Point | georay | Published | Δ |
| --- | --- | --- | --- |
| Eibsee, lake surface | 973 m | 973 m | 0 |
| Mittenwald | 915 m | 912 m | +3 |
| Garmisch-Partenkirchen | 703 m | 708 m | −5 |
| Munich, Marienplatz | 523 m | 519 m | +4 |
| Munich airport (MUC) | 455 m | 448 m | +7 |
| Zugspitze summit | 2940 m | 2962 m | −22 |
| Ehrwald | 970 m | 996 m | −26 |

A flat lake surface is the strongest check — no slope to smooth — and it comes
out exact. The town values sit inside SRTM's ~6 m vertical accuracy, with the
positive bias at the Munich city centre and airport being what a DSM measuring
rooftops should do. The two large negatives are expected and not errors: a
30 m posting cannot resolve a knife-edge summit, and Ehrwald sits in a steep
valley where radar layover pulls the value down.

The sharpest check is the north–south gradient across `N48E011`, which falls
from 569 m near Munich to ~410 m toward the Danube. Read the rows south-first
and Munich comes out at 451 m — perfectly plausible, and 72 m wrong.

**georay itself never touches the network.** The script is the only thing that
does, and you run it deliberately; georay only ever reads local files. That
keeps the binary free of libcurl and TLS, keeps `--selftest` runnable offline,
and keeps bulk downloading something you asked for rather than something a
render loop does on your behalf.

Elevation comes from the AWS `elevation-tiles-prod` open-data bucket, which
serves SRTM in skadi layout — gzipped `.hgt`, no account, no API key. Tiles are
1° square, 3601×3601 at 1 arc-second (~30 m), about 7 MB compressed and 26 MB
on disk. SRTM covers 60°N to 56°S and has no tiles over open ocean.

### Why `--geoid-separation` is not optional

The two height systems in play are different systems. A GPS receiver reports
height above the **ellipsoid**; SRTM reports height above the **geoid**, which
is what "metres above sea level" means on a map. Near Munich they differ by
about 47 m.

Skip it and nothing looks broken — the terrain renders, the rays hit it, the
latitudes and longitudes are even correct. Only the heights are wrong, by the
separation; and since the aircraft's altitude is ellipsoidal while the ground
is orthometric, it ends up flying 47 m lower over the terrain than commanded,
which quietly changes every slant range, footprint and geolocated elevation.
The HUD shows `GEOID: NONE — HEIGHTS ARE ORTHOMETRIC` in red when it is unset.

A **single constant for the whole scene is the right answer here**, not a
shortcut: the geoid is a long-wavelength surface varying roughly a centimetre
per kilometre, so one value taken at the origin is good to a few centimetres
across a scene tens of kilometres wide — far below SRTM's own ~6 m vertical
accuracy. Rough values: ~47 m southern Germany, ~34 m Colorado, ~−30 m
southern India. `--geoid FILE` takes a full grid (converted from a
GeographicLib geoid file by `scripts/fetch_dem.py --geoid-pgm`) if you need
continental distances.

### What the DEM path guarantees

- **Heights go through the ellipsoid, not a subtraction.** Each post is
  converted geodetic → ENU through the same frame everything else uses, which
  carries the tangent-plane curvature term — flat ground 3 km from the origin
  sits 0.71 m *below* the tangent plane, and a flat subtraction would bake that
  into the terrain as a dome.
- **Voids are not zero.** SRTM's `-32768` void marker is excluded from the
  interpolation rather than read as a hole 32 km deep. Posts with no data at
  all are filled by diffusing their neighbours inwards — and the HUD then
  reports `DEM GAPS: n PCT FILLED` in red, because an interpolated post is an
  invented elevation, and that is often the right thing to do but never the
  right thing to hide.
- **Missing coverage is reported, not assumed to be sea.** A scene straddling
  a tile that is not on disk says so.
- **A failed load leaves the previous terrain alone** and falls back to
  synthetic with a message, rather than replacing the scene with a flat sheet
  of invented mean elevation that would look like a successful load of very
  boring ground.

## Controls

| Key | Action |
| --- | --- |
| Left click **in the camera view** | Geolocate that pixel |
| Left drag / right drag / wheel | Orbit / pan / zoom the 3D view |
| `Space`, `PgUp`/`PgDn` | Pause, simulation rate |
| `P`, `G`, arrows | Flight pattern, gimbal mode, manual gimbal aim |
| `T`, `B` | Track the picked point, follow the target |
| `1` `2` `3` `4` | Synthetic terrain: flat, hills, valley, mountains |
| `0` | Back to the real DEM terrain, if loaded |
| `[` `]`, `,` `.`, `-` `=`, `;` `'` | Camera zoom, height AGL, ground speed, orbit radius |
| `D`, `F`, `C`, `X`, `V`, `M` | Lens distortion, footprint, contours, flat-earth error, inset size, map |
| `R`, `F12`, `H`, `Q` | Reset, screenshot, help, quit |

## How the geolocation works

The pipeline lives in [`src/geo/geolocate.hpp`](src/geo/geolocate.hpp) and runs
entirely in double precision:

1. **Pixel → ray.** The pixel is un-distorted (Brown–Conrady, OpenCV's
   parameter order) and turned into a unit direction in the camera's optical
   frame — `+x` right, `+y` down, `+z` along the view axis.
2. **Optical → world.** A chain of rotations:
   `optical → gimbal → body → NED → ENU`, built from the gimbal's pan/tilt/roll
   and the aircraft's yaw/pitch/roll. The frames and their conventions are
   spelled out at the top of [`src/geo/pose.hpp`](src/geo/pose.hpp).
3. **Ray origin.** The GPS fix converted into the local ENU frame, plus the
   camera's lever arm from the GPS antenna rotated into world axes.
4. **Intersection.** The ray is marched against a sampled heightfield at half
   a grid cell per step until the first sign change of *(ray height − ground
   height)*, then bisected 50 times. Marching rather than solving, because a
   sampled DEM has no analytic form; bisecting afterwards, because a step
   coarse enough to cross kilometres would otherwise quantise the answer.
5. **Back to coordinates.** The hit point, ENU → ECEF → WGS84 geodetic.

The inverse direction — world point → pixel — is in the same file, and is what
the HUD's *reprojection* field exercises every frame.

### What it reports, and why

Besides latitude, longitude and elevation, the HUD shows the things that
determine whether a geolocation is worth anything:

- **Reprojection** — the picked point projected back through the camera, in
  pixels from where it was picked. A pipeline that cannot reproduce its own
  input has no business reporting coordinates.
- **Pixel GSD** — how wide that one pixel is on the ground, computed by tracing
  a neighbouring pixel against the tangent plane at the hit rather than by the
  textbook range/focal-length formula, which is wrong for oblique views.
- **D/tilt, D/heading, D/altitude** — how far the answer moves per degree of
  gimbal error, per degree of heading error, and per metre of altitude error.
  At a shallow depression angle a single degree is tens of metres, and no
  amount of GPS accuracy helps. This is the error budget, measured live.
- **Flat-earth comparison** (`X`) — the same pixel geolocated against a flat
  ground plane, with the horizontal distance between the two answers. On the
  mountain terrain this routinely exceeds 100 m.

## Verification

`--selftest` runs ~8600 numeric checks with no window and no GL, so it runs in
CI and on headless machines. It is also wired up as a CTest test
(`ctest --test-dir build`).

Every error in a tool like this is silent — a transposed matrix or a sign slip
produces a confident latitude that is simply somewhere else — so the suite
pins the conventions down with cases whose answers are known independently of
the code:

- Geodetic ↔ ECEF against points on the ellipsoid with known coordinates, and
  round trips to a micrometre over the whole globe.
- ENU axis directions: `+latitude` is north and not east, `+longitude` is east,
  `+altitude` is up by exactly that much.
- The rotation chain: nadir is nadir, yaw 90° looks east, every composition is
  orthonormal with determinant `+1`, and `aimGimbalAlong` inverts the chain.
- The whole pipeline against the closed-form plane intersection over flat
  ground, to a micrometre, for thousands of random poses.
- Round trips over all four terrain types, with and without lens distortion:
  geolocate a pixel, project the answer back, land within a tenth of a pixel.
- Failure modes: a ray above the horizon and a ray that leaves the DEM tile are
  reported as *different* things, because only one of them is fixed by loading
  more terrain.
- The rendered simulated-camera view against the geolocation maths, so what you
  click is what is computed.
- The DEM path, against synthetic `.hgt` files whose heights encode their own
  post indices — pinning the big-endian byte order, the north-first row order
  (flip it and the terrain still looks like terrain, just mirrored), void
  handling, tile selection across hemispheres, and continuity across a tile
  seam. The resampling is checked against the closed-form curvature drop
  `d²/2R` and against gradients computed from metres-per-degree, so none of it
  is verified by re-running the code under test.

The DEM tests build their own tiles rather than downloading one: a test that
needs 7 MB fetched from AWS is a test that gets skipped.

## Known limitations

These are deliberate, and each is where a production pipeline would differ.

- **The ground model is a DTM, not a DSM.** Rays intersect terrain, not the
  structures standing on it. Click the top of one of the towers and you get the
  ground *behind* it. That is left visible on purpose: it is the single largest
  error source in real oblique geolocation.
- **The rendered view cannot show lens distortion.** A projection matrix is
  linear. With distortion on (`D`), the maths models a distorted camera while
  the picture stays an ideal pinhole render, so the two diverge towards the
  frame edges — the HUD says so while it is active. Fixing it properly means
  rendering to a texture and warping it.
- **SRTM is a 2000-vintage radar DSM at ~30 m posting**, with roughly 6 m
  vertical accuracy, and over forest and cities it measures the canopy and
  rooftops rather than the ground. A photogrammetry pipeline that cared would
  use a modern lidar DTM where one exists.
- **The basemap is a map, not imagery.** At zoom 16 it is 1.45 m per pixel,
  while the camera's own GSD at 400 m is about 44 cm — so flying lower
  magnifies map pixels rather than resolving more ground. The HUD shows both
  numbers so the limit is visible. Pointing `--url` at an aerial imagery
  service you are licensed for fixes this.
- **Only SRTM `.hgt` is read.** Copernicus GLO-30 and most national products
  ship as GeoTIFF, which needs a TIFF reader this deliberately does not have.
  The conversion would belong in `scripts/fetch_dem.py`, where the messy
  formats already live, writing the same simple arrays the C++ side reads.
- **No atmospheric refraction**, which bends a long slant-range ray by a metre
  or so near the ground.

## Layout

```
src/geo/      the geolocation maths, and the DEM/geoid/map readers - no GL
              anywhere in it, and a separate CMake target so it stays that way
src/gl/       thin OpenGL wrappers: shaders, buffers, an embedded bitmap font
src/render/   terrain mesh, 3D overlays, the orbit camera
src/          the application: window, input, HUD, flight simulation, selftest
shaders/      GLSL
scenes/       ready-made scenes: berlin, munich, zugspitze
scripts/      fetch_dem.py and fetch_map.py - the only things here that
              touch the network
```
