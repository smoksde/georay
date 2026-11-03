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
./build/georay
```

Other modes:

| Command | What it does |
| --- | --- |
| `./build/georay --selftest` | Runs the numeric verification suite and exits. No window, no GL — works headless. |
| `./build/georay --screenshot out.bmp` | Renders a few frames, saves a BMP, exits. |
| `./build/georay --size 1920x1080` | Window size. |

## Controls

| Key | Action |
| --- | --- |
| Left click **in the camera view** | Geolocate that pixel |
| Left drag / right drag / wheel | Orbit / pan / zoom the 3D view |
| `Space`, `PgUp`/`PgDn` | Pause, simulation rate |
| `P`, `G`, arrows | Flight pattern, gimbal mode, manual gimbal aim |
| `T`, `B` | Track the picked point, follow the target |
| `1` `2` `3` `4` | Terrain: flat, hills, valley, mountains |
| `[` `]`, `,` `.`, `-` `=`, `;` `'` | Camera zoom, height AGL, ground speed, orbit radius |
| `D`, `F`, `C`, `X`, `V` | Lens distortion, footprint, contours, flat-earth error, inset size |
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

## Known limitations

These are deliberate, and each is where a production pipeline would differ.

- **Synthetic terrain, not a DEM.** The ground is a generated heightfield, but
  it is stored and sampled exactly as a DEM tile would be, so loading real data
  means filling the same array — see the header comment in
  [`src/geo/terrain.hpp`](src/geo/terrain.hpp) for the conversion that applies.
- **The ground model is a DTM, not a DSM.** Rays intersect terrain, not the
  structures standing on it. Click the top of one of the towers and you get the
  ground *behind* it. That is left visible on purpose: it is the single largest
  error source in real oblique geolocation.
- **The rendered view cannot show lens distortion.** A projection matrix is
  linear. With distortion on (`D`), the maths models a distorted camera while
  the picture stays an ideal pinhole render, so the two diverge towards the
  frame edges — the HUD says so while it is active. Fixing it properly means
  rendering to a texture and warping it.
- **Altitudes are ellipsoidal throughout.** Real DEMs are usually orthometric
  (above the geoid); a geoid model would be added in `sampleHeight`.
- **No atmospheric refraction**, which bends a long slant-range ray by a metre
  or so near the ground.

## Layout

```
src/geo/      the geolocation maths - no GL anywhere in it, and a separate
              CMake target so it stays that way
src/gl/       thin OpenGL wrappers: shaders, buffers, an embedded bitmap font
src/render/   terrain mesh, 3D overlays, the orbit camera
src/          the application: window, input, HUD, flight simulation, selftest
shaders/      GLSL
```
