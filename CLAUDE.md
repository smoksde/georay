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

## Process

- Verify every change: clean rebuild + `./build/georay --selftest` + a runtime
  smoke test (`DISPLAY=:1 timeout 20 ./build/georay`, termination by timeout is
  expected). For anything that changes what is drawn, also
  `DISPLAY=:1 ./build/georay --screenshot /tmp/shot.bmp` and look at it.
- Prove new tests non-vacuous: break the thing they cover, confirm the test
  fails, then restore it, before committing.
- When a test fails, work out whether the code or the expectation is wrong
  before changing either. Several of the existing expectations are
  counterintuitive and correct - the rolled-nadir case in `testPose` is the
  clearest example.
- Commit locally when a fix is verified; don't push unless asked.
