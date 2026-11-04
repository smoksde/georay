# Vendored third-party sources

## stb_image.h (v2.30)

Public domain (or MIT, dual-licensed — see the licence block at the end of the
file). From https://nothings.org/stb.

georay needs it because the map is now stitched from tiles at run time rather
than pre-assembled in Python, and the tiles on disk are PNG and JPEG. Decoding
them is the one thing the C++ side could previously avoid; live fetching and
zoom-matched detail make that no longer possible.

It is a single header with no build system of its own, compiled into exactly
one translation unit (`src/map/tilecache.cpp`) via `STB_IMAGE_IMPLEMENTATION`.
Only PNG and JPEG are enabled; every other format stb supports is switched off,
so a malformed tile has a smaller surface to go wrong in.
