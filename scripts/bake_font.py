#!/usr/bin/env python3
"""Rasterise a TrueType font into georay's bitmap atlas format.

The same division of labour as the map and geoid converters: the messy,
library-shaped work happens here, once, and the C++ side reads one trivially
parseable binary. That is what lets georay render a real programming font
without linking FreeType, stb_truetype, or anything else.

Baking rather than rasterising at run time also makes the text sharper, not
just the build simpler. The atlas is rendered at exactly the pixel sizes the
HUD draws at and sampled 1:1 with nearest filtering, so glyphs land on whole
pixels instead of being resampled - which is the difference between crisp text
and grey mush at 13 px.

    scripts/bake_font.py assets/fonts/JetBrainsMono-Regular.ttf

Re-run it after changing the sizes below; the output is committed, so nobody
needs this script to build or run georay.
"""

import argparse
import os
import struct
import sys

MAGIC = b"GEORAYF1"
# ASCII 32..126: everything a HUD of numbers and capitals needs, plus
# lowercase, which a real font can finally render.
FIRST_CHAR, LAST_CHAR = 32, 126
# The degree sign is the one non-ASCII glyph worth having - "48.137 deg N" is
# markedly worse than "48.137 N" with a proper degree mark. It is baked into
# the slot the old 3x5 font used for it, so existing format strings keep
# working.
DEGREE_SLOT = ord("^")
DEGREE_CHAR = "°"


def bake(font_path, pixel_sizes, out_path):
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        raise SystemExit("this script needs Pillow: pip install pillow")

    faces = []
    for size in pixel_sizes:
        font = ImageFont.truetype(font_path, size)
        ascent, descent = font.getmetrics()

        # Monospace: every glyph advances the same, so the atlas is a uniform
        # grid and the renderer needs no per-glyph metrics at all. Measured
        # rather than assumed, so a proportional font fails visibly here
        # instead of rendering as overlapping mush.
        advances = {font.getlength(chr(c)) for c in range(FIRST_CHAR, LAST_CHAR + 1)}
        if len(advances) != 1:
            raise SystemExit(
                "%s is not monospaced at %d px (%d different advances); georay's "
                "atlas is a uniform grid" % (font_path, size, len(advances)))
        advance = int(round(advances.pop()))

        cell_w = advance + 2          # a column of padding each side
        cell_h = ascent + descent + 2
        count = LAST_CHAR - FIRST_CHAR + 1
        columns = 16
        rows = (count + columns - 1) // columns

        atlas = Image.new("L", (columns * cell_w, rows * cell_h), 0)
        draw = ImageDraw.Draw(atlas)
        for index in range(count):
            code = FIRST_CHAR + index
            glyph = DEGREE_CHAR if code == DEGREE_SLOT else chr(code)
            x = (index % columns) * cell_w + 1
            y = (index // columns) * cell_h + 1
            # anchor="ls": x is the pen position, y the baseline. Drawing from
            # the baseline is what keeps every glyph on the same line rather
            # than aligned by its own bounding box.
            draw.text((x, y + ascent), glyph, font=font, fill=255, anchor="ls")

        faces.append({
            "size": size, "cell_w": cell_w, "cell_h": cell_h,
            "advance": advance, "ascent": ascent, "columns": columns, "rows": rows,
            "atlas": atlas,
        })
        print("  %2d px: advance %d, cell %dx%d, atlas %dx%d"
              % (size, advance, cell_w, cell_h, atlas.width, atlas.height))

    # georay font atlas, little-endian:
    #   magic[8], int32 face_count, int32 first_char, int32 char_count
    #   per face: int32 pixel_size, cell_w, cell_h, advance, ascent,
    #             columns, atlas_w, atlas_h, then atlas_w*atlas_h grey bytes
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "wb") as handle:
        handle.write(MAGIC)
        handle.write(struct.pack("<iii", len(faces), FIRST_CHAR,
                                 LAST_CHAR - FIRST_CHAR + 1))
        for face in faces:
            handle.write(struct.pack(
                "<iiiiiiii", face["size"], face["cell_w"], face["cell_h"],
                face["advance"], face["ascent"], face["columns"],
                face["atlas"].width, face["atlas"].height))
            handle.write(face["atlas"].tobytes())

    print("wrote %s (%.1f KB)" % (out_path, os.path.getsize(out_path) / 1024.0))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("font", nargs="?", default="assets/fonts/JetBrainsMono-Regular.ttf")
    parser.add_argument("--out", default="assets/fonts/georay-font.bin")
    # Three faces: small for footnotes and the map credit, normal for the
    # readouts, large for pane titles and status text.
    parser.add_argument("--sizes", default="11,15,22",
                        help="comma-separated pixel sizes to bake")
    args = parser.parse_args()

    sizes = [int(s) for s in args.sizes.split(",")]
    print("baking %s at %s px" % (args.font, args.sizes))
    bake(args.font, sizes, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
