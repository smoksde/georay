#pragma once

// Screen-space text, from a bitmap atlas baked out of a TrueType font by
// scripts/bake_font.py.
//
// Baked rather than rasterised at run time: keeps FreeType/stb_truetype out
// of the build, and the atlas is rendered at exactly the HUD's pixel sizes
// and sampled 1:1, so glyphs land on whole pixels - crisp text instead of
// grey mush at 13 px.
//
// Three faces: a readout size, a footnote size, a heading size - anything
// between would just be a blurry version of one of them.
//
// A 3x5 font is compiled in as a fallback, not vestigial: if the atlas is
// missing or unreadable, tiny capitals beat no text at all, and the failure
// announces itself instead of looking like a rendering bug.

#include <string>
#include <vector>

#include "gl/gl.hpp"

namespace georay {

class TextRenderer {
public:
    // Which baked size to draw at. Named rather than numbered so call sites
    // say what they mean.
    enum Face { kSmall = 0, kNormal = 1, kLarge = 2, kFaceCount = 3 };

    TextRenderer() = default;
    ~TextRenderer();

    TextRenderer(const TextRenderer&) = delete;
    TextRenderer& operator=(const TextRenderer&) = delete;

    // Builds the glyph texture, from `font_path` if it loads and from the
    // built-in 3x5 font otherwise. Needs a current GL context.
    void initialise(const std::string& font_path);

    // Queue a string. `position` is the top-left of the first line, in window
    // pixels with y downwards. Newlines start a new line at the same x.
    void add(const std::string& text, glm::vec2 position, int face, const glm::vec4& colour);

    // Metrics, from the loaded atlas rather than a formula a caller might
    // compute differently from what actually gets drawn.
    float advance(int face) const;
    float lineHeight(int face) const;
    float ascent(int face) const;
    float textWidth(const std::string& text, int face) const;

    // Draws everything queued since the last flush. The shader must already
    // be bound with its projection set.
    void flush();

private:
    struct FaceInfo {
        int pixel_size = 0;
        int cell_w = 0, cell_h = 0;
        int advance = 0, ascent = 0;
        // Stored, not derived from the cell: a baked face pads its cell by a
        // pixel each side, the fallback's cell already includes its
        // leading, and deriving it collapsed the fallback's lines together.
        int line_height = 0;
        int columns = 16;
        // Where this face's grid sits in the shared texture, so mixed sizes
        // still draw with a single bind.
        int atlas_x = 0, atlas_y = 0;
        int atlas_w = 0, atlas_h = 0;
        // Integer upscale, used only by the built-in fallback font.
        int scale = 1;
    };

    struct Vertex {
        glm::vec2 position;
        glm::vec2 uv;
        glm::vec4 colour;
    };

    // Returns false when the file is missing or malformed, leaving the
    // caller to fall back.
    bool loadBakedAtlas(const std::string& path, std::vector<unsigned char>& pixels,
                        int& width, int& height);
    void buildFallbackAtlas(std::vector<unsigned char>& pixels, int& width, int& height);
    void uploadAtlas(const std::vector<unsigned char>& pixels, int width, int height);
    const FaceInfo& face(int index) const;

    FaceInfo faces_[kFaceCount];
    int first_char_ = 32, char_count_ = 95;
    bool baked_ = false;

    std::vector<Vertex> vertices_;
    GLuint vao_ = 0, vbo_ = 0, texture_ = 0;
    int atlas_w_ = 1, atlas_h_ = 1;
    size_t capacity_ = 0;
};

}  // namespace georay
