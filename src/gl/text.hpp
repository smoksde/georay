#pragma once

// Screen-space text, from a 3x5 bitmap font compiled into the binary.
//
// A HUD is not optional for this tool - a latitude you cannot read is not a
// result - but a font file is a dependency, a loader, and a thing to ship
// alongside the executable. So the font lives in text.cpp as 64 glyphs of
// string art, which costs under a kilobyte, cannot go missing, and renders
// identically everywhere. Lowercase folds to uppercase; a HUD reads fine in
// capitals and it halves the glyph table.

#include <string>
#include <vector>

#include "gl/gl.hpp"

namespace georay {

class TextRenderer {
public:
    TextRenderer() = default;
    ~TextRenderer();

    TextRenderer(const TextRenderer&) = delete;
    TextRenderer& operator=(const TextRenderer&) = delete;

    // Builds the glyph atlas texture. Needs a current GL context.
    void initialise();

    // Queue a string. `scale` is an integer pixel multiplier: at scale 2 a
    // glyph is 6x10 pixels. Position is the top-left corner, y down, in
    // window pixels.
    void add(const std::string& text, glm::vec2 position, float scale, const glm::vec4& colour);

    // Metrics, so callers can lay out columns and panels without guessing.
    static float glyphAdvance(float scale) { return 4.0f * scale; }
    static float lineHeight(float scale) { return 7.0f * scale; }
    static float textWidth(const std::string& text, float scale);

    // Draws everything queued since the last flush. The shader must already
    // be bound with its projection set; `uploadAndDraw` only touches the
    // vertex buffer and the atlas texture unit.
    void flush();

    GLuint atlasTexture() const { return texture_; }
    bool ready() const { return texture_ != 0; }

private:
    struct Vertex {
        glm::vec2 position;
        glm::vec2 uv;
        glm::vec4 colour;
    };

    std::vector<Vertex> vertices_;
    GLuint vao_ = 0, vbo_ = 0, texture_ = 0;
    size_t capacity_ = 0;
};

}  // namespace georay
