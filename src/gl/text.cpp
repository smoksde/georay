#include "gl/text.hpp"

#include <cctype>
#include <vector>

namespace georay {
namespace {

// Glyph geometry. Cells are one pixel wider and two taller than the glyph so
// that linear-filtered sampling (and any half-pixel rounding) cannot pull in
// a neighbour's ink.
constexpr int kGlyphW = 3, kGlyphH = 5;
constexpr int kCellW = 4, kCellH = 7;
constexpr int kCols = 16, kRows = 4;
constexpr int kAtlasW = kCols * kCellW;  // 64
constexpr int kAtlasH = kRows * kCellH;  // 28
constexpr int kFirstChar = 32, kLastChar = 95;

// ASCII 32..95 in order: space, punctuation, digits, more punctuation, A-Z,
// brackets, '^' (used as a degree sign - see below), underscore.
//
// Written as string art rather than as hex bytes because the whole point of
// a hand-authored font is that a wrong glyph is obvious on sight, and
// `0x7, 0x5, 0x7` is not.
const char* const kFont[kLastChar - kFirstChar + 1][kGlyphH] = {
    {"...", "...", "...", "...", "..."},  // space
    {".#.", ".#.", ".#.", "...", ".#."},  // !
    {"#.#", "#.#", "...", "...", "..."},  // "
    {"#.#", "###", "#.#", "###", "#.#"},  // #
    {".#.", "###", ".#.", "###", ".#."},  // $
    {"#.#", "..#", ".#.", "#..", "#.#"},  // %
    {"##.", "##.", "###", "#.#", "###"},  // &
    {".#.", ".#.", "...", "...", "..."},  // '
    {"..#", ".#.", ".#.", ".#.", "..#"},  // (
    {"#..", ".#.", ".#.", ".#.", "#.."},  // )
    {"#.#", ".#.", "#.#", "...", "..."},  // *
    {"...", ".#.", "###", ".#.", "..."},  // +
    {"...", "...", "...", ".#.", "#.."},  // ,
    {"...", "...", "###", "...", "..."},  // -
    {"...", "...", "...", "...", ".#."},  // .
    {"..#", "..#", ".#.", "#..", "#.."},  // /
    {"###", "#.#", "#.#", "#.#", "###"},  // 0
    {".#.", "##.", ".#.", ".#.", "###"},  // 1
    {"###", "..#", "###", "#..", "###"},  // 2
    {"###", "..#", "###", "..#", "###"},  // 3
    {"#.#", "#.#", "###", "..#", "..#"},  // 4
    {"###", "#..", "###", "..#", "###"},  // 5
    {"###", "#..", "###", "#.#", "###"},  // 6
    {"###", "..#", "..#", "..#", "..#"},  // 7
    {"###", "#.#", "###", "#.#", "###"},  // 8
    {"###", "#.#", "###", "..#", "###"},  // 9
    {"...", ".#.", "...", ".#.", "..."},  // :
    {"...", ".#.", "...", ".#.", "#.."},  // ;
    {"..#", ".#.", "#..", ".#.", "..#"},  // <
    {"...", "###", "...", "###", "..."},  // =
    {"#..", ".#.", "..#", ".#.", "#.."},  // >
    {"###", "..#", ".#.", "...", ".#."},  // ?
    {"###", "#.#", "###", "#..", "###"},  // @
    {"###", "#.#", "###", "#.#", "#.#"},  // A
    {"##.", "#.#", "##.", "#.#", "##."},  // B
    {"###", "#..", "#..", "#..", "###"},  // C
    {"##.", "#.#", "#.#", "#.#", "##."},  // D
    {"###", "#..", "##.", "#..", "###"},  // E
    {"###", "#..", "##.", "#..", "#.."},  // F
    {"###", "#..", "#.#", "#.#", "###"},  // G
    {"#.#", "#.#", "###", "#.#", "#.#"},  // H
    {"###", ".#.", ".#.", ".#.", "###"},  // I
    {"..#", "..#", "..#", "#.#", "###"},  // J
    {"#.#", "#.#", "##.", "#.#", "#.#"},  // K
    {"#..", "#..", "#..", "#..", "###"},  // L
    {"#.#", "###", "###", "#.#", "#.#"},  // M
    // A diagonal stroke does not survive three pixels of width - it reads as
    // a percent sign in running text, which is how "LON" became "LO%". A
    // plain left-heavy form is less elegant and far more legible; the closed
    // bottom row is what keeps it distinct from D.
    {"##.", "#.#", "#.#", "#.#", "#.#"},  // N
    {"###", "#.#", "#.#", "#.#", "###"},  // O
    {"###", "#.#", "###", "#..", "#.."},  // P
    {"###", "#.#", "###", "..#", "..#"},  // Q
    {"###", "#.#", "##.", "#.#", "#.#"},  // R
    {"###", "#..", "###", "..#", "###"},  // S
    {"###", ".#.", ".#.", ".#.", ".#."},  // T
    {"#.#", "#.#", "#.#", "#.#", "###"},  // U
    {"#.#", "#.#", "#.#", "#.#", ".#."},  // V
    {"#.#", "#.#", "###", "###", "#.#"},  // W
    {"#.#", "#.#", ".#.", "#.#", "#.#"},  // X
    {"#.#", "#.#", ".#.", ".#.", ".#."},  // Y
    {"###", "..#", ".#.", "#..", "###"},  // Z
    {"###", "#..", "#..", "#..", "###"},  // [
    {"#..", "#..", ".#.", "..#", "..#"},  // backslash
    {"###", "..#", "..#", "..#", "###"},  // ]
    // '^' is pressed into service as a degree sign: latitudes and longitudes
    // are the main thing on screen, and "48.137^ N" reads far better than
    // "48.137 DEG N". A caret has no other use in a numeric HUD.
    {".#.", "#.#", ".#.", "...", "..."},  // ^ (degree)
    {"...", "...", "...", "...", "###"},  // _
};

}  // namespace

TextRenderer::~TextRenderer() {
    if (texture_) glDeleteTextures(1, &texture_);
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
}

void TextRenderer::initialise() {
    if (texture_ != 0) return;

    std::vector<unsigned char> pixels(size_t(kAtlasW) * kAtlasH, 0);
    for (int code = kFirstChar; code <= kLastChar; ++code) {
        const int index = code - kFirstChar;
        const int cell_x = (index % kCols) * kCellW;
        const int cell_y = (index / kCols) * kCellH;
        for (int row = 0; row < kGlyphH; ++row) {
            for (int col = 0; col < kGlyphW; ++col) {
                if (kFont[index][row][col] != '#') continue;
                const int x = cell_x + col;
                const int y = cell_y + row;
                pixels[size_t(y) * kAtlasW + x] = 255;
            }
        }
    }

    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    // Single-channel coverage: the shader uses it as alpha. GL_RED rather
    // than the long-deprecated GL_ALPHA, since this is a core profile.
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, kAtlasW, kAtlasH, 0, GL_RED, GL_UNSIGNED_BYTE,
                 pixels.data());
    // Nearest filtering, because the glyphs are 3x5 pixels drawn at integer
    // multiples: anything smoother turns crisp text into grey mush.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, position));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, uv));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, colour));
    glBindVertexArray(0);
}

float TextRenderer::textWidth(const std::string& text, float scale) {
    // One glyph's worth of advance per character, minus the trailing gap, so
    // a right-aligned string ends where its ink ends.
    if (text.empty()) return 0.0f;
    return float(text.size()) * glyphAdvance(scale) - scale;
}

void TextRenderer::add(const std::string& text, glm::vec2 position, float scale,
                       const glm::vec4& colour) {
    const float gw = kGlyphW * scale;
    const float gh = kGlyphH * scale;
    float x = position.x;
    const float y = position.y;

    for (char raw : text) {
        if (raw == '\n') {
            // Newlines are handled here rather than by the caller so that a
            // multi-line block can be one add() call.
            x = position.x;
            position.y += lineHeight(scale);
            continue;
        }
        const int code = std::toupper(static_cast<unsigned char>(raw));
        if (code < kFirstChar || code > kLastChar) {
            x += glyphAdvance(scale);
            continue;
        }
        const int index = code - kFirstChar;
        // Space has no ink; skipping it is the common case and saves six
        // vertices per blank column in a HUD that is mostly padding.
        if (code != 32) {
            const float u0 = float((index % kCols) * kCellW) / float(kAtlasW);
            const float v0 = float((index / kCols) * kCellH) / float(kAtlasH);
            const float u1 = u0 + float(kGlyphW) / float(kAtlasW);
            const float v1 = v0 + float(kGlyphH) / float(kAtlasH);

            const float y_top = position.y;
            const Vertex tl{{x, y_top}, {u0, v0}, colour};
            const Vertex tr{{x + gw, y_top}, {u1, v0}, colour};
            const Vertex br{{x + gw, y_top + gh}, {u1, v1}, colour};
            const Vertex bl{{x, y_top + gh}, {u0, v1}, colour};
            vertices_.insert(vertices_.end(), {tl, tr, br, tl, br, bl});
        }
        x += glyphAdvance(scale);
    }
    (void)y;
}

void TextRenderer::flush() {
    if (vertices_.empty() || texture_ == 0) {
        vertices_.clear();
        return;
    }

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture_);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    if (vertices_.size() > capacity_) {
        capacity_ = vertices_.size() * 2;
        glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(capacity_ * sizeof(Vertex)), nullptr,
                     GL_DYNAMIC_DRAW);
    }
    glBufferSubData(GL_ARRAY_BUFFER, 0, GLsizeiptr(vertices_.size() * sizeof(Vertex)),
                    vertices_.data());
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(vertices_.size()));
    glBindVertexArray(0);

    vertices_.clear();
}

}  // namespace georay
