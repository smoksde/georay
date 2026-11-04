#include "gl/text.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

namespace georay {
namespace {

// --- the built-in fallback font ------------------------------------------
//
// Reached only when the baked atlas fails to load. Three pixels wide and
// barely legible, but better than no text; the startup message says which
// font is in use, so this never reads as a rendering bug.
constexpr int kGlyphW = 3, kGlyphH = 5;
constexpr int kFallbackCellW = 4, kFallbackCellH = 7;
constexpr int kFallbackCols = 16, kFallbackRows = 4;
constexpr int kFallbackFirst = 32, kFallbackLast = 95;

const char* const kFont[kFallbackLast - kFallbackFirst + 1][kGlyphH] = {
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

const TextRenderer::FaceInfo& TextRenderer::face(int index) const {
    // Clamping rather than asserting: a bad face index should draw text at
    // the wrong size, not take the window down.
    return faces_[std::min(std::max(index, 0), int(kFaceCount) - 1)];
}

float TextRenderer::advance(int index) const {
    const FaceInfo& f = face(index);
    return float(f.advance * f.scale);
}

float TextRenderer::lineHeight(int index) const {
    const FaceInfo& f = face(index);
    return float(f.line_height * f.scale);
}

float TextRenderer::ascent(int index) const {
    const FaceInfo& f = face(index);
    return float(f.ascent * f.scale);
}

float TextRenderer::textWidth(const std::string& text, int index) const {
    if (text.empty()) return 0.0f;
    // Longest line, so a multi-line block measures as the box it occupies.
    size_t longest = 0, current = 0;
    for (char c : text) {
        if (c == '\n') {
            longest = std::max(longest, current);
            current = 0;
        } else {
            ++current;
        }
    }
    longest = std::max(longest, current);
    return float(longest) * advance(index);
}

bool TextRenderer::loadBakedAtlas(const std::string& path, std::vector<unsigned char>& pixels,
                                  int& width, int& height) {
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;

    char magic[8] = {};
    file.read(magic, sizeof(magic));
    if (!file || std::memcmp(magic, "GEORAYF1", 8) != 0) return false;

    int32_t face_count = 0, first_char = 0, char_count = 0;
    file.read(reinterpret_cast<char*>(&face_count), 4);
    file.read(reinterpret_cast<char*>(&first_char), 4);
    file.read(reinterpret_cast<char*>(&char_count), 4);
    if (!file || face_count < 1 || face_count > kFaceCount || char_count < 1 ||
        char_count > 512) {
        return false;
    }

    struct Loaded {
        FaceInfo info;
        std::vector<unsigned char> pixels;
    };
    std::vector<Loaded> loaded;

    for (int i = 0; i < face_count; ++i) {
        int32_t header[8] = {};
        file.read(reinterpret_cast<char*>(header), sizeof(header));
        if (!file) return false;

        Loaded entry;
        entry.info.pixel_size = header[0];
        entry.info.cell_w = header[1];
        entry.info.cell_h = header[2];
        entry.info.advance = header[3];
        entry.info.ascent = header[4];
        entry.info.columns = header[5];
        entry.info.atlas_w = header[6];
        entry.info.atlas_h = header[7];
        // The baked cell carries a pixel of padding on each side so that
        // neighbouring cells cannot bleed into one another; that padding is
        // not line spacing.
        entry.info.line_height = entry.info.cell_h - 2;

        // Bounds before allocating: a corrupt header must not be able to ask
        // for an arbitrary amount of memory.
        if (entry.info.cell_w < 1 || entry.info.cell_h < 1 || entry.info.columns < 1 ||
            entry.info.atlas_w < 1 || entry.info.atlas_h < 1 || entry.info.atlas_w > 8192 ||
            entry.info.atlas_h > 8192) {
            return false;
        }

        const size_t bytes = size_t(entry.info.atlas_w) * size_t(entry.info.atlas_h);
        entry.pixels.resize(bytes);
        file.read(reinterpret_cast<char*>(entry.pixels.data()), std::streamsize(bytes));
        if (file.gcount() != std::streamsize(bytes)) return false;
        loaded.push_back(std::move(entry));
    }

    // Stack the faces into one texture, so drawing mixed sizes still takes a
    // single bind and a single draw call.
    width = 0;
    height = 0;
    for (const Loaded& entry : loaded) {
        width = std::max(width, entry.info.atlas_w);
        height += entry.info.atlas_h;
    }
    pixels.assign(size_t(width) * size_t(height), 0);

    int y = 0;
    for (size_t i = 0; i < loaded.size(); ++i) {
        const Loaded& entry = loaded[i];
        for (int row = 0; row < entry.info.atlas_h; ++row) {
            std::copy(entry.pixels.begin() + long(row) * entry.info.atlas_w,
                      entry.pixels.begin() + long(row + 1) * entry.info.atlas_w,
                      pixels.begin() + long(y + row) * width);
        }
        faces_[i] = entry.info;
        faces_[i].atlas_x = 0;
        faces_[i].atlas_y = y;
        faces_[i].scale = 1;
        y += entry.info.atlas_h;
    }
    // Fewer faces baked than the HUD asks for: repeat the last one rather
    // than leaving an unusable zero-sized face behind.
    for (int i = int(loaded.size()); i < kFaceCount; ++i) faces_[i] = faces_[loaded.size() - 1];

    first_char_ = first_char;
    char_count_ = char_count;
    return true;
}

void TextRenderer::buildFallbackAtlas(std::vector<unsigned char>& pixels, int& width,
                                      int& height) {
    width = kFallbackCols * kFallbackCellW;
    height = kFallbackRows * kFallbackCellH;
    pixels.assign(size_t(width) * size_t(height), 0);

    for (int code = kFallbackFirst; code <= kFallbackLast; ++code) {
        const int index = code - kFallbackFirst;
        const int cell_x = (index % kFallbackCols) * kFallbackCellW;
        const int cell_y = (index / kFallbackCols) * kFallbackCellH;
        for (int row = 0; row < kGlyphH; ++row) {
            for (int col = 0; col < kGlyphW; ++col) {
                if (kFont[index][row][col] != '#') continue;
                pixels[size_t(cell_y + row) * size_t(width) + size_t(cell_x + col)] = 255;
            }
        }
    }

    first_char_ = kFallbackFirst;
    char_count_ = kFallbackLast - kFallbackFirst + 1;
    // One atlas, three faces, differing only by integer upscale.
    const int scales[kFaceCount] = {1, 2, 3};
    for (int i = 0; i < kFaceCount; ++i) {
        FaceInfo& f = faces_[i];
        f.pixel_size = kGlyphH * scales[i];
        f.cell_w = kFallbackCellW;
        f.cell_h = kFallbackCellH;
        f.advance = kFallbackCellW;
        f.ascent = kGlyphH;
        // The fallback's cell is the glyph plus its leading already, so the
        // whole cell is the line height.
        f.line_height = kFallbackCellH;
        f.columns = kFallbackCols;
        f.atlas_x = 0;
        f.atlas_y = 0;
        f.atlas_w = width;
        f.atlas_h = height;
        f.scale = scales[i];
    }
}

void TextRenderer::uploadAtlas(const std::vector<unsigned char>& pixels, int width,
                               int height) {
    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    // Single-channel coverage, used as alpha by the shader. GL_RED rather
    // than the long-deprecated GL_ALPHA, since this is a core profile.
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED, GL_UNSIGNED_BYTE,
                 pixels.data());
    // Nearest filtering, and that is not a compromise: the atlas is baked at
    // the exact sizes it is drawn at, so every texel maps to one pixel.
    // Linear filtering would resample a 1:1 image and soften it for nothing.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    atlas_w_ = width;
    atlas_h_ = height;
}

void TextRenderer::initialise(const std::string& font_path) {
    if (texture_ != 0) return;

    std::vector<unsigned char> pixels;
    int width = 0, height = 0;

    if (loadBakedAtlas(font_path, pixels, width, height)) {
        baked_ = true;
    } else {
        // Say so. A silent fallback to 3x5 capitals looks like a rendering
        // bug rather than a missing file.
        std::cerr << "georay: cannot read font atlas " << font_path
                  << ", falling back to the built-in 3x5 font\n"
                     "georay: regenerate it with scripts/bake_font.py\n";
        buildFallbackAtlas(pixels, width, height);
    }

    uploadAtlas(pixels, width, height);

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

void TextRenderer::add(const std::string& text, glm::vec2 position, int index,
                       const glm::vec4& colour) {
    if (texture_ == 0) return;
    const FaceInfo& f = face(index);

    const float cell_w = float(f.cell_w * f.scale);
    const float cell_h = float(f.cell_h * f.scale);
    const float step = float(f.advance * f.scale);
    const float line_step = lineHeight(index);

    float x = position.x;
    float line_top = position.y;

    for (char raw : text) {
        if (raw == '\n') {
            x = position.x;
            line_top += line_step;
            continue;
        }

        int code = static_cast<unsigned char>(raw);
        // The fallback font has no lowercase; the baked one does. Folding
        // only where it is needed keeps real text readable.
        if (!baked_) code = std::toupper(code);
        if (code < first_char_ || code >= first_char_ + char_count_) {
            x += step;  // unknown character: leave its space, draw nothing
            continue;
        }

        if (code != ' ') {
            const int glyph = code - first_char_;
            const int col = glyph % f.columns;
            const int row = glyph / f.columns;
            const float u0 = float(f.atlas_x + col * f.cell_w) / float(atlas_w_);
            const float v0 = float(f.atlas_y + row * f.cell_h) / float(atlas_h_);
            const float u1 = u0 + float(f.cell_w) / float(atlas_w_);
            const float v1 = v0 + float(f.cell_h) / float(atlas_h_);

            // The cell carries a pixel of padding, and the glyph was drawn
            // with its pen one pixel in - so the quad sits one pixel left of
            // and above the pen position.
            const float qx = x - float(f.scale);
            const float qy = line_top - float(f.scale);

            const Vertex tl{{qx, qy}, {u0, v0}, colour};
            const Vertex tr{{qx + cell_w, qy}, {u1, v0}, colour};
            const Vertex br{{qx + cell_w, qy + cell_h}, {u1, v1}, colour};
            const Vertex bl{{qx, qy + cell_h}, {u0, v1}, colour};
            vertices_.insert(vertices_.end(), {tl, tr, br, tl, br, bl});
        }
        x += step;
    }
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
