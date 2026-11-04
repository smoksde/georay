#pragma once

// Two GL buffer wrappers, which is all this tool draws with:
//
//   SurfaceMesh    - an indexed triangle mesh uploaded once (the terrain).
//   PrimitiveBatch - vertex-coloured lines or triangles rebuilt every frame
//                    (the frustum, rays, footprint, grid, markers, the
//                    aircraft, and the HUD's panel backgrounds).
//
// Everything in the 3D view that is not the terrain is lines. That is a
// deliberate choice for a geometry debugging tool: a line either passes
// through the point it should or it visibly does not, whereas shaded solids
// hide a metre of error behind a highlight.

#include <vector>

#include "gl/gl.hpp"

namespace georay {

struct SurfaceVertex {
    glm::vec3 position;
    glm::vec3 normal;
    // Map texture coordinates, computed per post from the post's own
    // latitude and longitude through the full Web Mercator transform.
    //
    // One set, not one per layer: the map textures now cover a geographic
    // box rather than whatever tiles they were made of, so every layer - and
    // every zoom level - shares the same coordinates. That is what lets the
    // map be re-stitched at a finer zoom without touching the mesh.
    //
    // Per-vertex rather than derived in the shader from the world position,
    // because the map is in Mercator and the world is in a local tangent
    // plane; the relation is not linear, and approximating it per fragment
    // slides the map off the ground by a metre or so across the scene.
    glm::vec2 map_uv{0.0f, 0.0f};
};

class SurfaceMesh {
public:
    SurfaceMesh() = default;
    ~SurfaceMesh();

    SurfaceMesh(const SurfaceMesh&) = delete;
    SurfaceMesh& operator=(const SurfaceMesh&) = delete;

    void upload(const std::vector<SurfaceVertex>& vertices,
                const std::vector<unsigned int>& indices);
    void draw() const;
    bool empty() const { return index_count_ == 0; }

private:
    GLuint vao_ = 0, vbo_ = 0, ebo_ = 0;
    GLsizei index_count_ = 0;
};

class PrimitiveBatch {
public:
    enum class Mode { Lines, Triangles };

    explicit PrimitiveBatch(Mode mode = Mode::Lines) : mode_(mode) {}
    ~PrimitiveBatch();

    PrimitiveBatch(const PrimitiveBatch&) = delete;
    PrimitiveBatch& operator=(const PrimitiveBatch&) = delete;

    void clear() { vertices_.clear(); }

    void addSegment(const glm::vec3& a, const glm::vec3& b, const glm::vec4& colour);
    void addSegment(const glm::vec3& a, const glm::vec3& b, const glm::vec4& colour_a,
                    const glm::vec4& colour_b);
    void addBox(const glm::vec3& centre, const glm::vec3& half_extent, const glm::vec4& colour);
    // A circle in the horizontal plane, for ground markers and range rings.
    void addCircleXY(const glm::vec3& centre, float radius, const glm::vec4& colour,
                     int segments = 48);

    // Triangle-mode helpers. A filled quad is what a HUD panel background
    // is, and screen-space rectangles are the only filled geometry this tool
    // needs beyond the terrain itself.
    void addTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                     const glm::vec4& colour);
    void addTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                     const glm::vec4& ca, const glm::vec4& cb, const glm::vec4& cc);
    void addQuad(const glm::vec2& top_left, const glm::vec2& size, const glm::vec4& colour);

    // Uploads whatever has accumulated and draws it.
    void flush();

private:
    struct Vertex {
        glm::vec3 position;
        glm::vec4 colour;
    };

    void ensureBuffers();

    Mode mode_ = Mode::Lines;
    std::vector<Vertex> vertices_;
    GLuint vao_ = 0, vbo_ = 0;
    size_t capacity_ = 0;  // in vertices, so growth can be amortised
};

}  // namespace georay
