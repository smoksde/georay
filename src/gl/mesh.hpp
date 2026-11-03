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
    // Map raster coordinates for each layer, computed per post from the
    // post's own latitude and longitude. Per-vertex rather than derived in
    // the shader from the world position, because the map is in Web Mercator
    // and the world is in a local tangent plane - the relation between them
    // is not linear, and approximating it in the fragment shader would slide
    // the map off the ground by a metre or so across the scene.
    //
    // One set per layer, because the satellite and street rasters are
    // separate files that need not share a zoom or a tile range. Eight extra
    // bytes a post is cheaper than rebuilding a 263k-post mesh every time
    // somebody presses M.
    glm::vec2 map_uv[2]{{0.0f, 0.0f}, {0.0f, 0.0f}};
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
    // Connects consecutive points; `closed` adds the wrap-around segment.
    void addPolyline(const std::vector<glm::vec3>& points, const glm::vec4& colour,
                     bool closed = false);
    // A three-axis cross, for marking a point in space where a dot would be
    // ambiguous about depth.
    void addCross(const glm::vec3& centre, float size, const glm::vec4& colour);
    void addBox(const glm::vec3& centre, const glm::vec3& half_extent, const glm::vec4& colour);
    // A circle in the horizontal plane, for ground markers and range rings.
    void addCircleXY(const glm::vec3& centre, float radius, const glm::vec4& colour,
                     int segments = 48);

    // Triangle-mode helpers. A filled quad is what a HUD panel background
    // is, and screen-space rectangles are the only filled geometry this tool
    // needs beyond the terrain itself.
    void addTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                     const glm::vec4& colour);
    void addQuad(const glm::vec2& top_left, const glm::vec2& size, const glm::vec4& colour);

    // Uploads whatever has accumulated and draws it.
    void flush();

    size_t vertexCount() const { return vertices_.size(); }

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
