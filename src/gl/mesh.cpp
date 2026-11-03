#include "gl/mesh.hpp"

#include <cmath>

namespace georay {

SurfaceMesh::~SurfaceMesh() {
    if (ebo_) glDeleteBuffers(1, &ebo_);
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
}

void SurfaceMesh::upload(const std::vector<SurfaceVertex>& vertices,
                         const std::vector<unsigned int>& indices) {
    if (vao_ == 0) {
        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);
        glGenBuffers(1, &ebo_);
    }

    glBindVertexArray(vao_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(vertices.size() * sizeof(SurfaceVertex)),
                 vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indices.size() * sizeof(unsigned int)),
                 indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(SurfaceVertex),
                          (void*)offsetof(SurfaceVertex, position));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(SurfaceVertex),
                          (void*)offsetof(SurfaceVertex, normal));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(SurfaceVertex),
                          (void*)offsetof(SurfaceVertex, map_uv));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(SurfaceVertex),
                          (void*)(offsetof(SurfaceVertex, map_uv) + sizeof(glm::vec2)));

    glBindVertexArray(0);
    index_count_ = GLsizei(indices.size());
}

void SurfaceMesh::draw() const {
    if (index_count_ == 0) return;
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

PrimitiveBatch::~PrimitiveBatch() {
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
}

void PrimitiveBatch::addSegment(const glm::vec3& a, const glm::vec3& b, const glm::vec4& colour) {
    addSegment(a, b, colour, colour);
}

void PrimitiveBatch::addSegment(const glm::vec3& a, const glm::vec3& b, const glm::vec4& ca,
                           const glm::vec4& cb) {
    vertices_.push_back({a, ca});
    vertices_.push_back({b, cb});
}

void PrimitiveBatch::addPolyline(const std::vector<glm::vec3>& points, const glm::vec4& colour,
                            bool closed) {
    if (points.size() < 2) return;
    for (size_t i = 0; i + 1 < points.size(); ++i) addSegment(points[i], points[i + 1], colour);
    if (closed) addSegment(points.back(), points.front(), colour);
}

void PrimitiveBatch::addCross(const glm::vec3& c, float s, const glm::vec4& colour) {
    addSegment(c - glm::vec3(s, 0, 0), c + glm::vec3(s, 0, 0), colour);
    addSegment(c - glm::vec3(0, s, 0), c + glm::vec3(0, s, 0), colour);
    addSegment(c - glm::vec3(0, 0, s), c + glm::vec3(0, 0, s), colour);
}

void PrimitiveBatch::addBox(const glm::vec3& c, const glm::vec3& h, const glm::vec4& colour) {
    glm::vec3 corner[8];
    for (int i = 0; i < 8; ++i) {
        corner[i] = c + glm::vec3((i & 1) ? h.x : -h.x, (i & 2) ? h.y : -h.y,
                                  (i & 4) ? h.z : -h.z);
    }
    // Each edge joins two corners whose indices differ in exactly one bit.
    for (int i = 0; i < 8; ++i) {
        for (int bit = 1; bit <= 4; bit <<= 1) {
            const int j = i ^ bit;
            if (j > i) addSegment(corner[i], corner[j], colour);
        }
    }
}

void PrimitiveBatch::addCircleXY(const glm::vec3& centre, float radius,
                                 const glm::vec4& colour, int segments) {
    const int n = segments > 3 ? segments : 3;
    glm::vec3 previous = centre + glm::vec3(radius, 0, 0);
    for (int i = 1; i <= n; ++i) {
        const float a = 6.28318530718f * float(i) / float(n);
        const glm::vec3 p = centre + glm::vec3(radius * std::cos(a), radius * std::sin(a), 0);
        addSegment(previous, p, colour);
        previous = p;
    }
}

void PrimitiveBatch::addTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                                 const glm::vec4& colour) {
    vertices_.push_back({a, colour});
    vertices_.push_back({b, colour});
    vertices_.push_back({c, colour});
}

void PrimitiveBatch::addQuad(const glm::vec2& top_left, const glm::vec2& size,
                             const glm::vec4& colour) {
    const glm::vec3 tl(top_left, 0.0f);
    const glm::vec3 tr(top_left.x + size.x, top_left.y, 0.0f);
    const glm::vec3 br(top_left.x + size.x, top_left.y + size.y, 0.0f);
    const glm::vec3 bl(top_left.x, top_left.y + size.y, 0.0f);
    addTriangle(tl, tr, br, colour);
    addTriangle(tl, br, bl, colour);
}

void PrimitiveBatch::ensureBuffers() {
    if (vao_ != 0) return;
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, position));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, colour));
    glBindVertexArray(0);
}

void PrimitiveBatch::flush() {
    if (vertices_.empty()) return;
    ensureBuffers();

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    const size_t bytes = vertices_.size() * sizeof(Vertex);
    if (vertices_.size() > capacity_) {
        // Grow in doublings and reallocate with glBufferData; steady state is
        // then glBufferSubData into a buffer that is already big enough,
        // which is what keeps a per-frame rebuild cheap.
        capacity_ = vertices_.size() * 2;
        glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(capacity_ * sizeof(Vertex)), nullptr,
                     GL_DYNAMIC_DRAW);
    }
    glBufferSubData(GL_ARRAY_BUFFER, 0, GLsizeiptr(bytes), vertices_.data());

    glDrawArrays(mode_ == Mode::Lines ? GL_LINES : GL_TRIANGLES, 0,
                 GLsizei(vertices_.size()));
    glBindVertexArray(0);
    vertices_.clear();
}

}  // namespace georay
