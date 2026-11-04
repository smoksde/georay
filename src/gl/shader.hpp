#pragma once

#include <string>
#include <unordered_map>

#include "gl/gl.hpp"

namespace georay {

// A linked GLSL program, loaded from files on disk. Uniform locations are
// looked up lazily and cached - a glGetUniformLocation per uniform per frame
// is a measurable cost for no benefit.
class Shader {
public:
    Shader(const std::string& vertex_path, const std::string& fragment_path);
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    void bind() const { glUseProgram(program_); }

    // Whether the program compiled and linked - a failed one still exists
    // and binds without error, drawing nothing or garbage, so this must be
    // checked explicitly or a shader typo shows up as a blank window.
    bool ok() const { return ok_; }
    const std::string& error() const { return error_; }

    void set(const char* name, int value);
    void set(const char* name, float value);
    void set(const char* name, const glm::vec2& value);
    void set(const char* name, const glm::vec3& value);
    void set(const char* name, const glm::vec4& value);
    void set(const char* name, const glm::mat4& value);

private:
    GLint location(const char* name);
    static GLuint compile(const std::string& source, GLenum type, std::string& error_out);

    GLuint program_ = 0;
    bool ok_ = false;
    std::string error_;
    std::unordered_map<std::string, GLint> uniforms_;
};

// Absolute path to a file in the project's shaders/ directory.
std::string shaderPath(const std::string& filename);
// ...and in assets/, which holds the baked font atlas.
std::string assetPath(const std::string& filename);

}  // namespace georay
