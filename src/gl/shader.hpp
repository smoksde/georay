#pragma once

#include <string>
#include <unordered_map>

#include "gl/gl.hpp"

namespace georay {

// A linked GLSL program, loaded from files on disk.
//
// Uniform locations are looked up lazily and cached, because the alternative
// - a glGetUniformLocation per uniform per frame - is a measurable cost for
// no benefit, and naming uniforms at the call site keeps the drawing code
// readable.
class Shader {
public:
    Shader(const std::string& vertex_path, const std::string& fragment_path);
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    void bind() const { glUseProgram(program_); }

    // Whether the program compiled and linked. A failed program still exists
    // as an object and still binds without error - it just draws nothing, or
    // garbage. So this has to be checked explicitly rather than assumed, or
    // a shader typo shows up as a blank window with no message.
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

}  // namespace georay
