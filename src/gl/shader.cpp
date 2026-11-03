#include "gl/shader.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#ifndef GEORAY_SHADER_DIR
#define GEORAY_SHADER_DIR "shaders"
#endif

namespace georay {
namespace {

bool readFile(const std::string& path, std::string& out) {
    std::ifstream file(path);
    if (!file) return false;
    std::ostringstream ss;
    ss << file.rdbuf();
    out = ss.str();
    return true;
}

}  // namespace

std::string shaderPath(const std::string& filename) {
    return std::string(GEORAY_SHADER_DIR) + "/" + filename;
}

GLuint Shader::compile(const std::string& source, GLenum type, std::string& error_out) {
    const GLuint shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint status = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE) {
        GLint length = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
        std::vector<char> log(size_t(length > 1 ? length : 1));
        glGetShaderInfoLog(shader, length, nullptr, log.data());
        error_out = std::string(log.data());
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

Shader::Shader(const std::string& vertex_path, const std::string& fragment_path) {
    std::string vertex_source, fragment_source;
    if (!readFile(vertex_path, vertex_source)) {
        error_ = "cannot read " + vertex_path;
        std::cerr << "georay: " << error_ << "\n";
        return;
    }
    if (!readFile(fragment_path, fragment_source)) {
        error_ = "cannot read " + fragment_path;
        std::cerr << "georay: " << error_ << "\n";
        return;
    }

    std::string stage_error;
    const GLuint vertex = compile(vertex_source, GL_VERTEX_SHADER, stage_error);
    if (vertex == 0) {
        error_ = vertex_path + ": " + stage_error;
        std::cerr << "georay: " << error_ << "\n";
        return;
    }
    const GLuint fragment = compile(fragment_source, GL_FRAGMENT_SHADER, stage_error);
    if (fragment == 0) {
        error_ = fragment_path + ": " + stage_error;
        std::cerr << "georay: " << error_ << "\n";
        glDeleteShader(vertex);
        return;
    }

    program_ = glCreateProgram();
    glAttachShader(program_, vertex);
    glAttachShader(program_, fragment);
    glLinkProgram(program_);
    glDeleteShader(vertex);
    glDeleteShader(fragment);

    GLint status = GL_FALSE;
    glGetProgramiv(program_, GL_LINK_STATUS, &status);
    if (status != GL_TRUE) {
        GLint length = 0;
        glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &length);
        std::vector<char> log(size_t(length > 1 ? length : 1));
        glGetProgramInfoLog(program_, length, nullptr, log.data());
        error_ = "link failed: " + std::string(log.data());
        std::cerr << "georay: " << error_ << "\n";
        return;
    }
    ok_ = true;
}

Shader::~Shader() {
    if (program_ != 0) glDeleteProgram(program_);
}

GLint Shader::location(const char* name) {
    const auto it = uniforms_.find(name);
    if (it != uniforms_.end()) return it->second;
    const GLint loc = glGetUniformLocation(program_, name);
    uniforms_.emplace(name, loc);
    return loc;
}

void Shader::set(const char* name, int v) { glUniform1i(location(name), v); }
void Shader::set(const char* name, float v) { glUniform1f(location(name), v); }
void Shader::set(const char* name, const glm::vec2& v) { glUniform2fv(location(name), 1, glm::value_ptr(v)); }
void Shader::set(const char* name, const glm::vec3& v) { glUniform3fv(location(name), 1, glm::value_ptr(v)); }
void Shader::set(const char* name, const glm::vec4& v) { glUniform4fv(location(name), 1, glm::value_ptr(v)); }
void Shader::set(const char* name, const glm::mat4& v) {
    glUniformMatrix4fv(location(name), 1, GL_FALSE, glm::value_ptr(v));
}

}  // namespace georay
