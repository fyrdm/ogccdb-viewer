#include "ogccdb/render/shader.hpp"

#include <stdexcept>
#include <vector>

namespace {
GLuint compile(GLenum type, const char* src) {
    const GLuint s = gl::CreateShader(type);
    gl::ShaderSource(s, 1, &src, nullptr);
    gl::CompileShader(s);
    GLint ok = 0;
    gl::GetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        gl::GetShaderiv(s, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log(static_cast<std::size_t>(len > 1 ? len : 1));
        gl::GetShaderInfoLog(s, len, nullptr, log.data());
        gl::DeleteShader(s);
        throw std::runtime_error(std::string("shader compile failed: ") + log.data());
    }
    return s;
}
}  // namespace

namespace ogccdb::render {

GLuint make_program(const char* vertex_source, const char* fragment_source) {
    const GLuint vs = compile(GL_VERTEX_SHADER, vertex_source);
    const GLuint fs = compile(GL_FRAGMENT_SHADER, fragment_source);
    const GLuint p = gl::CreateProgram();
    gl::AttachShader(p, vs);
    gl::AttachShader(p, fs);
    gl::LinkProgram(p);
    gl::DeleteShader(vs);
    gl::DeleteShader(fs);
    GLint ok = 0;
    gl::GetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        gl::GetProgramiv(p, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log(static_cast<std::size_t>(len > 1 ? len : 1));
        gl::GetProgramInfoLog(p, len, nullptr, log.data());
        gl::DeleteProgram(p);
        throw std::runtime_error(std::string("shader link failed: ") + log.data());
    }
    return p;
}

}  // namespace ogccdb::render
