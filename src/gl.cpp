#include "gl.hpp"

#include <cstring>

namespace gl {
#define OGCCDB_GL_DEFINE(type, name) type name = nullptr;
OGCCDB_GL_FUNCTIONS(OGCCDB_GL_DEFINE)
#undef OGCCDB_GL_DEFINE

const char* load(LoadFn fn) {
#define OGCCDB_GL_LOAD(type, name)                                \
    name = reinterpret_cast<type>(fn("gl" #name));                \
    if (!name) return "gl" #name;
    OGCCDB_GL_FUNCTIONS(OGCCDB_GL_LOAD)
#undef OGCCDB_GL_LOAD
    return nullptr;
}

bool has_extension(const char* ext) {
    GLint n = 0;
    GetIntegerv(GL_NUM_EXTENSIONS, &n);
    for (GLint i = 0; i < n; ++i) {
        const auto* e = reinterpret_cast<const char*>(GetStringi(GL_EXTENSIONS, static_cast<GLuint>(i)));
        if (e && std::strcmp(e, ext) == 0) return true;
    }
    return false;
}
}  // namespace gl
