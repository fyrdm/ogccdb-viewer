#pragma once
#include <string>

#include "gl.hpp"

/// Compiles and links a vertex + fragment shader pair. Throws std::runtime_error
/// with the driver's log on failure.
GLuint make_program(const char* vertex_source, const char* fragment_source);
