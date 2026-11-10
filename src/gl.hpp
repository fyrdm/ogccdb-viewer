#pragma once
// Minimal OpenGL 3.3 core function loader. Function pointers live in the
// `gl` namespace (gl::Clear, ...) so they never clash with symbols that a
// system OpenGL library exports.
#include <GL/glcorearb.h>

#define OGCCDB_GL_FUNCTIONS(X)                                         \
    X(PFNGLCLEARPROC, Clear)                                           \
    X(PFNGLCLEARCOLORPROC, ClearColor)                                 \
    X(PFNGLCLEARDEPTHPROC, ClearDepth)                                 \
    X(PFNGLENABLEPROC, Enable)                                         \
    X(PFNGLDISABLEPROC, Disable)                                       \
    X(PFNGLVIEWPORTPROC, Viewport)                                     \
    X(PFNGLDEPTHFUNCPROC, DepthFunc)                                   \
    X(PFNGLDEPTHMASKPROC, DepthMask)                                   \
    X(PFNGLPOLYGONMODEPROC, PolygonMode)                               \
    X(PFNGLPOLYGONOFFSETPROC, PolygonOffset)                           \
    X(PFNGLBLENDFUNCPROC, BlendFunc)                                   \
    X(PFNGLGETINTEGERVPROC, GetIntegerv)                               \
    X(PFNGLGETFLOATVPROC, GetFloatv)                                   \
    X(PFNGLGETSTRINGPROC, GetString)                                   \
    X(PFNGLGETSTRINGIPROC, GetStringi)                                 \
    X(PFNGLREADPIXELSPROC, ReadPixels)                                 \
    X(PFNGLREADBUFFERPROC, ReadBuffer)                                 \
    X(PFNGLPIXELSTOREIPROC, PixelStorei)                               \
    X(PFNGLFINISHPROC, Finish)                                         \
    X(PFNGLGENVERTEXARRAYSPROC, GenVertexArrays)                       \
    X(PFNGLBINDVERTEXARRAYPROC, BindVertexArray)                       \
    X(PFNGLDELETEVERTEXARRAYSPROC, DeleteVertexArrays)                 \
    X(PFNGLGENBUFFERSPROC, GenBuffers)                                 \
    X(PFNGLBINDBUFFERPROC, BindBuffer)                                 \
    X(PFNGLBUFFERDATAPROC, BufferData)                                 \
    X(PFNGLDELETEBUFFERSPROC, DeleteBuffers)                           \
    X(PFNGLVERTEXATTRIBPOINTERPROC, VertexAttribPointer)               \
    X(PFNGLENABLEVERTEXATTRIBARRAYPROC, EnableVertexAttribArray)       \
    X(PFNGLCREATESHADERPROC, CreateShader)                             \
    X(PFNGLSHADERSOURCEPROC, ShaderSource)                             \
    X(PFNGLCOMPILESHADERPROC, CompileShader)                           \
    X(PFNGLGETSHADERIVPROC, GetShaderiv)                               \
    X(PFNGLGETSHADERINFOLOGPROC, GetShaderInfoLog)                     \
    X(PFNGLDELETESHADERPROC, DeleteShader)                             \
    X(PFNGLCREATEPROGRAMPROC, CreateProgram)                           \
    X(PFNGLATTACHSHADERPROC, AttachShader)                             \
    X(PFNGLLINKPROGRAMPROC, LinkProgram)                               \
    X(PFNGLGETPROGRAMIVPROC, GetProgramiv)                             \
    X(PFNGLGETPROGRAMINFOLOGPROC, GetProgramInfoLog)                   \
    X(PFNGLUSEPROGRAMPROC, UseProgram)                                 \
    X(PFNGLDELETEPROGRAMPROC, DeleteProgram)                           \
    X(PFNGLGETUNIFORMLOCATIONPROC, GetUniformLocation)                 \
    X(PFNGLUNIFORMMATRIX4FVPROC, UniformMatrix4fv)                     \
    X(PFNGLUNIFORM1IPROC, Uniform1i)                                   \
    X(PFNGLUNIFORM1FPROC, Uniform1f)                                   \
    X(PFNGLUNIFORM3FPROC, Uniform3f)                                   \
    X(PFNGLUNIFORM4FPROC, Uniform4f)                                   \
    X(PFNGLGENTEXTURESPROC, GenTextures)                               \
    X(PFNGLBINDTEXTUREPROC, BindTexture)                               \
    X(PFNGLTEXIMAGE2DPROC, TexImage2D)                                 \
    X(PFNGLTEXPARAMETERIPROC, TexParameteri)                           \
    X(PFNGLTEXPARAMETERFPROC, TexParameterf)                           \
    X(PFNGLGENERATEMIPMAPPROC, GenerateMipmap)                         \
    X(PFNGLDELETETEXTURESPROC, DeleteTextures)                         \
    X(PFNGLACTIVETEXTUREPROC, ActiveTexture)                           \
    X(PFNGLDRAWELEMENTSPROC, DrawElements)                             \
    X(PFNGLDRAWARRAYSPROC, DrawArrays)

namespace gl {
#define OGCCDB_GL_DECLARE(type, name) extern type name;
OGCCDB_GL_FUNCTIONS(OGCCDB_GL_DECLARE)
#undef OGCCDB_GL_DECLARE

using LoadFn = void* (*)(const char* name);
/// Loads every function with `load` (e.g. glfwGetProcAddress). Returns the
/// name of the first one missing, or nullptr if all were found.
const char* load(LoadFn load);
/// True if the context offers the extension.
bool has_extension(const char* name);
}  // namespace gl
