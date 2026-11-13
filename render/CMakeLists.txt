# ogccdb-render: draws the runtime's terrain with OpenGL 3.3 core, into a window
# or an offscreen framebuffer. No window or UI code: the caller makes the
# OpenGL context (the viewer with a window, tools with a hidden one).
add_library(ogccdb_render STATIC
  src/gl.cpp
  src/shader.cpp
  src/renderer.cpp
  src/camera.cpp
  src/framebuffer.cpp
  src/image.cpp
)
add_library(ogccdb::render ALIAS ogccdb_render)
target_include_directories(ogccdb_render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(ogccdb_render SYSTEM PUBLIC ${OGCCDB_GLCOREARB_DIR})
target_include_directories(ogccdb_render SYSTEM PRIVATE ${stb_SOURCE_DIR})
target_link_libraries(ogccdb_render PUBLIC ogccdb::runtime)
target_compile_features(ogccdb_render PUBLIC cxx_std_17)
target_compile_options(ogccdb_render PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
)
