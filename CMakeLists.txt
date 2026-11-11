cmake_minimum_required(VERSION 3.20)

project(ogccdb-viewer VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ogccdb-runtimepublisher 0.1 CONFIG REQUIRED)
find_package(glfw3 3.3 CONFIG REQUIRED)

# OpenGL headers only: functions are loaded at run time (src/gl.cpp).
find_path(OGCCDB_GLCOREARB_DIR GL/glcorearb.h REQUIRED)

include(FetchContent)
# Dear ImGui and stb have no CMake build of their own; fetch pinned versions.
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.9
  GIT_SHALLOW TRUE
)
FetchContent_Declare(stb
  GIT_REPOSITORY https://github.com/nothings/stb.git
  GIT_TAG 4e9b368bfc071e97206ea5c6adba68dd0cbf8897
)
FetchContent_MakeAvailable(imgui stb)

add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC glfw ${CMAKE_DL_LIBS})

add_executable(ogccdb-viewer
  src/main.cpp
  src/gl.cpp
  src/shader.cpp
  src/globe_camera.cpp
  src/fly_camera.cpp
  src/renderer.cpp
  src/screenshot.cpp
)
target_include_directories(ogccdb-viewer SYSTEM PRIVATE ${OGCCDB_GLCOREARB_DIR} ${stb_SOURCE_DIR})
target_link_libraries(ogccdb-viewer PRIVATE ogccdb::runtime imgui glfw)
target_compile_options(ogccdb-viewer PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
)
install(TARGETS ogccdb-viewer RUNTIME DESTINATION bin)
