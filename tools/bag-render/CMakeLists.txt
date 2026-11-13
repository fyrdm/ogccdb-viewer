# ogccdb-bag-render: adds a simulated camera to a recorded flight (a ROS 2 bag),
# rendered from a CDB store without a window. The camera model, image realism
# and prior-pose drift are a small library of their own, so they are tested
# without OpenGL.
add_library(ogccdb_camera_sim STATIC camera_sim.cpp)
target_include_directories(ogccdb_camera_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ogccdb_camera_sim PUBLIC ogccdb::render ogccdb::flight ogccdb::runtime)
target_compile_features(ogccdb_camera_sim PUBLIC cxx_std_17)
target_compile_options(ogccdb_camera_sim PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_executable(ogccdb-bag-render main.cpp)
target_link_libraries(ogccdb-bag-render PRIVATE ogccdb_camera_sim glfw)
target_compile_options(ogccdb-bag-render PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
)
# Next to the viewer in the build folder.
set_target_properties(ogccdb-bag-render PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
install(TARGETS ogccdb-bag-render RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(GTest CONFIG)
  if(NOT GTest_FOUND)
    find_package(GTest)
  endif()
  if(GTest_FOUND)
    add_executable(ogccdb_camera_sim_tests camera_sim_test.cpp)
    target_link_libraries(ogccdb_camera_sim_tests PRIVATE ogccdb_camera_sim GTest::gtest_main)
    add_test(NAME ogccdb_camera_sim_tests COMMAND ogccdb_camera_sim_tests)
  endif()
endif()
