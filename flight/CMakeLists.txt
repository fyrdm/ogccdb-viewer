# ogccdb-flight: vehicle flights from ROS 2 bags (MCAP or SQLite storage),
# without ROS, and MCAP bags written back. No graphics: usable from any
# renderer or engine.
find_package(SQLite3 REQUIRED)

add_library(ogccdb_flight STATIC
  src/bag.cpp
  src/tar.cpp
  src/mcap.cpp
  src/mcap_writer.cpp
  src/bag_metadata.cpp
  src/sqlite_bag.cpp
  src/messages.cpp
  src/track.cpp
)
add_library(ogccdb::flight ALIAS ogccdb_flight)
target_include_directories(ogccdb_flight PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ogccdb_flight PUBLIC ogccdb::runtime PRIVATE SQLite::SQLite3)
target_compile_features(ogccdb_flight PUBLIC cxx_std_17)

# Compressed MCAP chunks: zstd and lz4, if their libraries are installed.
find_path(OGCCDB_ZSTD_INCLUDE zstd.h)
find_library(OGCCDB_ZSTD_LIB zstd)
if(OGCCDB_ZSTD_INCLUDE AND OGCCDB_ZSTD_LIB)
  target_include_directories(ogccdb_flight PRIVATE ${OGCCDB_ZSTD_INCLUDE})
  target_link_libraries(ogccdb_flight PRIVATE ${OGCCDB_ZSTD_LIB})
  target_compile_definitions(ogccdb_flight PRIVATE OGCCDB_FLIGHT_HAVE_ZSTD)
endif()
find_path(OGCCDB_LZ4_INCLUDE lz4frame.h)
find_library(OGCCDB_LZ4_LIB lz4)
if(OGCCDB_LZ4_INCLUDE AND OGCCDB_LZ4_LIB)
  target_include_directories(ogccdb_flight PRIVATE ${OGCCDB_LZ4_INCLUDE})
  target_link_libraries(ogccdb_flight PRIVATE ${OGCCDB_LZ4_LIB})
  target_compile_definitions(ogccdb_flight PRIVATE OGCCDB_FLIGHT_HAVE_LZ4)
endif()
target_compile_options(ogccdb_flight PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_executable(ogccdb-bag-info tools/bag_info.cpp)
target_link_libraries(ogccdb-bag-info PRIVATE ogccdb::flight)
install(TARGETS ogccdb-bag-info RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(GTest CONFIG)
  if(NOT GTest_FOUND)
    find_package(GTest)
  endif()
  if(GTest_FOUND)
    add_executable(ogccdb_flight_tests tests/flight_test.cpp)
    target_link_libraries(ogccdb_flight_tests PRIVATE ogccdb::flight GTest::gtest_main)
    add_test(NAME ogccdb_flight_tests COMMAND ogccdb_flight_tests)
  endif()
endif()
