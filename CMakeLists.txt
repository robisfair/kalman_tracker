cmake_minimum_required(VERSION 4.2.3)
project(KalmanTracker VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ── Dependencies ──────────────────────────────────────────────────────────────

# Eigen3 (header-only linear algebra)
find_package(Eigen3 5.0.1 REQUIRED NO_MODULE)

# nlohmann/json (header-only JSON)
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  )
  FetchContent_MakeAvailable(nlohmann_json)
endif()

# Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/v1.14.0.tar.gz
)
FetchContent_MakeAvailable(googletest)

# ── Library target ────────────────────────────────────────────────────────────
add_library(tracker_lib
  src/target.cpp
  src/sensor.cpp
  src/kalman_filter.cpp
  src/simulation.cpp
  src/statistics.cpp
  src/real_track.cpp
)

target_include_directories(tracker_lib PUBLIC include)
target_link_libraries(tracker_lib PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json)

# ── Main executable ───────────────────────────────────────────────────────────
add_executable(kalman_tracker src/main.cpp)
target_link_libraries(kalman_tracker PRIVATE tracker_lib)

# ── Tests ─────────────────────────────────────────────────────────────────────
enable_testing()
add_executable(tracker_tests
  tests/test_kalman_filter.cpp
  tests/test_sensor.cpp
  tests/test_statistics.cpp
)
target_link_libraries(tracker_tests PRIVATE tracker_lib GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(tracker_tests)
