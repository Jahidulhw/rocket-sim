cmake_minimum_required(VERSION 3.16...3.31)
project(rocket_fc VERSION 0.1.0 LANGUAGES CXX)

# Software-in-the-loop flight computer: reads sensor frames on stdin, writes
# one reply per frame on stdout. Event detection + parachute deploy only.
#
#   cmake -S fc -B fc/build
#   cmake --build fc/build --config Release
#   ctest --test-dir fc/build -C Release --output-on-failure

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Single-config generators (Makefiles/Ninja) default to an optimised build.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(FC_BUILD_TESTS "Build the GoogleTest unit tests" ON)
option(FC_WARNINGS_AS_ERRORS "Treat compiler warnings as errors (CI)" OFF)

function(fc_set_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
    if(FC_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
    if(FC_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

# All logic lives in a library so the unit tests exercise exactly the code the
# executable runs; main.cpp is only the stdin/stdout loop.
add_library(fc_core STATIC
  src/protocol.cpp
  src/runner.cpp
)
target_include_directories(fc_core PUBLIC include)
fc_set_warnings(fc_core)

add_executable(flight_computer src/main.cpp)
target_link_libraries(flight_computer PRIVATE fc_core)
fc_set_warnings(flight_computer)

if(FC_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
    URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  # MSVC: link gtest against the same (dynamic) CRT as our targets.
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(fc_tests
    tests/test_protocol.cpp
    tests/test_runner.cpp
  )
  target_link_libraries(fc_tests PRIVATE fc_core GTest::gtest_main)
  fc_set_warnings(fc_tests)

  include(GoogleTest)
  gtest_discover_tests(fc_tests DISCOVERY_MODE PRE_TEST)
endif()
