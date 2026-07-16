cmake_minimum_required(VERSION 3.20)
project(NammaMetroRouter VERSION 1.0.0 LANGUAGES CXX)

# ─────────────────────────────────────────────
# C++17 standard — mandatory per CLAUDE.md
# ─────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ─────────────────────────────────────────────
# Build type defaulting
# ─────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ─────────────────────────────────────────────
# Compiler flags
# ─────────────────────────────────────────────
set(COMMON_FLAGS "-Wall -Wextra -Wpedantic -Wshadow -Wno-unused-parameter")

# Release: maximum vectorization + native ISA (AVX2 on Dell G15 i7-12700H)
# M5 FIX: -ffast-math enables -funsafe-math-optimizations which may reassociate
# the composite cost formula (wait + travel + lambda*crowd + penalty), changing
# which train is selected at boundary conditions. The FIFO proof and test suite
# verify Debug builds (no -ffast-math) — the Release build must match.
# -fno-math-errno + -ffinite-math-only retain vectorization benefits without
# breaking float associativity or the priority queue's strict weak ordering.
set(RELEASE_FLAGS "${COMMON_FLAGS} -O3 -march=native -funroll-loops -fno-math-errno -ffinite-math-only -DNDEBUG")

# Debug: sanitizers for allocator development
set(DEBUG_FLAGS "${COMMON_FLAGS} -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer")

# Asan test target
set(ASAN_FLAGS "${COMMON_FLAGS} -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RELEASE_FLAGS}")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEBUG_FLAGS}")
endif()

# ─────────────────────────────────────────────
# Include directories
# ─────────────────────────────────────────────
include_directories(${CMAKE_SOURCE_DIR}/include)

# ─────────────────────────────────────────────
# Core library (shared between engine + bench)
# ─────────────────────────────────────────────
add_library(namma_metro_core STATIC
  src/gtfs_parser.cpp
  src/graph_builder.cpp
  src/routing.cpp
  src/benchmark.cpp   # C2 FIX: calibrate_tsc_ns + compute_percentiles definitions
)
target_include_directories(namma_metro_core PUBLIC ${CMAKE_SOURCE_DIR}/include)

# ─────────────────────────────────────────────
# Benchmark executable
# ─────────────────────────────────────────────
add_executable(routing_engine_benchmark src/main_bench.cpp)
target_link_libraries(routing_engine_benchmark PRIVATE namma_metro_core)

# ─────────────────────────────────────────────
# Google Test via FetchContent (auto-downloaded)
# ─────────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.14.0
)
# Prevent overriding the parent project's compiler/linker settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# ─────────────────────────────────────────────
# Test executable with ASan for allocator dev
# ─────────────────────────────────────────────
add_executable(namma_metro_tests
  tests/test_fifo.cpp
  tests/test_dominance.cpp
  tests/test_lazy_deletion.cpp
  tests/test_gtfs_parser.cpp
  tests/test_graph.cpp
  tests/test_arena.cpp
  tests/test_pareto_oracle.cpp
  tests/test_fifo_invariant.cpp
)
target_link_libraries(namma_metro_tests
  PRIVATE
    namma_metro_core
    GTest::gtest_main
    GTest::gmock_main
)

# Apply sanitizers to test target in Debug mode
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(namma_metro_tests PRIVATE
    -fsanitize=address,undefined -fno-omit-frame-pointer
  )
  target_link_options(namma_metro_tests PRIVATE
    -fsanitize=address,undefined
  )
endif()

include(GoogleTest)
gtest_discover_tests(namma_metro_tests)

# ─────────────────────────────────────────────
# Static analysis helper target (optional)
# ─────────────────────────────────────────────
find_program(CLANG_TIDY clang-tidy)
if(CLANG_TIDY)
  set_target_properties(namma_metro_core PROPERTIES
    CXX_CLANG_TIDY "${CLANG_TIDY};--warnings-as-errors=*"
  )
endif()

# ─────────────────────────────────────────────
# Print memory layout sizes at configure time
# ─────────────────────────────────────────────
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Flags: ${CMAKE_CXX_FLAGS}")
