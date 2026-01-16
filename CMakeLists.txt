cmake_minimum_required(VERSION 3.23)

# Use new behaviour for FindBoost if present (CMP0167)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# Use new download timestamp behavior if present (CMP0135)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

# Enable ccache if available
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# ──────────────────────────────────────────────────────────────
#  Project
# ──────────────────────────────────────────────────────────────
project(arcgen VERSION 1.0.0 LANGUAGES CXX)

# Export compile_commands.json for tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ──────────────────────────────────────────────────────────────
#  Build options
# ──────────────────────────────────────────────────────────────
option(AG_BUILD_TESTS           "Build unit / property / timing tests"                          OFF)
option(AG_ENABLE_PLOTS          "Generate SVG plots in tests"                                   OFF)
option(AG_BUILD_DEMOS           "Build demo tools (e.g. visualization generator)"               OFF)
option(AG_STRICT_WARNINGS       "Enable a strict warnings profile"                              ON)
option(AG_ENABLE_IPO            "Enable inter-procedural optimization (LTO)"                    OFF)
option(AG_ENABLE_OPENMP         "Enable OpenMP parallelism in arcgen"                           OFF)
option(AG_ENABLE_TEST_REPORT    "Generate a comprehensive planner statistics report from tests" OFF)
if (AG_ENABLE_TEST_REPORT)
    add_compile_definitions (AG_ENABLE_TEST_REPORT)
endif ()

option(AG_ENABLE_COVERAGE "Enable code coverage generation (GCC/Clang)" OFF)
if(AG_ENABLE_COVERAGE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(--coverage)
    add_link_options(--coverage)
  else()
    message(WARNING "Coverage not supported for this compiler")
  endif()
endif()

# ──────────────────────────────────────────────────────────────
#  Compiler settings
# ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

# Optional IPO/LTO
if(AG_ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_ok OUTPUT ipo_msg)
  if(ipo_ok)
    message(STATUS "IPO/LTO enabled")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
  else()
    message(WARNING "IPO/LTO not supported: ${ipo_msg}")
  endif()
endif()

# ──────────────────────────────────────────────────────────────
#  Dependencies
# ──────────────────────────────────────────────────────────────
include(FetchContent)

# 1. Boost (Header-Only)
find_package(Boost 1.72 QUIET)
if(NOT Boost_FOUND)
  message(STATUS "Boost not found locally. Fetching Boost 1.83.0 (Flat)...")
  FetchContent_Declare(
    boost
    URL https://archives.boost.io/release/1.83.0/source/boost_1_83_0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP ON
  )
  # Populate only (skip build to treat as header-only)
  FetchContent_GetProperties(boost)
  if(NOT boost_POPULATED)
    FetchContent_Populate(boost)
  endif()
  
  set(Boost_INCLUDE_DIRS ${boost_SOURCE_DIR})
  message(STATUS "Using fetched Boost from: ${Boost_INCLUDE_DIRS}")
endif()

# 2. CGAL (Header-Only)
# CGAL uses legacy FindBoost internally; suppress policy warning if we found system boost
if(POLICY CMP0167)
  cmake_policy(PUSH)
  cmake_policy(SET CMP0167 OLD)
endif()

find_package(CGAL QUIET CONFIG)
if(NOT CGAL_FOUND)
  message(STATUS "CGAL not found locally. Fetching CGAL 5.6...")
  FetchContent_Declare(
    cgal
    URL https://github.com/CGAL/cgal/releases/download/v5.6/CGAL-5.6.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP ON
  )
  # Manually populate to avoid add_subdirectory which triggers CGAL's complex build
  FetchContent_GetProperties(cgal)
  if(NOT cgal_POPULATED)
    FetchContent_Populate(cgal)
  endif()
  
  # create a header-only target for CGAL
  if(NOT TARGET CGAL::CGAL)
    add_library(CGAL::CGAL INTERFACE IMPORTED)
    target_include_directories(CGAL::CGAL INTERFACE "$<BUILD_INTERFACE:${cgal_SOURCE_DIR}/include>")
    # CGAL needs GMP/MPFR. In dockcross they are usually in /usr/lib or can be disabled/mocked if strictly header-only
    # For now, let's assume the container has libgmp/libmpfr, or we might need to fetch those too.
  endif()
endif()

if(POLICY CMP0167)
  cmake_policy(POP)
endif()

# Optional OpenMP
if(AG_ENABLE_OPENMP)
  find_package(OpenMP REQUIRED)
endif()

# ──────────────────────────────────────────────────────────────
#  Header-only library target
# ──────────────────────────────────────────────────────────────
add_library(arcgen INTERFACE)
add_library(arcgen::arcgen ALIAS arcgen)

# Public headers for consumers
target_include_directories(arcgen
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(arcgen SYSTEM
  INTERFACE
    $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
)

# Link interface deps so consumers link what they need
target_link_libraries(arcgen
  INTERFACE
    CGAL::CGAL
)

# Only link OpenMP (and thus propagate flags) if enabled
if(AG_ENABLE_OPENMP)
  target_link_libraries(arcgen INTERFACE OpenMP::OpenMP_CXX)
endif()

# Require C++23 for consumers
target_compile_features(arcgen INTERFACE cxx_std_23)

# ───────────────────────── Installation & Package ─────────────
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# The package config file that consumers will find via find_package(arcgen)
set(arcgen_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/arcgen")

# Configure arcgenConfig.cmake from template
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arcgenConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/arcgenConfig.cmake"
  INSTALL_DESTINATION "${arcgen_INSTALL_CMAKEDIR}"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/arcgenConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

# Install header-only target and headers
install(TARGETS arcgen
        EXPORT  arcgenTargets)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT arcgenTargets
        FILE arcgenTargets.cmake
        NAMESPACE arcgen::
        DESTINATION ${arcgen_INSTALL_CMAKEDIR})

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/arcgenConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/arcgenConfigVersion.cmake"
    DESTINATION ${arcgen_INSTALL_CMAKEDIR}
)

# ──────────────────────────────────────────────────────────────
#  Utils Helper Library (Shared by Tests & Tools)
# ──────────────────────────────────────────────────────────────
add_library(arcgen_utils INTERFACE)
add_library(arcgen::utils ALIAS arcgen_utils)
target_include_directories(arcgen_utils INTERFACE common)
target_link_libraries(arcgen_utils INTERFACE arcgen::arcgen)

# ──────────────────────────────────────────────────────────────
#  Tests
# ──────────────────────────────────────────────────────────────
if(AG_BUILD_TESTS)
  enable_testing()

  include(FetchContent)
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    message(STATUS "GoogleTest not found locally, fetching from source...")
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  add_executable(ag_tests
    tests/core/math_tests.cpp
    tests/core/state_tests.cpp
    tests/core/control_tests.cpp
    tests/core/numeric_tests.cpp
    tests/steering/path_tests.cpp
    tests/steering/dubins_tests.cpp
    tests/steering/reeds_shepp_tests.cpp
    tests/planner/geometry/straight_skeleton_tests.cpp
    tests/planner/geometry/robot_tests.cpp
    tests/planner/geometry/workspace_tests.cpp
    tests/planner/graph/astar_tests.cpp
    tests/planner/constraints/collision_tests.cpp
    tests/planner/constraints/path_length_tests.cpp
    tests/planner/constraints/footprint_collision_tests.cpp
    tests/planner/evaluator_tests.cpp
    tests/planner/connector/greedy_connector_tests.cpp
    tests/planner/graph/graph_search_tests.cpp
    tests/planner/engine/search_engine_tests.cpp

  )

  target_link_libraries(ag_tests
    PRIVATE
      arcgen::arcgen
      arcgen::utils
      GTest::gtest_main
  )

  # If OpenMP is enabled, ensure the tests inherit it (explicit).
  if(AG_ENABLE_OPENMP)
    target_link_libraries(ag_tests PRIVATE OpenMP::OpenMP_CXX)
  endif()

  target_include_directories(ag_tests
    PRIVATE
      ${PROJECT_SOURCE_DIR}/tests
  )

  target_include_directories(ag_tests SYSTEM
    PRIVATE
      ${Boost_INCLUDE_DIRS}
  )

  # Optional plotting in tests only
  if(AG_ENABLE_PLOTS)
    target_compile_definitions(ag_tests PRIVATE AG_ENABLE_PLOTS)
  endif()

  # Apply strict warnings to the test binary (compiler-specific)
  if(AG_STRICT_WARNINGS)
    if(MSVC)
      target_compile_options(ag_tests PRIVATE /W4 /permissive- /w44265 /w44062)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(ag_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
        -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual
        -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wuseless-cast)

    # Suppress -Wmaybe-uninitialized for specific tests using Boost.Geometry
    # which triggers false positives in GCC.
    set(BOOST_GEOMETRY_TESTS
      tests/planner/geometry/robot_tests.cpp
      tests/planner/geometry/workspace_tests.cpp
      tests/planner/graph/graph_search_tests.cpp
      tests/planner/engine/search_engine_tests.cpp
    )
    set_source_files_properties(${BOOST_GEOMETRY_TESTS} PROPERTIES 
        COMPILE_OPTIONS "-Wno-maybe-uninitialized")

    # Also suppress -Wpsabi to avoid "parameter passing for argument..." notes on ARM.
    target_compile_options(ag_tests PRIVATE -Wno-psabi)

    else() # Clang / AppleClang and others
      target_compile_options(ag_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
        -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual)
    endif()
  endif()

  include(GoogleTest)
  gtest_discover_tests(ag_tests
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS "arcgen"
  )
endif()
# ──────────────────────────────────────────────────────────────
#  Tools / Demos
# ──────────────────────────────────────────────────────────────
if(AG_BUILD_DEMOS)
  message(STATUS "Building Demos (Visualization Tool)")
  add_executable(generate_visualizations tools/generate_visualizations.cpp)
  
  target_link_libraries(generate_visualizations
    PRIVATE
      arcgen::arcgen
      arcgen::utils
  )
  
  target_include_directories(generate_visualizations SYSTEM
    PRIVATE
      ${Boost_INCLUDE_DIRS}
  )

  # Strict warnings for tool as well
  if(AG_STRICT_WARNINGS)
      if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
          target_compile_options(generate_visualizations PRIVATE -Wall -Wextra -O3)
      else()
          target_compile_options(generate_visualizations PRIVATE -Wall -Wextra -O3)
      endif()
  endif()
endif()
