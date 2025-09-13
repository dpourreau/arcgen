cmake_minimum_required(VERSION 3.23)

# Use new behaviour for FindPackageHandleStandardArgs if present
cmake_policy(SET CMP0167 NEW)

# ──────────────────────────────────────────────────────────────
#  Project
# ──────────────────────────────────────────────────────────────
project(arcgen VERSION 1.0.0 LANGUAGES CXX)

# Export compile_commands.json for tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ──────────────────────────────────────────────────────────────
#  Build options
# ──────────────────────────────────────────────────────────────
option(AG_BUILD_TESTS      "Build unit / property / timing tests"           OFF)
option(AG_ENABLE_PLOTS     "Generate SVG plots in tests"                    OFF)
option(AG_STRICT_WARNINGS  "Enable a strict warnings profile"               ON)
option(AG_ENABLE_IPO       "Enable inter-procedural optimization (LTO)"     OFF)
option(AG_ENABLE_OPENMP    "Enable OpenMP parallelism in arcgen"            OFF)

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
find_package(Boost REQUIRED)                # headers (graph & geometry)

# CGAL uses legacy FindBoost internally; keep their behavior isolated.
if(POLICY CMP0167)
  cmake_policy(PUSH)
  cmake_policy(SET CMP0167 OLD)             # allow legacy FindBoost usage inside CGAL
endif()
find_package(CGAL REQUIRED CONFIG)          # imported target CGAL::CGAL
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
    ${Boost_INCLUDE_DIRS}
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
#  Tests
# ──────────────────────────────────────────────────────────────
if(AG_BUILD_TESTS)
  enable_testing()

  find_package(GTest QUIET CONFIG)
  if(NOT GTest_FOUND)
    message(FATAL_ERROR "GoogleTest not found")
  endif()

  add_executable(ag_tests
    tests/steering/steering_tests.cpp
    tests/geometry/skeleton_tests.cpp
    tests/geometry/robot_tests.cpp
    tests/planning/search/graph_search_tests.cpp
    tests/planning/engine/search_engine_tests.cpp
  )

  target_link_libraries(ag_tests
    PRIVATE
      arcgen::arcgen
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
