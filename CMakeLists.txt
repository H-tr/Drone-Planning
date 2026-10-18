cmake_minimum_required(VERSION 3.20)
project(drone_planning_ext LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(CheckCXXCompilerFlag)
# BoostConfig.cmake lookup: conda Boost in pixi, boost-devel 1.75 on
# manylinux_2_34, and the from-source Boost 1.82 (via Boost_ROOT) that the
# release workflow builds for manylinux_2_28.
cmake_policy(SET CMP0167 NEW)

# ── Dependencies ──────────────────────────────────────────────────────

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(nanobind CONFIG REQUIRED)
# Boost.System is header-only; its stub library was removed in Boost 1.89.
find_package(Boost REQUIRED COMPONENTS serialization filesystem)
find_package(Eigen3 REQUIRED NO_MODULE)

# ── Architecture flags ───────────────────────────────────────────────
# x86-64-v3 is the Haswell (2013+) micro-arch level: guarantees AVX2,
# BMI1/BMI2, FMA — everything VAMP/SIMDxorshift rely on. Using a fixed
# level instead of -march=native so CI-built wheels run on any modern
# x86_64 CPU, not only the builder. Override for local perf builds:
#   cmake -DDRONE_ARCH_FLAGS=-march=native ...
set(DRONE_ARCH_FLAGS "-march=x86-64-v3"
    CACHE STRING "Target CPU arch flags for native extensions")

# Older compilers may not recognize newer micro-architecture levels such as
# x86-64-v3. If that happens, keep the required SIMD feature set by falling
# back to explicit AVX2/FMA/BMI2 flags.
set(_drone_arch_flags "${DRONE_ARCH_FLAGS}")
check_cxx_compiler_flag("${_drone_arch_flags}" DRONE_ARCH_FLAGS_SUPPORTED)
if(NOT DRONE_ARCH_FLAGS_SUPPORTED)
    message(WARNING "Compiler does not support DRONE_ARCH_FLAGS='${DRONE_ARCH_FLAGS}'. Falling back to '-march=x86-64 -mavx2 -mfma -mbmi2'.")
    set(_drone_arch_flags "-march=x86-64 -mavx2 -mfma -mbmi2")
    check_cxx_compiler_flag("${_drone_arch_flags}" DRONE_ARCH_FALLBACK_SUPPORTED)
    if(NOT DRONE_ARCH_FALLBACK_SUPPORTED)
        message(FATAL_ERROR "Compiler does not support fallback architecture flags '${_drone_arch_flags}'. Set -DDRONE_ARCH_FLAGS to a supported value.")
    endif()
endif()

separate_arguments(DRONE_ARCH_FLAGS_LIST NATIVE_COMMAND "${_drone_arch_flags}")

# ── VAMP dependencies (header-only, fetched via CPM) ─────────────────

include(third_party/vamp/cmake/FetchInitCPM.cmake)
include(third_party/vamp/cmake/Dependencies.cmake)

# SIMDxorshift needs AVX2 (conda compiler defaults to -march=nocona)
if(TARGET simdxorshift)
    target_compile_options(simdxorshift PRIVATE ${DRONE_ARCH_FLAGS_LIST})
endif()

# cddlib backs VAMP's polytope obstacles, which the extension does not use.
set_target_properties(cdd PROPERTIES EXCLUDE_FROM_ALL TRUE)

# VAMP headers (upstream, header-only) plus our own robot model header
# shipped alongside the ompl_vamp extension. The custom header layout
# mirrors vamp's (include/vamp/robots/<name>.hh) so code in ext/ompl_vamp/
# can write `#include <vamp/robots/x500.hh>` without forking the vamp
# submodule.
set(VAMP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vamp/src/impl")
set(OMPL_VAMP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ext/ompl_vamp/include")

# ── OMPL (from source, no Python bindings) ────────────────────────────

set(OMPL_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_VAMP OFF CACHE BOOL "" FORCE)
set(OMPL_REGISTRATION OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_SHARED OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/ompl EXCLUDE_FROM_ALL)

# ── OMPL + VAMP extension ────────────────────────────────────────────

# NOMINSIZE: nanobind compiles modules with -Os by default, which runs the
# VAMP collision kernels 2.5x slower than -O3 (validate_batch on the demo
# room: 12.3 vs 5.0 us per configuration).
nanobind_add_module(_ompl_vamp NOMINSIZE
    ext/ompl_vamp/ompl_vamp_ext.cpp
)

target_include_directories(_ompl_vamp PRIVATE
    ${OMPL_VAMP_INCLUDE_DIR}
    ${VAMP_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ompl/src
)

target_link_libraries(_ompl_vamp PRIVATE
    ompl
    Eigen3::Eigen
    nigh
    pdqsort
    Boost::serialization
    Boost::filesystem
)

target_compile_options(_ompl_vamp PRIVATE
    ${DRONE_ARCH_FLAGS_LIST}
)

install(TARGETS _ompl_vamp LIBRARY DESTINATION drone_planning)

# ── Time parameterization extension (TOPP-RA, Eigen-only) ────────────

file(GLOB_RECURSE TOPPRA_SOURCES "ext/time_parameterization/toppra/*.cpp")

nanobind_add_module(_time_parameterization
    ${TOPPRA_SOURCES}
    ext/time_parameterization/time_parameterization_ext.cpp
)

target_include_directories(_time_parameterization PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ext/time_parameterization
)

target_link_libraries(_time_parameterization PRIVATE
    Eigen3::Eigen
)

target_compile_options(_time_parameterization PRIVATE
    ${DRONE_ARCH_FLAGS_LIST}
)

install(TARGETS _time_parameterization LIBRARY DESTINATION drone_planning)
