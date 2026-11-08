cmake_minimum_required(VERSION 3.20)

project(GeodesicDistance VERSION 1.0 LANGUAGES CXX)

# The original implementation is header-implemented: putting it in a compiled
# static library would duplicate its non-inline definitions in client code.
# This target is therefore intentionally INTERFACE-only.
add_library(geodesic_distance INTERFACE)
add_library(geodesic::distance ALIAS geodesic_distance)

target_include_directories(geodesic_distance INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_compile_features(geodesic_distance INTERFACE cxx_std_14)

# MSVC 19.4x hides std::auto_ptr unless this legacy compatibility switch is set.
if(MSVC)
    target_compile_definitions(geodesic_distance INTERFACE _HAS_AUTO_PTR_ETC=1)
endif()

option(GEODESIC_DISTANCE_BUILD_EXAMPLES "Build the upstream command-line examples" OFF)
if(GEODESIC_DISTANCE_BUILD_EXAMPLES)
    add_executable(geodesic_example0 src/example0.cpp)
    target_link_libraries(geodesic_example0 PRIVATE geodesic::distance)

    add_executable(geodesic_example1 src/example1.cpp)
    target_link_libraries(geodesic_example1 PRIVATE geodesic::distance)

    add_executable(ply_exact_random_pairs examples/ply_exact_random_pairs.cpp)
    target_link_libraries(ply_exact_random_pairs PRIVATE geodesic::distance)
endif()

# Hessian VIPSS usage:
#   add_subdirectory(path/to/geodesic_distance)
#   target_link_libraries(hessian_vipss_target PRIVATE geodesic::distance)
