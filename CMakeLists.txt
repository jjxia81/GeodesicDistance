cmake_minimum_required(VERSION 3.20)

project(GeodesicDistance VERSION 1.0 LANGUAGES CXX)

# Make a single-config generator (Unix Makefiles/Ninja) produce optimized
# artifacts when the caller did not explicitly select a configuration.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Package the upstream exact implementation as a concrete static archive.
add_library(geodesic_distance STATIC
    src/geodesic_distance.cpp
    src/geodesic_distance_api.h)
add_library(geodesic::distance ALIAS geodesic_distance)

target_include_directories(geodesic_distance PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_compile_features(geodesic_distance PUBLIC cxx_std_14)

# MSVC 19.4x hides std::auto_ptr unless this legacy compatibility switch is set.
if(MSVC)
    target_compile_definitions(geodesic_distance PUBLIC _HAS_AUTO_PTR_ETC=1)
endif()

option(GEODESIC_DISTANCE_BUILD_EXAMPLES "Build the command-line examples" ON)
if(GEODESIC_DISTANCE_BUILD_EXAMPLES)
    add_executable(geodesic_example0 src/example0.cpp)
    target_link_libraries(geodesic_example0 PRIVATE geodesic::distance)

    add_executable(geodesic_example1 src/example1.cpp)
    target_link_libraries(geodesic_example1 PRIVATE geodesic::distance)

    add_executable(ply_exact_random_pairs examples/ply_exact_random_pairs.cpp)
    target_link_libraries(ply_exact_random_pairs PRIVATE geodesic::distance)

    add_executable(ply_multisource_distance_field examples/ply_multisource_distance_field.cpp)
    target_link_libraries(ply_multisource_distance_field PRIVATE geodesic::distance)
endif()

# Hessian VIPSS usage:
#   add_subdirectory(path/to/geodesic_distance)
#   target_link_libraries(hessian_vipss_target PRIVATE geodesic::distance)
