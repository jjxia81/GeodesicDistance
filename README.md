# GeodesicDistance

A modern CMake-ready C++ library for exact and approximate geodesic distance and shortest-path computation on triangular meshes.

## Origin and attribution

This repository packages the original C++ implementation by Danil Kirsanov, with a minimal CMake integration and compatibility updates for modern MSVC. The original project is archived at [Google Code: geodesic](https://code.google.com/archive/p/geodesic/).

The exact method is based on:

> Joseph S. B. Mitchell, David M. Mount, and Christos H. Papadimitriou. “The Discrete Geodesic Problem.” *SIAM Journal on Computing*, 16(4), 647–668, 1987. [https://doi.org/10.1137/0216045](https://doi.org/10.1137/0216045)

## Use from CMake

```cmake
add_subdirectory(path/to/GeodesicDistance)
target_link_libraries(your_target PRIVATE geodesic::distance)
```

`geodesic::distance` is an interface target because the upstream implementation is header-implemented; client code receives the include path, C++14 requirement, and required MSVC compatibility definition without duplicate compiled definitions.

To build the upstream examples:

```powershell
cmake -S . -B build -DGEODESIC_DISTANCE_BUILD_EXAMPLES=ON
cmake --build build --config Release
```

## License

The upstream source is distributed under the MIT License; see [LICENSE](LICENSE).
