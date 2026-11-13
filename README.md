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

`geodesic::distance` is a static-library target. It publishes the upstream headers, C++14 requirement, and required MSVC compatibility definition to consumers.

For a compiled-library API that does not expose the header-implemented solver internals, include `geodesic_distance_api.h` and call:

```cpp
double distance = geodesic_distance::exact_vertex_distance(
    vertices, faces, source_vertex_id, target_vertex_id);
```

`vertices` must contain `x, y, z` triples and `faces` must contain zero-based triangle-index triples.

To build the upstream examples:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

For Unix Makefiles or Ninja, `cmake -S . -B build` defaults to a Release build; `cmake --build build` then builds `libgeodesic_distance.a` and all examples.

### Exact random vertex-pair example

`ply_exact_random_pairs` reads an ASCII triangular PLY, a text file of permitted
zero-based vertex IDs (one ID per line; `#` comments are allowed), and samples
unique random pairs from that set. It writes exact distances to CSV and each
traced path as explicit OBJ line segments:

```powershell
./build/Release/ply_exact_random_pairs mesh.ply vertex_ids.txt 20 results/random_paths 12345
```

The final argument is an optional random seed. This produces
`results/random_paths.csv`, `results/random_paths.obj`, and
`results/random_paths_selected_vertices.ply`, plus
`results/random_paths_timing.csv`. The timing CSV records mesh/ID loading,
pair sampling, mesh initialization, and per-pair solver construction,
propagation, traceback, and distance lookup durations. The PLY contains every unique
vertex selected as an endpoint of a sampled pair, with its original zero-based
`vertex_id` stored as a vertex property.

By default, the example writes a complete exact distance field for each sampled
source as `*_distance_field.ply`. Each PLY has a `quality`
property containing the geodesic distance at every mesh vertex:

```powershell
./build/Release/ply_exact_random_pairs mesh.ply vertex_ids.txt 2 results/random_paths 12345
```

Distance-field export performs full-mesh propagation for each pair, so it is
substantially slower and uses more memory than a target-limited path query.
Append `--no-save-distance-fields` to skip distance-field export and use
target-limited propagation instead.

### Exact multi-source field from a point PLY

`ply_multisource_distance_field` accepts an ASCII triangular mesh PLY and an
ASCII PLY containing source-point positions. It projects every input point to
its closest mesh triangle, constructs face-based `SurfacePoint` sources, and
performs one exact multi-source propagation with an infinite distance limit:

```powershell
./build/Release/ply_multisource_distance_field mesh.ply source_points.ply results/multisource
```

It writes `multisource_projected_sources.ply` (projected positions, source IDs,
face IDs, and projection distances), `multisource_distance_field.ply` (the
original mesh topology with `quality` = exact distance and nearest `source_id`
per vertex), and `multisource_timing.csv`.

The solver supports sources inside faces directly; it is not necessary to
retriangulate the mesh just to insert these points.

## License

The upstream source is distributed under the MIT License; see [LICENSE](LICENSE).
