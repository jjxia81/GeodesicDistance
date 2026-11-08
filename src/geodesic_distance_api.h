#pragma once

#include <vector>

namespace geodesic_distance {

// Computes the exact geodesic distance between two zero-based mesh vertices.
// vertices contains xyz triples and faces contains zero-based triangle triples.
double exact_vertex_distance(const std::vector<double>& vertices,
                             const std::vector<unsigned>& faces,
                             unsigned source_vertex,
                             unsigned target_vertex);

} // namespace geodesic_distance
