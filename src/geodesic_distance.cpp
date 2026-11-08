#include "geodesic_distance_api.h"
#include "geodesic_algorithm_exact.h"

#include <stdexcept>

namespace geodesic_distance {

double exact_vertex_distance(const std::vector<double>& vertices,
                             const std::vector<unsigned>& faces,
                             unsigned source_vertex,
                             unsigned target_vertex)
{
    if (vertices.size() % 3 != 0 || faces.size() % 3 != 0)
        throw std::invalid_argument("vertices and faces must contain xyz/index triples");
    if (source_vertex >= vertices.size() / 3 || target_vertex >= vertices.size() / 3)
        throw std::out_of_range("source or target vertex index is out of range");

    geodesic::Mesh mesh;
    mesh.initialize_mesh_data(vertices, faces);
    geodesic::SurfacePoint source(&mesh.vertices()[source_vertex]);
    geodesic::SurfacePoint target(&mesh.vertices()[target_vertex]);
    std::vector<geodesic::SurfacePoint> sources(1, source), stop_points(1, target);
    geodesic::GeodesicAlgorithmExact algorithm(&mesh);
    algorithm.propagate(sources, geodesic::GEODESIC_INF, &stop_points);

    double distance = 0.0;
    algorithm.best_source(target, distance);
    return distance;
}

} // namespace geodesic_distance
