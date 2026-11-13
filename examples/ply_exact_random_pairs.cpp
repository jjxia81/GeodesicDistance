// Sample exact geodesics between random pairs drawn from a supplied vertex-ID set.
// Usage:
//   ply_exact_random_pairs mesh.ply vertex_ids.txt pair_count output_prefix [seed]

#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geodesic_algorithm_exact.h"

static bool read_ascii_triangle_ply(const std::string& filename,
                                    std::vector<double>& vertices,
                                    std::vector<unsigned>& faces)
{
    std::ifstream input(filename);
    if (!input) return false;

    std::string line, element, value;
    std::size_t vertex_count = 0, face_count = 0;
    bool ascii = false;
    while (std::getline(input, line))
    {
        std::istringstream record(line);
        std::string keyword;
        record >> keyword;
        if (keyword == "format") { record >> value; ascii = value == "ascii"; }
        else if (keyword == "element")
        {
            record >> element;
            if (element == "vertex") record >> vertex_count;
            else if (element == "face") record >> face_count;
        }
        else if (keyword == "end_header") break;
    }
    if (!ascii || vertex_count == 0 || face_count == 0) return false;

    vertices.reserve(3 * vertex_count);
    for (std::size_t i = 0; i < vertex_count; ++i)
    {
        double x, y, z;
        if (!(input >> x >> y >> z)) return false;
        vertices.push_back(x); vertices.push_back(y); vertices.push_back(z);
        std::getline(input, line); // Ignore optional normals/colors/properties.
    }

    faces.reserve(3 * face_count);
    for (std::size_t i = 0; i < face_count; ++i)
    {
        unsigned count, a, b, c;
        if (!(input >> count >> a >> b >> c) || count != 3) return false;
        faces.push_back(a); faces.push_back(b); faces.push_back(c);
        std::getline(input, line);
    }
    return true;
}

static bool read_vertex_ids(const std::string& filename, std::vector<unsigned>& ids)
{
    std::ifstream input(filename);
    if (!input) return false;
    std::string line;
    while (std::getline(input, line))
    {
        const std::size_t comment = line.find('#');
        std::istringstream record(line.substr(0, comment));
        unsigned id;
        if (record >> id) ids.push_back(id);
    }
    return !ids.empty();
}

int main(int argc, char** argv)
{
    typedef std::chrono::steady_clock Clock;
    const Clock::time_point total_start = Clock::now();
    if (argc != 5 && argc != 6)
    {
        std::cerr << "Usage: ply_exact_random_pairs mesh.ply vertex_ids.txt pair_count output_prefix [seed]\n";
        return 2;
    }

    const std::size_t requested_pairs = static_cast<std::size_t>(std::stoull(argv[3]));
    const unsigned seed = argc == 6 ? static_cast<unsigned>(std::stoul(argv[5])) : 12345u;
    std::vector<double> vertices;
    std::vector<unsigned> faces, vertex_ids;
    const Clock::time_point mesh_read_start = Clock::now();
    if (!read_ascii_triangle_ply(argv[1], vertices, faces))
    {
        std::cerr << "Expected an ASCII triangular PLY.\n";
        return 3;
    }
    const double mesh_read_seconds = std::chrono::duration<double>(Clock::now() - mesh_read_start).count();
    const Clock::time_point ids_read_start = Clock::now();
    if (!read_vertex_ids(argv[2], vertex_ids)) { std::cerr << "Expected a non-empty vertex-ID file.\n"; return 3; }
    const double ids_read_seconds = std::chrono::duration<double>(Clock::now() - ids_read_start).count();
    for (unsigned id : vertex_ids)
        if (id >= vertices.size() / 3) { std::cerr << "Vertex ID out of range: " << id << '\n'; return 4; }
    if (vertex_ids.size() < 2) { std::cerr << "At least two vertex IDs are required.\n"; return 5; }

    const std::size_t max_pairs = vertex_ids.size() * (vertex_ids.size() - 1) / 2;
    if (requested_pairs > max_pairs) { std::cerr << "Requested more unique pairs than possible.\n"; return 6; }

    const Clock::time_point pair_sampling_start = Clock::now();
    std::mt19937 generator(seed);
    std::uniform_int_distribution<std::size_t> choose(0, vertex_ids.size() - 1);
    std::set<std::pair<unsigned, unsigned>> pairs;
    while (pairs.size() < requested_pairs)
    {
        unsigned a = vertex_ids[choose(generator)], b = vertex_ids[choose(generator)];
        if (a == b) continue;
        if (a > b) std::swap(a, b);
        pairs.insert({a, b});
    }
    const double pair_sampling_seconds = std::chrono::duration<double>(Clock::now() - pair_sampling_start).count();

    const Clock::time_point mesh_initialization_start = Clock::now();
    geodesic::Mesh mesh;
    mesh.initialize_mesh_data(vertices, faces);
    const double mesh_initialization_seconds = std::chrono::duration<double>(Clock::now() - mesh_initialization_start).count();
    const std::string prefix(argv[4]);
    std::ofstream csv(prefix + ".csv"), obj(prefix + ".obj");
    if (!csv || !obj) { std::cerr << "Could not open output files.\n"; return 7; }

    std::set<unsigned> selected_ids;
    for (const auto& pair : pairs) { selected_ids.insert(pair.first); selected_ids.insert(pair.second); }
    std::ofstream selected_ply(prefix + "_selected_vertices.ply");
    if (!selected_ply) { std::cerr << "Could not open selected-vertices PLY.\n"; return 8; }
    std::ofstream timing(prefix + "_timing.csv");
    if (!timing) { std::cerr << "Could not open timing CSV.\n"; return 9; }
    timing << std::setprecision(12)
           << "phase,pair_index,source_vertex,target_vertex,seconds\n"
           << "read_mesh,-1,-1,-1," << mesh_read_seconds << '\n'
           << "read_vertex_ids,-1,-1,-1," << ids_read_seconds << '\n'
           << "sample_pairs,-1,-1,-1," << pair_sampling_seconds << '\n'
           << "initialize_mesh,-1,-1,-1," << mesh_initialization_seconds << '\n';
    selected_ply << "ply\nformat ascii 1.0\n"
                 << "element vertex " << selected_ids.size() << "\n"
                 << "property double x\nproperty double y\nproperty double z\n"
                 << "property uint vertex_id\nend_header\n"
                 << std::setprecision(12);
    for (unsigned id : selected_ids)
        selected_ply << vertices[3*id] << ' ' << vertices[3*id+1] << ' ' << vertices[3*id+2] << ' ' << id << '\n';

    csv << std::setprecision(12) << "source_vertex,target_vertex,exact_geodesic_distance,path_point_count\n";
    obj << std::setprecision(12) << "# Exact geodesic paths; one group per sampled vertex pair\n";

    unsigned obj_index = 1;
    std::size_t pair_index = 0;
    for (const auto& pair : pairs)
    {
        geodesic::SurfacePoint source(&mesh.vertices()[pair.first]);
        geodesic::SurfacePoint target(&mesh.vertices()[pair.second]);
        std::vector<geodesic::SurfacePoint> sources(1, source), stop_points(1, target), path;
        const Clock::time_point solver_construction_start = Clock::now();
        geodesic::GeodesicAlgorithmExact algorithm(&mesh);
        const double solver_construction_seconds = std::chrono::duration<double>(Clock::now() - solver_construction_start).count();
        const Clock::time_point propagation_start = Clock::now();
        algorithm.propagate(sources, geodesic::GEODESIC_INF, &stop_points);
        const double propagation_seconds = std::chrono::duration<double>(Clock::now() - propagation_start).count();
        const Clock::time_point trace_back_start = Clock::now();
        algorithm.trace_back(target, path);
        const double trace_back_seconds = std::chrono::duration<double>(Clock::now() - trace_back_start).count();

        const Clock::time_point distance_lookup_start = Clock::now();
        double distance = 0.0;
        algorithm.best_source(target, distance);
        const double distance_lookup_seconds = std::chrono::duration<double>(Clock::now() - distance_lookup_start).count();
        csv << pair.first << ',' << pair.second << ',' << distance << ',' << path.size() << '\n';
        timing << "construct_solver," << pair_index << ',' << pair.first << ',' << pair.second << ',' << solver_construction_seconds << '\n'
               << "propagate," << pair_index << ',' << pair.first << ',' << pair.second << ',' << propagation_seconds << '\n'
               << "trace_back," << pair_index << ',' << pair.first << ',' << pair.second << ',' << trace_back_seconds << '\n'
               << "distance_lookup," << pair_index << ',' << pair.first << ',' << pair.second << ',' << distance_lookup_seconds << '\n';

        obj << "g path_" << pair.first << "_to_" << pair.second << '\n';
        for (auto& point : path) obj << "v " << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
        // Explicit two-vertex segments have broad OBJ-viewer compatibility.
        for (unsigned i = 0; i + 1 < path.size(); ++i)
            obj << "l " << obj_index + i << ' ' << obj_index + i + 1 << '\n';
        obj_index += static_cast<unsigned>(path.size());
        std::cout << pair.first << " -> " << pair.second << ": " << distance
                  << " (propagation " << propagation_seconds << " s)\n";
        ++pair_index;
    }
    timing << "total,-1,-1,-1," << std::chrono::duration<double>(Clock::now() - total_start).count() << '\n';
    return 0;
}
