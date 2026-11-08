// Sample exact geodesics between random pairs drawn from a supplied vertex-ID set.
// Usage:
//   ply_exact_random_pairs mesh.ply vertex_ids.txt pair_count output_prefix [seed]

#include <fstream>
#include <iomanip>
#include <iostream>
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
    if (argc != 5 && argc != 6)
    {
        std::cerr << "Usage: ply_exact_random_pairs mesh.ply vertex_ids.txt pair_count output_prefix [seed]\n";
        return 2;
    }

    const std::size_t requested_pairs = static_cast<std::size_t>(std::stoull(argv[3]));
    const unsigned seed = argc == 6 ? static_cast<unsigned>(std::stoul(argv[5])) : 12345u;
    std::vector<double> vertices;
    std::vector<unsigned> faces, vertex_ids;
    if (!read_ascii_triangle_ply(argv[1], vertices, faces) || !read_vertex_ids(argv[2], vertex_ids))
    {
        std::cerr << "Expected an ASCII triangular PLY and a non-empty vertex-ID file.\n";
        return 3;
    }
    for (unsigned id : vertex_ids)
        if (id >= vertices.size() / 3) { std::cerr << "Vertex ID out of range: " << id << '\n'; return 4; }
    if (vertex_ids.size() < 2) { std::cerr << "At least two vertex IDs are required.\n"; return 5; }

    const std::size_t max_pairs = vertex_ids.size() * (vertex_ids.size() - 1) / 2;
    if (requested_pairs > max_pairs) { std::cerr << "Requested more unique pairs than possible.\n"; return 6; }

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

    geodesic::Mesh mesh;
    mesh.initialize_mesh_data(vertices, faces);
    const std::string prefix(argv[4]);
    std::ofstream csv(prefix + ".csv"), obj(prefix + ".obj");
    if (!csv || !obj) { std::cerr << "Could not open output files.\n"; return 7; }

    std::set<unsigned> selected_ids;
    for (const auto& pair : pairs) { selected_ids.insert(pair.first); selected_ids.insert(pair.second); }
    std::ofstream selected_ply(prefix + "_selected_vertices.ply");
    if (!selected_ply) { std::cerr << "Could not open selected-vertices PLY.\n"; return 8; }
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
    for (const auto& pair : pairs)
    {
        geodesic::SurfacePoint source(&mesh.vertices()[pair.first]);
        geodesic::SurfacePoint target(&mesh.vertices()[pair.second]);
        std::vector<geodesic::SurfacePoint> sources(1, source), stop_points(1, target), path;
        geodesic::GeodesicAlgorithmExact algorithm(&mesh);
        algorithm.propagate(sources, geodesic::GEODESIC_INF, &stop_points);
        algorithm.trace_back(target, path);

        double distance = 0.0;
        algorithm.best_source(target, distance);
        csv << pair.first << ',' << pair.second << ',' << distance << ',' << path.size() << '\n';

        obj << "g path_" << pair.first << "_to_" << pair.second << '\n';
        for (auto& point : path) obj << "v " << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
        // Explicit two-vertex segments have broad OBJ-viewer compatibility.
        for (unsigned i = 0; i + 1 < path.size(); ++i)
            obj << "l " << obj_index + i << ' ' << obj_index + i + 1 << '\n';
        obj_index += static_cast<unsigned>(path.size());
        std::cout << pair.first << " -> " << pair.second << ": " << distance << '\n';
    }
    return 0;
}
