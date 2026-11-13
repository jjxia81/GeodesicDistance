// Exact multi-source distance field from points projected onto a triangular mesh.
// Usage: ply_multisource_distance_field mesh.ply source_points.ply output_prefix

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "geodesic_algorithm_exact.h"

struct Point { double x, y, z; };
struct Projection { unsigned face; Point point; double squared_distance; };

static Point operator+(Point a, Point b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
static Point operator-(Point a, Point b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
static Point operator*(Point a, double s) { return {a.x*s, a.y*s, a.z*s}; }
static double dot(Point a, Point b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static double norm2(Point a) { return dot(a, a); }

static Point closest_on_triangle(Point p, Point a, Point b, Point c)
{
    const Point ab=b-a, ac=c-a, ap=p-a;
    const double d1=dot(ab,ap), d2=dot(ac,ap);
    if (d1<=0 && d2<=0) return a;
    const Point bp=p-b; const double d3=dot(ab,bp), d4=dot(ac,bp);
    if (d3>=0 && d4<=d3) return b;
    const double vc=d1*d4-d3*d2;
    if (vc<=0 && d1>=0 && d3<=0) return a+ab*(d1/(d1-d3));
    const Point cp=p-c; const double d5=dot(ab,cp), d6=dot(ac,cp);
    if (d6>=0 && d5<=d6) return c;
    const double vb=d5*d2-d1*d6;
    if (vb<=0 && d2>=0 && d6<=0) return a+ac*(d2/(d2-d6));
    const double va=d3*d6-d5*d4;
    if (va<=0 && (d4-d3)>=0 && (d5-d6)>=0) {
        const Point bc=c-b;
        return b+bc*((d4-d3)/((d4-d3)+(d5-d6)));
    }
    const double inverse=1.0/(va+vb+vc);
    return a+ab*(vb*inverse)+ac*(vc*inverse);
}

static bool read_ascii_ply(const std::string& filename, std::vector<Point>& points,
                           std::vector<unsigned>* faces)
{
    std::ifstream input(filename);
    if (!input) return false;
    std::string line, element, value;
    std::size_t point_count=0, face_count=0;
    bool ascii=false;
    while (std::getline(input,line)) {
        std::istringstream record(line); std::string key; record >> key;
        if (key=="format") { record >> value; ascii=value=="ascii"; }
        else if (key=="element") { record >> element; if(element=="vertex") record>>point_count; else if(element=="face") record>>face_count; }
        else if (key=="end_header") break;
    }
    if (!ascii || point_count==0) return false;
    points.reserve(point_count);
    for (std::size_t i=0;i<point_count;++i) {
        Point p; if (!(input>>p.x>>p.y>>p.z)) return false; points.push_back(p); std::getline(input,line);
    }
    if (!faces) return true;
    faces->reserve(3*face_count);
    for (std::size_t i=0;i<face_count;++i) {
        unsigned count,a,b,c; if (!(input>>count>>a>>b>>c) || count!=3) return false;
        faces->push_back(a); faces->push_back(b); faces->push_back(c); std::getline(input,line);
    }
    return !faces->empty();
}

static std::vector<Projection> project_sources(geodesic::Mesh& mesh, const std::vector<Point>& sources)
{
    std::vector<Projection> projections; projections.reserve(sources.size());
    for (const Point& source : sources) {
        Projection best{0,{},std::numeric_limits<double>::infinity()};
        for (unsigned i=0;i<mesh.faces().size();++i) {
            geodesic::Face& face=mesh.faces()[i]; auto& v=face.adjacent_vertices();
            const Point a{v[0]->x(),v[0]->y(),v[0]->z()};
            const Point b{v[1]->x(),v[1]->y(),v[1]->z()};
            const Point c{v[2]->x(),v[2]->y(),v[2]->z()};
            const Point q=closest_on_triangle(source,a,b,c);
            const double d2=norm2(q-source);
            if (d2<best.squared_distance) best={i,q,d2};
        }
        projections.push_back(best);
    }
    return projections;
}

static bool write_projected_sources(const std::string& filename, const std::vector<Projection>& projections)
{
    std::ofstream output(filename); if (!output) return false;
    output << std::setprecision(12) << "ply\nformat ascii 1.0\nelement vertex " << projections.size() << "\n"
           << "property double x\nproperty double y\nproperty double z\nproperty int source_id\n"
           << "property int face_id\nproperty double projection_distance\nend_header\n";
    for (unsigned i=0;i<projections.size();++i) {
        const Projection& p=projections[i];
        output << p.point.x << ' ' << p.point.y << ' ' << p.point.z << ' ' << i << ' ' << p.face << ' ' << std::sqrt(p.squared_distance) << '\n';
    }
    return true;
}

static bool write_distance_field(const std::string& filename, geodesic::Mesh& mesh,
                                 geodesic::GeodesicAlgorithmExact& algorithm,
                                 const std::vector<unsigned>& faces)
{
    std::ofstream output(filename); if (!output) return false;
    output << std::setprecision(12) << "ply\nformat ascii 1.0\nelement vertex " << mesh.vertices().size() << "\n"
           << "property double x\nproperty double y\nproperty double z\nproperty double quality\nproperty int source_id\n"
           << "element face " << faces.size()/3 << "\nproperty list uchar int vertex_indices\nend_header\n";
    for (unsigned i=0;i<mesh.vertices().size();++i) {
        geodesic::SurfacePoint point(&mesh.vertices()[i]); double distance=0.0;
        const unsigned source_id=algorithm.best_source(point,distance);
        output << point.x() << ' ' << point.y() << ' ' << point.z() << ' ' << distance << ' ' << source_id << '\n';
    }
    for (unsigned i=0;i<faces.size();i+=3) output << "3 " << faces[i] << ' ' << faces[i+1] << ' ' << faces[i+2] << '\n';
    return true;
}

int main(int argc, char** argv)
{
    typedef std::chrono::steady_clock Clock;
    if (argc!=4) { std::cerr << "Usage: ply_multisource_distance_field mesh.ply source_points.ply output_prefix\n"; return 2; }
    const Clock::time_point total_start=Clock::now();
    std::vector<Point> mesh_points, source_points; std::vector<unsigned> faces;
    const Clock::time_point read_mesh_start=Clock::now();
    if (!read_ascii_ply(argv[1],mesh_points,&faces)) { std::cerr << "Expected an ASCII triangular mesh PLY.\n"; return 3; }
    const double read_mesh_seconds=std::chrono::duration<double>(Clock::now()-read_mesh_start).count();
    const Clock::time_point read_sources_start=Clock::now();
    if (!read_ascii_ply(argv[2],source_points,nullptr)) { std::cerr << "Expected an ASCII source-point PLY.\n"; return 4; }
    const double read_sources_seconds=std::chrono::duration<double>(Clock::now()-read_sources_start).count();

    std::vector<double> vertices; vertices.reserve(3*mesh_points.size());
    for (const Point& p:mesh_points) { vertices.push_back(p.x); vertices.push_back(p.y); vertices.push_back(p.z); }
    const Clock::time_point mesh_init_start=Clock::now();
    geodesic::Mesh mesh; mesh.initialize_mesh_data(vertices,faces);
    const double mesh_init_seconds=std::chrono::duration<double>(Clock::now()-mesh_init_start).count();
    const Clock::time_point projection_start=Clock::now();
    const std::vector<Projection> projections=project_sources(mesh,source_points);
    const double projection_seconds=std::chrono::duration<double>(Clock::now()-projection_start).count();

    std::vector<geodesic::SurfacePoint> sources;
    for (const Projection& p:projections) sources.emplace_back(&mesh.faces()[p.face],p.point.x,p.point.y,p.point.z);
    const std::string prefix(argv[3]);
    std::ofstream timing(prefix+"_timing.csv");
    if (!timing) { std::cerr << "Could not open timing CSV.\n"; return 5; }
    timing << std::setprecision(12) << "phase,seconds\nread_mesh," << read_mesh_seconds
           << "\nread_source_points," << read_sources_seconds << "\ninitialize_mesh," << mesh_init_seconds
           << "\nproject_sources," << projection_seconds << '\n';
    if (!write_projected_sources(prefix+"_projected_sources.ply",projections)) { std::cerr << "Could not write projected sources.\n"; return 6; }

    const Clock::time_point solver_start=Clock::now();
    geodesic::GeodesicAlgorithmExact algorithm(&mesh);
    timing << "construct_solver," << std::chrono::duration<double>(Clock::now()-solver_start).count() << '\n';
    const Clock::time_point propagation_start=Clock::now();
    algorithm.propagate(sources,geodesic::GEODESIC_INF);
    timing << "propagate_full_distance_field," << std::chrono::duration<double>(Clock::now()-propagation_start).count() << '\n';
    const Clock::time_point write_start=Clock::now();
    if (!write_distance_field(prefix+"_distance_field.ply",mesh,algorithm,faces)) { std::cerr << "Could not write distance field.\n"; return 7; }
    timing << "write_distance_field," << std::chrono::duration<double>(Clock::now()-write_start).count()
           << "\ntotal," << std::chrono::duration<double>(Clock::now()-total_start).count() << '\n';
    std::cout << "Projected " << sources.size() << " sources and wrote " << prefix << "_distance_field.ply\n";
    return 0;
}
