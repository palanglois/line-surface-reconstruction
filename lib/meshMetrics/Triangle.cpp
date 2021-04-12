#include "Triangle.h"

using namespace std;
using namespace tinyply;
using namespace Eigen;

Triangle::Triangle() : a(Point(0., 0., 0.)), b(Point(0., 0., 0.)), c(Point(0., 0., 0.))
{

}

Triangle::Triangle(Point _a, Point _b, Point _c) : a(move(_a)), b(move(_b)), c(move(_c))
{

}

double Triangle::getArea() const
{
    return 0.5*(b-a).cross(c-a).norm();
}

const Point& Triangle::getA() const
{
    return a;
}

const Point& Triangle::getB() const
{
    return b;
}

const Point& Triangle::getC() const
{
    return c;
}

vector<Triangle> loadTrianglesFromPly(const string& path)
{
    try
    {
        //Load file
        ifstream ss(path, ios::binary);
        if (ss.fail()) throw runtime_error("failed to open " + path);

        PlyFile file;
        file.parse_header(ss);

        shared_ptr<PlyData> vertices, normals, faces, texcoords;

        try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
        catch (const exception & e) { cerr << "tinyply exception: " << e.what() << endl; }


        try { faces = file.request_properties_from_element("face", { "vertex_index" }); }
        catch (const exception & e) {

            try { faces = file.request_properties_from_element("face", { "vertex_indices" }); }
            catch (const exception & e) {
                cerr << "tinyply exception: " << e.what() << endl;
            }
        }

        file.read(ss);

        const size_t numVerticesBytes = vertices->buffer.size_bytes();
        vector<float3> verts(vertices->count);
        memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);

        const size_t numFacesBytes = faces->buffer.size_bytes();
        vector<uint3> facs(faces->count);
        memcpy(facs.data(), faces->buffer.get(), numFacesBytes);

        vector<Triangle> outTriangles;
        for (auto &faceCoord : facs) {
            Point a(verts[faceCoord.x].x, verts[faceCoord.x].y, verts[faceCoord.x].z);
            Point b(verts[faceCoord.y].x, verts[faceCoord.y].y, verts[faceCoord.y].z);
            Point c(verts[faceCoord.z].x, verts[faceCoord.z].y, verts[faceCoord.z].z);
            outTriangles.emplace_back(a, b, c);
        }
        return outTriangles;
    }
    catch (const exception & e)
    {
        cerr << "Caught tinyply exception: " << e.what() << endl;
        return vector<Triangle>(0);
    }
}

PointCloud samplePointsOnMesh(const vector<Triangle>& mesh, int nbSamples)
{
    // Build the cumulated areas histogram
    vector<double> cumulatedAreas;
    double accum(0);
    for(const auto triangle: mesh)
    {
        accum += triangle.getArea();
        cumulatedAreas.push_back(accum);
    }
    // Normalize it
    for(double &cumulatedArea: cumulatedAreas)
        cumulatedArea /= accum;
    // Actual sampling
    PointCloud sampledPoints(nbSamples, 3);
#pragma omp parallel for
    for(int i=0; i < nbSamples; i++)
    {
        // Select a random triangle according to the areas distribution
        double r = ((double) rand() / (RAND_MAX));
        size_t found_index = 0;
        for(size_t j=0; j<cumulatedAreas.size() && r > cumulatedAreas[j]; j++)
            found_index = j+1;

        // Draw a random point in this triangle
        double r1 = ((double) rand() / (RAND_MAX));
        double r2 = ((double) rand() / (RAND_MAX));
        Point A = mesh[found_index].getA();
        Point B = mesh[found_index].getB();
        Point C = mesh[found_index].getC();
        Point P = (1 - sqrt(r1)) * A + (sqrt(r1) * (1 - r2)) * B + (sqrt(r1) * r2) * C;
#pragma omp critical
        sampledPoints.row(i) = P;
    }
    return sampledPoints;
}

multiset<double> findPcDistance(const PointCloud& refPointCloud, const PointCloud& queryPointCloud)
{
    // Build the kd-tree for the reference Point Cloud
    const int dim = 3;
    const int maxLeaf = 10;
    EigenKdTree refTree(dim, cref(refPointCloud), maxLeaf);

    // do a knn search
    const size_t num_results = 1;
    multiset<double> allDistances;
#pragma omp parallel for
    for(int i=0; i < queryPointCloud.rows(); i++)
    {
        vector<size_t> ret_indexes(num_results);
        vector<double> out_dists_sqr(num_results);
        nanoflann::KNNResultSet<double> resultSet(num_results);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
        Point queryPoint = queryPointCloud.row(i);
        refTree.index->findNeighbors(resultSet, &queryPoint[0],
                                       nanoflann::SearchParams(10));
#pragma omp critical
        allDistances.insert(sqrt(out_dists_sqr[0]));
    }
    return allDistances;
}