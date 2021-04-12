#ifndef LINE_BASED_RECONS_REFACTO_PLANEARRANGEMENT_H
#define LINE_BASED_RECONS_REFACTO_PLANEARRANGEMENT_H

//STL
#include <unordered_map>
#include <cmath>
#include <tuple>
#include <fstream>

//CGAL
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Homogeneous_d.h>

//Polyhedral complex
#include "PolyhedralComplex/PlaneArrangementInterface.h"

//Other includes
#include <PrimitiveSet.h>
#include "PolyhedronBuilder.h"

struct PointHash
{
    std::size_t operator()(const Point& k) const
    {
        using std::size_t;
        using std::hash;
        using std::string;

        // Compute individual hash values for first,
        // second and third and combine them using XOR
        // and bit shifting:

        return ((hash<double>()(CGAL::to_double(k.x()))
                 ^ (hash<double>()(CGAL::to_double(k.y())) << 1)) >> 1)
               ^ (hash<double>()(CGAL::to_double(k.z())) << 1);
    }
};

struct OrientedCellsHandles
{
    bool hasFourCells; // If the intersection is on an edge of the plane arrangement, it has 4 neighbour cells
    int positive;
    int negative;
    int neutral[2];
    int facet; // The facet itself
};

struct SegmentWithNeighbourCells
{
    Segment segment;
    OrientedCellsHandles neighbourCells;

    bool operator==(const SegmentWithNeighbourCells& lhs) const
    {
        // Symmetric operator between source and target
        return (segment.source() == lhs.segment.source() && segment.target() == lhs.segment.target()) ||
                (segment.source() == lhs.segment.target() && segment.target() == lhs.segment.source());
    }
};

struct SegmentWithNeighbourCellsHash
{
    std::size_t operator()(const SegmentWithNeighbourCells& k) const
    {
        using std::size_t;
        using std::hash;
        using std::string;

        // Compute individual hash values for first,
        // second and third and combine them using XOR
        // and bit shifting:
        PointHash pointHash;
        return static_cast<size_t>(pointHash(k.segment.source()) + pointHash(k.segment.target()));
    }
};

struct IntersectionsOutput
{
    std::vector<SegmentWithNeighbourCells> segments;
    std::vector<Point> points;
};

typedef std::tuple<int, int, int> TripletCell;

/* This class derives from the Polyhedral complex implementation of the plane arrangement.
 * It contains specificities to our problem. */
class PlaneArrangement : public Polyhedral_complex_3::Arrangement_3<Kernel>
{
public:

    explicit PlaneArrangement(const CGAL::Bbox_3 &_bbox, const PrimitiveSet &_myPrimitiveSet, double _voxelSize = 0., bool _verbose = false);
    virtual ~PlaneArrangement();

    /* I/O methods */
    void saveArrangementAsPly(std::string fileName);

    // Typedefs for AABB trees
    typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> PrimitiveArr;
    typedef CGAL::AABB_traits<Kernel, PrimitiveArr> Traits;
    typedef CGAL::AABB_tree<Traits> Tree;

    // Typedefs for intersections with AABB trees
    typedef boost::optional<Tree::Intersection_and_primitive_id<Segment>::Type> Segment_intersection;
    typedef boost::optional<Tree::Intersection_and_primitive_id<Triangle>::Type> Triangle_intersection;

    //Intersections
    IntersectionsOutput intersectWithSegment(Segment segmentQuery) const;
    IntersectionsOutput intersectWithTriangle(Triangle triangleQuery) const;
    std::set<int> getListOfIntersectedFacets(Segment segmentQuery) const;
    void mergeSegments(std::vector<SegmentWithNeighbourCells>& segments, const std::vector<int>& segmentIds) const;

    const Polyhedron& getPolyhedron() const;
    int getPlaneHandleFromPrimitiveId(int i) const;
    void savePlyFromLabel(const std::string &filename, std::map<int, int> &fh_to_node,
                          const std::vector<size_t> &labels);
    void saveArrPlyFullCell(const std::string &filename);

    double facet_area(const PlaneArrangement::Face& f) const;

    //Setter for texturial costs
    void insertTexturialCost(std::pair<int, int> textCells, double cost);

    //Getter for texturial costs
    const std::map<std::pair<int, int>, double>& getTexturialCosts() const;

    //Setter for structural costs
    void insertStructuralCost(const TripletCell &structCells, double cost);

    //Getter for structural costs
    const std::map<TripletCell, double>& getStructuralCosts() const;

protected:

    /* Inserting the planes from a primitive set */
    void insertPlanes(const PrimitiveSet &myPrimitiveSet);

    template<class T, class U = std::hash<T>>
    void removeDuplicates(std::vector<T>& array) const;
    void removeDuplicatesSegment(std::vector<SegmentWithNeighbourCells>& array) const;
    template <class T>
    IntersectionsOutput postProcessIntersections(std::list<T> intersections, bool processPoints=true) const;
    void mergePoints(std::vector<Point>& points) const;
    void buildTree();

private:

    /* Insertion of the voxel planes */
    void insertVoxelPlanes();

    // Attributes
    const PrimitiveSet &myPrimitiveSet;
    const CGAL::Bbox_3 bbox;
    const double voxelSize;
    const bool verbose;
    std::map<int, int> plane_handle_to_prim;
    std::map<int, int> prim_to_plane_handle;
    std::map<int, OrientedCellsHandles> facetToCells;
    Tree* intersectionTree;
    Polyhedron* polyhedron;
    std::map<TripletCell, double> structuralCosts;
    std::map<std::pair<int, int>, double> texturialCosts;

};


#endif //LINE_BASED_RECONS_REFACTO_PLANEARRANGEMENT_H
