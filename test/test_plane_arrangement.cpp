#include <gtest_recons/gtest.h>
#include "plane_arrangement_fixture.h"

using namespace std;
using Json = nlohmann::json;

PlaneArrangementFixture::PlaneArrangementFixture()
{

}

PlaneArrangementFixture::~PlaneArrangementFixture()
{

}

/*
 * Setting up a simple plane arrangement with only 2 planes
 * */
void PlaneArrangementFixture::SetUp()
{
    //Make 2 test planes
    PrimitiveSet myPrimitiveSet;

    // Test plane 1 : good orientation
    Json testPlane1;
    testPlane1["inlier"] = {0., 0.5, 0.};
    testPlane1["normal"] = {0., 1., 0.};
    myPrimitiveSet.insert(testPlane1);

    // Test plane 2 : wrong orientation
    Json testPlane2;
    testPlane2["inlier"] = {0.5, 0., 0.};
    testPlane2["normal"] = {1., 0., 0.};
    myPrimitiveSet.insert(testPlane2);

    //Making a test bounding box
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    myPlaneArrangement = new PlaneArrangement(bbox, myPrimitiveSet);
    /* A bounding box creates 26 infinite cells + 1 cell which is
     * the bounding box itself*/
    /* After plane insertion, only the finite cells are cut */
    ASSERT_EQ(myPlaneArrangement->number_of_cells(), 30);
}

void PlaneArrangementFixture::TearDown()
{
    if(myPlaneArrangement) delete myPlaneArrangement;
}

Triangle getTriangleFromFacetIterator(Polyhedron::Facet_iterator it)
{
    vector<Point> trianglesPoint;
    auto j = it->facet_begin();
    do
    {
        trianglesPoint.push_back(j->vertex()->point());
    } while(++j != it->facet_begin());
    assert(trianglesPoint.size() == 3);
    return Triangle(trianglesPoint[0], trianglesPoint[1], trianglesPoint[2]);

}

bool areTrianglesFacetNeighbours(const Triangle& t1, const Triangle& t2)
{
    // Check that the two triangles share 2 vertices
    int nbOfSharedVertices = 0;
    for(int i=0; i < 3; i++)
        for(int j=0; j< 3; j++)
            if(t1[i] == t2[j]) nbOfSharedVertices++;
    if(nbOfSharedVertices != 2) return false;

    // Check that the two triangles are coplanar
    return(t1.supporting_plane() == t2.supporting_plane());

}

TEST(PlaneArrangementCustom, PolyhedronLabeling)
{
    /* When we want to perform intersections between a primitive and the plane arrangement,
     * we triangulate the plane arrangement and we give each triangle an id corresponding to the original polygonal
     * facet it comes from. This test makes sure the id assignment is cogent.*/


    //Make 2 test planes
    PrimitiveSet myPrimitiveSet;

    // Test plane 1 : good orientation
    Json testPlane1;
    testPlane1["inlier"] = {0., 0.5, 0.};
    testPlane1["normal"] = {0., 1., 0.};
    myPrimitiveSet.insert(testPlane1);

    // Test plane 2 : wrong orientation
    Json testPlane2;
    testPlane2["inlier"] = {0.5, 0., 0.};
    testPlane2["normal"] = {1., 0., 0.};
    myPrimitiveSet.insert(testPlane2);

    // Test plane 3 : in order to have facets with 5 vertice
    Json testPlane3;
    testPlane3["inlier"] = {0.75, 0.5, 0.};
    testPlane3["normal"] = {-1./sqrt(2), -1./sqrt(2), 0.};
    myPrimitiveSet.insert(testPlane3);

    //Making a test bounding box
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    PlaneArrangement myPlaneArrangement(bbox, myPrimitiveSet);

    // Retreive all triangles per facet
    Polyhedron polyhedron = myPlaneArrangement.getPolyhedron();
    map<int, vector<Polyhedron::Facet_iterator>> facetIdToTriangles;
    for(Polyhedron::Facet_iterator facet_it = polyhedron.facets_begin();
        facet_it != polyhedron.facets_end(); ++facet_it) {
        if ( facetIdToTriangles.find(facet_it->id()) == facetIdToTriangles.end() ) {
            // not found
            facetIdToTriangles[facet_it->id()] = {facet_it};
        } else {
            // found
            facetIdToTriangles[facet_it->id()].push_back(facet_it);
        }
    }

    /* Check that each triangle in a facet shares an edge with at least one other triangle from the facet
    * and is coplanar with it*/
    for(auto faceIt = facetIdToTriangles.begin(); faceIt != facetIdToTriangles.end(); faceIt++)
    {
        vector<Polyhedron::Facet_iterator>& triangles = faceIt->second;
        if(triangles.size() == 1) continue; // A facet with only 1 triangle is valid
        bool isTriangleChecked = false;
        for(size_t i=0; i < triangles.size(); i++)
        {
            Triangle curTriangle = getTriangleFromFacetIterator(triangles[i]);

            // Check that curTriangle is neighbour to at least one other triangle
            for(size_t j=0; j < triangles.size(); j++)
            {
                if(i == j) continue;
                Triangle compTriangle = getTriangleFromFacetIterator(triangles[j]);
                if(areTrianglesFacetNeighbours(curTriangle, compTriangle))
                {
                    ASSERT_NE(i, j);
                    isTriangleChecked = true;
                    break;
                }
            }
            ASSERT_TRUE(isTriangleChecked);
        }
    }
}

TEST_F(PlaneArrangementFixture, AABBTreeSegmentIntersection)
{
    // Construct a segment query
    Point a(0.25, 0.25, 0.25);
    Point b(0.75, 0.75, 0.25);
    Segment segment_query(a,b);

    IntersectionsOutput intersections = myPlaneArrangement->intersectWithSegment(segment_query);

    // There should be no segment intersection
    ASSERT_EQ(intersections.segments.size(), (size_t)0);

    // There should be 1 point intersection at the center of the bounding box
    ASSERT_EQ(intersections.points.size(), (size_t)1);
}

TEST_F(PlaneArrangementFixture, AABBTreeSegmentIntersectionOnBoundary)
{
    // Construct a segment query
    Point a(0.25, 0.5, 0.25);
    Point b(0.75, 0.5, 0.25);
    Segment segment_query(a,b);

    IntersectionsOutput intersections = myPlaneArrangement->intersectWithSegment(segment_query);
    
    // There should be exactly 2 segments which intersect the arrangement facets
    ASSERT_EQ(intersections.segments.size(), (size_t)2);

    // There should be 1 point intersection
    ASSERT_EQ(intersections.points.size(), (size_t)1);
    Point testPoint(0.5, 0.5, 0.25);
    ASSERT_EQ(intersections.points[0], testPoint);

    // Construct a segment query
    a = Point(0.25, 0.75, 0.5);
    b = Point(0.25, 0.25, 0.5);
    segment_query = Segment(a,b);

    intersections = myPlaneArrangement->intersectWithSegment(segment_query);

    // There should be exactly 1 intersection point which intersect the arrangement facets
    ASSERT_EQ(intersections.points.size(), (size_t)1);

    // There should be no intersection segment
    ASSERT_EQ(intersections.segments.size(), (size_t)0);

    // Construct a segment query
    a = Point(0.2, 0.5, 0.5);
    b = Point(0.4, 0.5, 0.5);
    segment_query = Segment(a,b);

    intersections = myPlaneArrangement->intersectWithSegment(segment_query);

    // There should be exactly 0 intersection point which intersects the arrangement facets
    ASSERT_EQ(intersections.points.size(), (size_t)0);

    // There should be exactly 1 intersection segment which intersects the arrangement facets
    ASSERT_EQ(intersections.segments.size(), (size_t)1);
}

TEST(PlaneArrangementCustom, AABBTreeSegmentIntersectionEndPointOnBoundary)
{

    //Make 2 test planes
    PrimitiveSet myPrimitiveSet;

    // Test plane 1 : good orientation
    Json testPlane1;
    testPlane1["inlier"] = {0., 0.5, 0.};
    testPlane1["normal"] = {0., 1., 0.};
    myPrimitiveSet.insert(testPlane1);

    // Test plane 2 : wrong orientation
    Json testPlane2;
    testPlane2["inlier"] = {0.5, 0., 0.};
    testPlane2["normal"] = {1., 0., 0.};
    myPrimitiveSet.insert(testPlane2);

    // Test plane 3
    Json testPlane3;
    testPlane3["inlier"] = {0.25, 0.5, 0.};
    testPlane3["normal"] = {1., 0., 0.};
    myPrimitiveSet.insert(testPlane3);

    //Making a test bounding box
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    PlaneArrangement myPlaneArrangement(bbox, myPrimitiveSet);

    // Construct a segment query
    Point a(0.25, 0.5, 0.25);
    Point b(0.75, 0.5, 0.25);
    Segment segment_query(a,b);

    IntersectionsOutput intersections = myPlaneArrangement.intersectWithSegment(segment_query);

    // There should be exactly 2 segments which intersect the arrangement facets
    ASSERT_EQ(intersections.segments.size(), (size_t)2);

    // There should be exactly 2 point intersections with the arrangement facets
    ASSERT_EQ(intersections.points.size(), (size_t)2);

    // Construct a segment query
    a = Point(0.2, 0.5, 0.25);
    b = Point(0.75, 0.5, 0.25);
    segment_query = Segment(a,b);

    intersections = myPlaneArrangement.intersectWithSegment(segment_query);

    // There should be exactly 2 segments which intersect the arrangement facets
    ASSERT_EQ(intersections.segments.size(), (size_t)3);

    ASSERT_EQ(intersections.points.size(), (size_t)2);
}

/* In the case of triangles, point intersections are meaningless */
TEST_F(PlaneArrangementFixture, AABBTreeTriangleIntersection)
{
    // Construct a triangle query
    Point a(0.25, 0.25, 0.5);
    Point b(0.25, 0.75, 0.5);
    Point c(0.75, 0.5, 0.5);
    Triangle triangle(a, b, c);

    IntersectionsOutput intersections = myPlaneArrangement->intersectWithTriangle(triangle);
    ASSERT_EQ(intersections.segments.size(), (size_t)4);
}

TEST_F(PlaneArrangementFixture, AABBTreeTriangleIntersectionOnBoundary)
{
    // Construct a triangle query
    Point a(0.5, 0.25, 0.25);
    Point b(0.5, 0.75, 0.25);
    Point c(0.75, 0.5, 0.25);
    Triangle triangle(a, b, c);

    IntersectionsOutput intersections = myPlaneArrangement->intersectWithTriangle(triangle);
    ASSERT_EQ(intersections.segments.size(), (size_t)3);
}

TEST_F(PlaneArrangementFixture, AABBTreeTriangleIntersectionWithEdge)
{
    //Contruct the triangle query
    Point a(0.1, 0.1, 0.1);
    Point b(0.1, 0.1, 0.4);
    Point c(0.7, 0.7, 0.4);
    Triangle triangle(a, b, c);

    IntersectionsOutput intersections = myPlaneArrangement->intersectWithTriangle(triangle);
    ASSERT_EQ(intersections.segments.size(), (size_t)1);
    ASSERT_TRUE(intersections.segments[0].neighbourCells.hasFourCells);
}

TEST(PlaneArrangementCustom, IntersectionWithProjOnTwoPlane)
{
    //Make 2 test planes
    PrimitiveSet myPrimitiveSet;

    // Test plane 1
    Json testPlane1;
    testPlane1["inlier"] = {0., 0.5, 0.};
    testPlane1["normal"] = {0., 1., 0.};
    myPrimitiveSet.insert(testPlane1);

    // Test plane 2
    Json testPlane2;
    testPlane2["inlier"] = {0.5, 0., 0.};
    testPlane2["normal"] = {1., 0., 0.};
    myPrimitiveSet.insert(testPlane2);

    // Test plane 3
    Json testPlane3;
    testPlane3["inlier"] = {0.5, 0.5, 0.5};
    testPlane3["normal"] = {0., 0., 1.};
    myPrimitiveSet.insert(testPlane3);

    //Making a test bounding box
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    PlaneArrangement myPlaneArrangement(bbox, myPrimitiveSet);

    // Test line
    Point e1(0.25, 0.25, 0.75);
    Point e2(0.25, 0.75, 0.25);
    Segment test_segment(e1, e2);

    // Project on the intersection of the 2 first primitives
    Segment projSegment = projectSegmentOnPlanesIntersection(test_segment,
            myPlaneArrangement.plane(myPlaneArrangement.getPlaneHandleFromPrimitiveId(0)),
            myPlaneArrangement.plane(myPlaneArrangement.getPlaneHandleFromPrimitiveId(1)));

    // Intersect with the plane arrangement
    IntersectionsOutput intersections = myPlaneArrangement.intersectWithSegment(projSegment);

    ASSERT_EQ(intersections.points.size(), (size_t)1);
    ASSERT_EQ(intersections.segments.size(), (size_t)2);
    ASSERT_TRUE(intersections.segments[0].neighbourCells.hasFourCells);
    ASSERT_TRUE(intersections.segments[1].neighbourCells.hasFourCells);

}

TEST(PlaneArrangementCustom, mergeSegments)
{
    /* Test that 2 occurences of the same segment with the same facet ID are correctly merged */
    OrientedCellsHandles dummyNeighbours = {true, -1, -1, {-1, -1}, -1};
    SegmentWithNeighbourCells a = {Segment(Point(0.5, 0.5, 0.5), Point(0.375, 0.375, 0.5)),  dummyNeighbours};
    SegmentWithNeighbourCells b = {Segment(Point(0.5, 0.5, 0.5), Point(0.375, 0.375, 0.5)),  dummyNeighbours};
    int facetId = 80;

    vector<SegmentWithNeighbourCells> segments = {a, b};
    vector<int> segmentIds = {facetId, facetId};

    //Making a test bounding box
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);

    // Making an empty primitive set
    PrimitiveSet myPrimitiveSet;

    PlaneArrangement arr(bbox, myPrimitiveSet);

    arr.mergeSegments(segments, segmentIds);

    ASSERT_EQ(segments.size(), (size_t)1);
}

TEST_F(PlaneArrangementFixture, intersectedFacets)
{
    Segment querySegment(Point(0.5, 0.25, 0.25), Point(0.5, 0.25, 0.75));
    set<int> intersectedFacets = myPlaneArrangement->getListOfIntersectedFacets(querySegment);

    Point queryPoint(0.25, 0.25, 0.25);
    Polyhedral_complex_3::Arrangement_3<Kernel> &arrangementBase = *myPlaneArrangement;
    auto hf = find_containing_cell((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, queryPoint);
    auto facet = find_facet((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, myPlaneArrangement->cell(hf), myPlaneArrangement->getPlaneHandleFromPrimitiveId(1));
    ASSERT_EQ(intersectedFacets.size(), (size_t)1);
    ASSERT_EQ(int(facet), *intersectedFacets.begin());
}


TEST(PlaneArrangementCustom, SegmentWithNeighbourCellsEquality)
{
    OrientedCellsHandles dummyNeighbours = {true, -1, -1, {-1, -1}, -1};

    Point a(1., 0., 0.);
    Point b(0., 1., 0.);
    Point c(1., 0., 1.);

    SegmentWithNeighbourCells seg1 = {Segment(a, b), dummyNeighbours};
    SegmentWithNeighbourCells seg2 = {Segment(c, a), dummyNeighbours};

    ASSERT_FALSE(seg1 == seg2);
}
