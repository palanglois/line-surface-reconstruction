#include <gtest_recons/gtest.h>
#include "plane_arrangement_weight_fixture.h"


using namespace std;
using Json = nlohmann::json;

PlaneArrangementWeightFixture::PlaneArrangementWeightFixture()
{

}

PlaneArrangementWeightFixture::~PlaneArrangementWeightFixture()
{

}

/*
 * Setting up a simple plane arrangement with only 2 planes
 * */
void PlaneArrangementWeightFixture::SetUp()
{

    //Make 2 test lines
    myLineSet = new LineSet();

    // Test line 1 (textural)
    Json testLine_1;
    testLine_1["pt1"] = {0.5, 0.25, 0.25};
    testLine_1["pt2"] = {0.5, 0.25, 0.75};
    testLine_1["plane_index"] = {1};
    testLine_1["pt_views"] = {{0.25, 0.25, 0.25}};
    myLineSet->insert(testLine_1);

    // Test line 2 (structural)
    Json testLine_2;
    testLine_2["pt1"] = {0.5, 0.45, 0.2};
    testLine_2["pt2"] = {0.5, 0.5, 0.75};
    testLine_2["plane_index"] = {0, 1};
    testLine_2["pt_views"] = {{0.25, 0.25, 0.25}};
    myLineSet->insert(testLine_2);

    //Make 3 test planes
    myPrimitiveSet = new PrimitiveSet();

    // Test plane 1
    Json testPlane1;
    testPlane1["inlier"] = {0., 0.5, 0.};
    testPlane1["normal"] = {0., 1., 0.};
    myPrimitiveSet->insert(testPlane1);

    // Test plane 2
    Json testPlane2;
    testPlane2["inlier"] = {0.5, 0., 0.};
    testPlane2["normal"] = {1., 0., 0.};
    myPrimitiveSet->insert(testPlane2);

    // Test plane 3
    Json testPlane3;
    testPlane3["inlier"] = {0., 0., 0.5};
    testPlane3["normal"] = {0., 0., 1.};
    myPrimitiveSet->insert(testPlane3);

    // Re-orientate primitives
    myPrimitiveSet->orientatePrimitives(*myLineSet);

    //Making a test bounding box
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    myPlaneArrangement = new PlaneArrangement(bbox, *myPrimitiveSet);
}

void PlaneArrangementWeightFixture::TearDown()
{
    if(myPlaneArrangement) delete myPlaneArrangement;
    if(myLineSet) delete myLineSet;
    if(myPrimitiveSet) delete myPrimitiveSet;
}

/*TEST_F(PlaneArrangementWeightFixture, testTexturalAndStructuralTerm)
{
    computeWeights(*myPlaneArrangement, *myLineSet, 1.);

    Point queryPoint(0.1, 0.1, 0.1);
    PlaneArrangement::Face_handle hf = find_containing_cell(*myPlaneArrangement, queryPoint);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_void_points, 0.25 + 0.3);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_full_points, 0.);

    queryPoint = Point(0.6, 0.1, 0.1);
    hf = find_containing_cell(*myPlaneArrangement, queryPoint);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_void_points, 0.);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_full_points, 0.25);

    queryPoint = Point(0.2, 0.7, 0.7);
    hf = find_containing_cell(*myPlaneArrangement, queryPoint);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_void_points, 0.);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_full_points, 0.);

    queryPoint = Point(0.2, 0.2, 0.7);
    hf = find_containing_cell(*myPlaneArrangement, queryPoint);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_void_points, 0.25 + 0.25);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_full_points, 0.);

    queryPoint = Point(0.7, 0.7, 0.2);
    hf = find_containing_cell(*myPlaneArrangement, queryPoint);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_void_points, 0.);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_full_points, 0.3);

    queryPoint = Point(0.2, 0.7, 0.2);
    hf = find_containing_cell(*myPlaneArrangement, queryPoint);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_void_points, 0.);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_full_points, 0.);

    queryPoint = Point(0.7, 0.2, 0.7);
    hf = find_containing_cell(*myPlaneArrangement, queryPoint);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_void_points, 0.);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_full_points, 0.25);

    queryPoint = Point(0.7, 0.7, 0.7);
    hf = find_containing_cell(*myPlaneArrangement, queryPoint);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_void_points, 0.);
    ASSERT_EQ(myPlaneArrangement->cell(hf).cell_value_full_points, 0.25);
}*/

TEST_F(PlaneArrangementWeightFixture, findStructuralCells)
{
    //Making a fake test segment for finding cells
    Segment testSegment(Point(0.5, 0.5, 0.2), Point(0.5, 0.5, 0.8));

    Point queryPoint(0.2, 0.2, 0.2);
    PlaneArrangement::Face_handle cell0 = find_containing_cell((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, queryPoint);
    queryPoint = Point(0.7, 0.2, 0.2);
    PlaneArrangement::Face_handle cell1 = find_containing_cell((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, queryPoint);
    queryPoint = Point(0.7, 0.7, 0.2);
    PlaneArrangement::Face_handle cell2 = find_containing_cell((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, queryPoint);
    queryPoint = Point(0.2, 0.7, 0.2);
    PlaneArrangement::Face_handle cell3 = find_containing_cell((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, queryPoint);

    SegmentWithNeighbourCells testSegCells = {testSegment, {true, cell0, cell1, {cell2, cell3}}};

    // First point of view test
    Point testPointOfView(0.1, 0.1, 0.1);

    TripletCell structTripletCells = findStructuralCells(testSegCells, testPointOfView, *myPlaneArrangement);

    ASSERT_NE(cell0, get<0>(structTripletCells));
    ASSERT_NE(cell0, get<1>(structTripletCells));
    ASSERT_NE(cell0, get<2>(structTripletCells));

    // Second point of view test
    testPointOfView = Point(0.7, 0.7, 0.7);
    structTripletCells = findStructuralCells(testSegCells, testPointOfView, *myPlaneArrangement);

    ASSERT_NE(cell2, get<0>(structTripletCells));
    ASSERT_NE(cell2, get<1>(structTripletCells));
    ASSERT_NE(cell2, get<2>(structTripletCells));

    // Third point of view test
    testPointOfView = Point(0.2, 0.7, -0.7);
    structTripletCells = findStructuralCells(testSegCells, testPointOfView, *myPlaneArrangement);

    ASSERT_NE(cell3, get<0>(structTripletCells));
    ASSERT_NE(cell3, get<1>(structTripletCells));
    ASSERT_NE(cell3, get<2>(structTripletCells));
}

TEST_F(PlaneArrangementWeightFixture, findFullTexturialCell)
{
    //Making a fake test segment for finding cells
    Segment testSegment(Point(0.5, 0.2, 0.2), Point(0.5, 0.2, 0.8));

    Point queryPoint(0.2, 0.2, 0.2);
    PlaneArrangement::Face_handle cell0 = find_containing_cell((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, queryPoint);
    queryPoint = Point(0.7, 0.2, 0.2);
    PlaneArrangement::Face_handle cell1 = find_containing_cell((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, queryPoint);

    SegmentWithNeighbourCells testSegCells = {testSegment, {false, cell0, cell1, {-1, -1}}};

    // First point of view test
    Point testPointOfView(0.1, 0.1, 0.1);
    int queryCell = findFullTexturialCell(testSegCells, testPointOfView, *myPlaneArrangement);

    ASSERT_EQ(queryCell, cell1);

    // Second point of view test
    testPointOfView = Point(0.7, 0.1, 0.1);
    queryCell = findFullTexturialCell(testSegCells, testPointOfView, *myPlaneArrangement);

    ASSERT_EQ(queryCell, cell0);

}

TEST(PlaneArrangementWeightCustom, testTexturalStructuralTerm2)
{

    //Make 2 test lines
    LineSet myLineSet;

    // Test line 1 (textural)
    Json testLine_1;
    testLine_1["pt1"] = {0.1001, 0.5, 0.997};
    testLine_1["pt2"] = {0.1985, 0.5, 0.2};
    testLine_1["plane_index"] = {1};
    testLine_1["pt_views"] = {{0.5, 0.1, 0.1}};
    myLineSet.insert(testLine_1);

    // Test line 2 (structural)
    Json testLine_2;
    testLine_2["pt1"] = {0.09, 0.902, 0.112};
    testLine_2["pt2"] = {0.801, 0.198, 0.75};
    testLine_2["plane_index"] = {0, 1};
    testLine_2["pt_views"] = {{0.5, 0.1, 0.1}};
    myLineSet.insert(testLine_2);

    //Make 3 test planes
    PrimitiveSet myPrimitiveSet;

    // Test plane 1
    Json testPlane1;
    testPlane1["inlier"] = {1., 0., 0.};
    testPlane1["normal"] = {0., 1./sqrt(2.), 1./sqrt(2.)};
    myPrimitiveSet.insert(testPlane1);

    // Test plane 2
    Json testPlane2;
    testPlane2["inlier"] = {1., 0., 1.};
    testPlane2["normal"] = {-1./sqrt(2.), 0., 1./sqrt(2.)};
    myPrimitiveSet.insert(testPlane2);

    // Re-orientate primitives
    myPrimitiveSet.orientatePrimitives(myLineSet);

    //Making a test bounding box
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    PlaneArrangement myPlaneArrangement(bbox, myPrimitiveSet);

    // Projection of line 2 on the intersection of the 2 first primitives and intersection with the plane arrangement

    // Getting the primitives and the segment to intersect
    const Plane& plane_0 = myPlaneArrangement.plane(myPlaneArrangement.getPlaneHandleFromPrimitiveId(0));
    const Plane& plane_1 = myPlaneArrangement.plane(myPlaneArrangement.getPlaneHandleFromPrimitiveId(1));
    const Segment& intersectLine = myLineSet[1].getSegment();

    Segment projSegment = projectSegmentOnPlanesIntersection(intersectLine, plane_0, plane_1);

    // Intersect with the plane arrangement
    IntersectionsOutput intersections = myPlaneArrangement.intersectWithSegment(projSegment);

    ASSERT_EQ(intersections.segments.size(), 1);

    // Projection of line 1 on plane 2 and intersection with the plane arrangement
    const Segment& intersectLine_2 = myLineSet[0].getSegment();
    projSegment = Segment(plane_1.projection(intersectLine_2.source()), plane_1.projection(intersectLine_2.target()));

    // Intersect with the plane arrangement
    intersections = myPlaneArrangement.intersectWithSegment(projSegment);

    ASSERT_EQ(intersections.segments.size(), 1);

}


TEST_F(PlaneArrangementWeightFixture, testVisibilityTerm)
{

    //Make 2 custom test lines
    LineSet myCustomLineSet;

    // Test line 1 (textural)
    Json testLine_1;
    testLine_1["pt1"] = {0.5, 0.25, 0.25};
    testLine_1["pt2"] = {0.5, 0.25, 0.75};
    testLine_1["plane_index"] = {1};
    testLine_1["pt_views"] = {{0.25, 0.75, 0.25}}; // So that there is a facet which is being intersected
    myCustomLineSet.insert(testLine_1);

    // Test line 2 (structural)
    Json testLine_2;
    testLine_2["pt1"] = {0.5, 0.45, 0.2};
    testLine_2["pt2"] = {0.5, 0.5, 0.75};
    testLine_2["plane_index"] = {0, 1};
    testLine_2["pt_views"] = {{0.25, 0.25, 0.25}};
    myCustomLineSet.insert(testLine_2);

    computeWeights(*myPlaneArrangement, myCustomLineSet, 1.);

    // Check that the facet in {y=0.5, x \in [0, 0.5], z\in [0, 0.5]} is intersected
    Point queryPoint(0.25, 0.25, 0.25);
    int queryPlane = 0;
    auto hf = find_containing_cell((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, queryPoint);
    auto facet = find_facet((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, myPlaneArrangement->cell(hf),
                            myPlaneArrangement->getPlaneHandleFromPrimitiveId(queryPlane));
    ASSERT_GT(myPlaneArrangement->facet(facet).facet_nbr_ray_vis_point, 0.);

    // Check that the facet in {x=0.5, y \in [0, 0.5], z\in [0, 0.5]} is intersected
    queryPoint = Point(0.25, 0.25, 0.25);
    queryPlane = 1;
    hf = find_containing_cell((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, queryPoint);
    facet = find_facet((Polyhedral_complex_3::Arrangement_3<Kernel>) *myPlaneArrangement, myPlaneArrangement->cell(hf),
                       myPlaneArrangement->getPlaneHandleFromPrimitiveId(queryPlane));
    ASSERT_EQ(myPlaneArrangement->facet(facet).facet_nbr_ray_vis_point, 0.);

}

TEST_F(PlaneArrangementWeightFixture, testMosek)
{
    // Compute weights
    computeWeights(*myPlaneArrangement, *myLineSet, 1.);

    // Make the optimisation
    double cost_visibility = 1.;
    double cost_area = 0.1;
    double cost_edge = 0.1;
    double cost_corners = 0.1;
    bool force_filled_bounding_box = true;
    bool force_empty_bounding_box = false;
    bool verbose = false;
    auto * mosekOptimizer = new ArrangementToMosek(*myPlaneArrangement,
            cost_visibility, cost_area, cost_edge, cost_corners, force_filled_bounding_box,
            force_empty_bounding_box, verbose);

    // Checking the nu computation
    for(auto facet_it = myPlaneArrangement->facets_begin(); facet_it != myPlaneArrangement->facets_end(); facet_it++)
    {
        int cell0 = facet_it->superface(0);
        int cell1 = facet_it->superface(1);
        if(!myPlaneArrangement->is_cell_bounded(myPlaneArrangement->cell(cell0)) ||
           !myPlaneArrangement->is_cell_bounded(myPlaneArrangement->cell(cell1))) continue;
        ASSERT_NE(mosekOptimizer->getNu(cell0), mosekOptimizer->getNu(cell1));
        ASSERT_EQ(mosekOptimizer->getNu(cell0)*mosekOptimizer->getNu(cell1), -1);
    }

    mosekOptimizer->setAndSolve();
    delete mosekOptimizer;

}