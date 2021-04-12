#include <gtest_recons/gtest.h>
#include <PlaneArrangement.h>

using namespace std;
using Json = nlohmann::json;

TEST(Primitive, orthogonalProjection)
{
Point planeInlier(0., 0., 0.);
Vector planeNormal(1./sqrt(2.), 1./sqrt(2.), 0.);
Primitive testPlane(planeInlier, planeNormal);
Point queryPoint(5., 5., 0.);
Point projection = testPlane.orthogonalProjection(queryPoint);
EXPECT_NEAR(0., CGAL::to_double(projection[0]), 1e-10);
EXPECT_NEAR(0., CGAL::to_double(projection[1]), 1e-10);
EXPECT_NEAR(0., CGAL::to_double(projection[2]), 1e-10);
}

TEST(LineSet, lineInsertion)
{
//Insert first line
Json testLine_1;
testLine_1["pt1"] = {0., 0., 0.};
testLine_1["pt2"] = {1.5, -3.45, 6.05785};
testLine_1["plane_index"] = {0, 3};
testLine_1["pt_views"] = {{0.658, 0.4985, 0.654}, {0.4355, 0.4519, 0.1822}, {0.4866, 0.21, 0.6542}};
LineSet myLineSet;
myLineSet.insert(testLine_1);
ASSERT_EQ(myLineSet.size(), (size_t)1);
ASSERT_EQ(CGAL::to_double(myLineSet[0].getFirstPoint().x()), 0.);
ASSERT_EQ(CGAL::to_double(myLineSet[0].getSecondPoint().x()), 1.5);
ASSERT_EQ(myLineSet[0].getAssociatedPrimitives().size(), (size_t)2);
ASSERT_TRUE(myLineSet[0].isStructural());
ASSERT_EQ(myLineSet[0].getPointOfViews().size(), (size_t)3);
ASSERT_EQ(CGAL::to_double(myLineSet[0].getPointOfViews()[1].x()), 0.4355);

//Insert second line
Json testLine_2;
testLine_2["pt1"] = {1., 0.5, 0.3};
testLine_2["pt2"] = {2., -4., 7.};
testLine_2["plane_index"] = {1};
testLine_2["pt_views"] = {{0.4355, 0.4519, 0.1822}, {0.4866, 0.21, 0.6542}};
myLineSet.insert(testLine_2);
ASSERT_EQ(myLineSet.size(), (size_t)2);
ASSERT_FALSE(myLineSet[1].isStructural());
ASSERT_EQ(myLineSet.getBbox().xmin(), 0.);
ASSERT_EQ(myLineSet.getBbox().xmax(), 2.);
ASSERT_EQ(myLineSet.getBbox().ymin(), -4.);
ASSERT_EQ(myLineSet.getBbox().ymax(), 0.5);
ASSERT_EQ(myLineSet.getBbox().zmin(), 0.);
ASSERT_EQ(myLineSet.getBbox().zmax(), 7.);

}

TEST(PrimitiveSet, primitiveInsertion)
{
Json testPrimitive;
testPrimitive["inlier"] = {-1., 1., 0.};
testPrimitive["normal"] = {1./sqrt(2.), 1./sqrt(2.), 0.};
PrimitiveSet myPrimitiveSet;
myPrimitiveSet.insert(testPrimitive);
ASSERT_EQ(myPrimitiveSet.size(), (size_t)1);
Point queryPoint(5., 5., 0.);
Point projectedPoint = myPrimitiveSet[0].orthogonalProjection(queryPoint);
EXPECT_NEAR(0., CGAL::to_double(projectedPoint[0]), 1e-10);
EXPECT_NEAR(0., CGAL::to_double(projectedPoint[1]), 1e-10);
EXPECT_NEAR(0., CGAL::to_double(projectedPoint[2]), 1e-10);
ASSERT_EQ(CGAL::to_double(myPrimitiveSet[0].getInlier().x()), -1.);
ASSERT_EQ(CGAL::to_double(myPrimitiveSet[0].getNormal().y()), 1./sqrt(2.));

}

TEST(PrimitiveSet, orientatePlanes)
{
// Test Line
Json testLine;
testLine["pt1"] = {1., 3., 0.};
testLine["pt2"] = {3., 1., 0.};
testLine["plane_index"] = {0, 1};
testLine["pt_views"] = {{5., 5., 0.}};
LineSet myLineSet;
myLineSet.insert(testLine);

//Making test planes
PrimitiveSet myPrimitiveSet;

// Test plane 1 : good orientation
Json testPlane1;
testPlane1["inlier"] = {0., 0., 0.};
testPlane1["normal"] = {0., 1., 0.};
myPrimitiveSet.insert(testPlane1);

// Test plane 2 : wrong orientation
Json testPlane2;
testPlane2["inlier"] = {0., 0., 0.};
testPlane2["normal"] = {-1., 0., 0.};
myPrimitiveSet.insert(testPlane2);

int nbOfReorientedPlanes = myPrimitiveSet.orientatePrimitives(myLineSet);
ASSERT_EQ(nbOfReorientedPlanes, 1);
ASSERT_EQ(CGAL::to_double(myPrimitiveSet[1].getNormal().x()), 1.);
}
