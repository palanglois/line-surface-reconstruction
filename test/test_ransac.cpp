#include <gtest_recons/gtest.h>
#include "lib/ransacOnLines/RANSAC.h"

using namespace std;

/* Tests the midPointSegment function */
TEST(geometricFunctions, midPointSegment)
{
    //First line
    Vec3d firstPoint_0 = Vec3d(0., 4., 1.);
    Vec3d secondPoint_0 = Vec3d(0., 3., 1.);
    LineRansac firstLine(firstPoint_0, secondPoint_0);

    //Second line
    Vec3d firstPoint_1 = Vec3d(5., 0., 0.);
    Vec3d secondPoint_1 = Vec3d(3., 0., 0.);
    LineRansac secondLine(firstPoint_1, secondPoint_1);

    Vec3d midPoint = getMidPointSegment(firstLine, secondLine);

    EXPECT_EQ(midPoint[0], 0.);
    EXPECT_EQ(midPoint[1], 0.);
    EXPECT_EQ(midPoint[2], 0.5);
}

/* Tests the distanceToPlane function in the case of no intersection
 * between the line and the plane*/
TEST(geometricFunctions, distanceToPlaneNoIntersection)
{
    // Test plane
    Vec3d planeNormal = Vec3d(1. / sqrt(2.), 1. / sqrt(2.), 0.);
    Vec3d planeInlier = Vec3d(0., 0., 0.);
    Plane plane(planeNormal, planeInlier);

    // Test line
    Vec3d firstPoint = Vec3d(sqrt(3./2.) - 1., sqrt(3./2.) + 1., 0.);
    Vec3d secondPoint = Vec3d(4., 0., 0.);
    LineRansac line(firstPoint, secondPoint);

    double dist = computeDistanceLineToPlane(line, plane);

    EXPECT_NEAR(dist, 1./2. * (sqrt(3.) + 2.*sqrt(2.)), 1e-11);
}

/* Tests the distanceToPlane function in the case of an intersection
 * between the line and the plane*/
TEST(geometricFunctions, distanceToPlaneWithIntersection)
{
    // Test plane
    Vec3d planeNormal = Vec3d(1. / sqrt(2.), 1. / sqrt(2.), 0.);
    Vec3d planeInlier = Vec3d(0., 0., 0.);
    Plane plane(planeNormal, planeInlier);

    // Test line
    Vec3d firstPoint = Vec3d(sqrt(3./2.) - 1., sqrt(3./2.) + 1., 0.);
    Vec3d secondPoint = Vec3d(0., -4., 0.);
    LineRansac line(firstPoint, secondPoint);

    double dist = computeDistanceLineToPlane(line, plane);

    EXPECT_NEAR(dist, 1./2. * (sqrt(3.) + 2.*sqrt(2.)), 1e-11);
}

/* Tests the distanceToIntersection function */
TEST(geometricFunctions, distanceToIntersection)
{
    //First test plane
    Vec3d firstPlaneNormal = Vec3d(1., 0., 0.);
    Vec3d firstPlaneInlier = Vec3d(0., 2., 3.);
    Plane firstPlane(firstPlaneNormal, firstPlaneInlier);

    //Second test plane
    Vec3d secondPlaneNormal = Vec3d(0., 0., 1.);
    Vec3d secondPlaneInlier = Vec3d(20., 3., 0.);
    Plane secondPlane(secondPlaneNormal, secondPlaneInlier);

    //Test line
    Vec3d firstPointA = Vec3d(3., 45., 0.);
    Vec3d firstPointB = Vec3d(5., 35., 0.);
    LineRansac firstLine(firstPointA, firstPointB);

    double firstDist = distanceToIntersection(firstLine, firstPlane, secondPlane);

    EXPECT_EQ(firstDist, 5.);

    //Other line
    Vec3d secondPointA = Vec3d(0., 445.5, 0.) + 4.68475*Vec3d(1./sqrt(2), 0., 1./sqrt(2));
    Vec3d secondPointB = Vec3d(0., 55.5, 0.) + 0.768*Vec3d(1./sqrt(2), 0., 1./sqrt(2));
    LineRansac secondLine(secondPointA, secondPointB);

    double secondDist = distanceToIntersection(secondLine, firstPlane, secondPlane);

    EXPECT_NEAR(secondDist, 4.68475, 1e-10);
}

/* Tests the fact that a line cannot be classified twice in the same plane */
TEST(Line, notClassifiedTwiceInSamePlane)
{
#ifndef NDEBUG
    // Test line
    Vec3d firstPoint = Vec3d(sqrt(3./2.) - 1., sqrt(3./2.) + 1., 0.);
    Vec3d secondPoint = Vec3d(4., 0., 0.);
    LineRansac line(firstPoint, secondPoint);

    line.classify(0);
    ASSERT_DEATH(line.classify(0), "Assertion.*failed.");
#endif
}

/*Tests the fact that a line cannot be classified more than twice overall*/
TEST(Line, notClassifiedMoreThanTwice)
{
#ifndef NDEBUG
    // Test line
    Vec3d firstPoint = Vec3d(sqrt(3./2.) - 1., sqrt(3./2.) + 1., 0.);
    Vec3d secondPoint = Vec3d(4., 0., 0.);
    LineRansac line(firstPoint, secondPoint);

    line.classify(0);
    line.classify(1);
    ASSERT_DEATH(line.classify(2), "Assertion.*failed.");
#endif
}

/*Test that a line has correctly been classified*/
TEST(Line, hasBeenClassified)
{
    // Test line
    Vec3d firstPoint = Vec3d(sqrt(3./2.) - 1., sqrt(3./2.) + 1., 0.);
    Vec3d secondPoint = Vec3d(4., 0., 0.);
    LineRansac line(firstPoint, secondPoint);

    ASSERT_FALSE(line.hasBeenClassifiedInPlane(0));
    line.classify(0);
    ASSERT_TRUE(line.hasBeenClassifiedInPlane(0));
    line.classify(1);
    ASSERT_TRUE(line.hasBeenClassifiedInPlane(0));
    ASSERT_TRUE(line.hasBeenClassifiedInPlane(1));
    ASSERT_FALSE(line.hasBeenClassifiedInPlane(2));
}

/* Tests the updateLists function */
TEST(RANSAC, updateLists)
{
    vector<LineRansac> testLines(10);
    vector<int> firstFakeInliers = {0, 2, 3, 4};
    vector<int> secondFakeInliers = {1, 5, 7};
    vector<int> thirdFakeInliers = {2, 8, 9};

    RANSAC myRansac(testLines, 1e-5, 0.2);

    //Add first set of fake inliers
    myRansac.updateLists(firstFakeInliers);
    vector<int> isNotClassifiedTwice = myRansac.getIsNotClassifiedTwice();
    vector<vector<int>> drawnWithoutPlaneI = myRansac.getDrawnWithoutPlaneI();
    vector<int> testNotClassifiedTwice = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    ASSERT_EQ(isNotClassifiedTwice, testNotClassifiedTwice);
    vector<int> testVecPlane0FirstIt = {1, 5, 6, 7, 8, 9};
    ASSERT_EQ(drawnWithoutPlaneI.size(), 1);
    ASSERT_EQ(drawnWithoutPlaneI[0], testVecPlane0FirstIt);

    //Add second set of fake inliers
    myRansac.updateLists(secondFakeInliers);
    isNotClassifiedTwice = myRansac.getIsNotClassifiedTwice();
    drawnWithoutPlaneI = myRansac.getDrawnWithoutPlaneI();
    ASSERT_EQ(isNotClassifiedTwice, testNotClassifiedTwice);
    ASSERT_EQ(drawnWithoutPlaneI.size(), 2);
    ASSERT_EQ(drawnWithoutPlaneI[0], testVecPlane0FirstIt);
    vector<int> testVecPlane1SecondIt = {0, 2, 3, 4, 6, 8, 9};
    ASSERT_EQ(drawnWithoutPlaneI[1], testVecPlane1SecondIt);

    //Add third set of fake inliers
    myRansac.updateLists(thirdFakeInliers);
    isNotClassifiedTwice = myRansac.getIsNotClassifiedTwice();
    drawnWithoutPlaneI = myRansac.getDrawnWithoutPlaneI();
    testNotClassifiedTwice = {0, 1, 3, 4, 5, 6, 7, 8, 9};
    ASSERT_EQ(drawnWithoutPlaneI.size(), 3);
    ASSERT_EQ(isNotClassifiedTwice, testNotClassifiedTwice);
    ASSERT_EQ(drawnWithoutPlaneI[0], testVecPlane0FirstIt);
    vector<int> testVecPlane1ThirdIt = {0, 3, 4, 6, 8, 9};
    ASSERT_EQ(drawnWithoutPlaneI[1], testVecPlane1ThirdIt);
}

/* Tests the inlier count function */
TEST(RANSAC, evaluateModel)
{
    vector<LineRansac> testLines;

    // Make inlier Lines
    testLines.emplace_back(Vec3d(2, 3, 0), Vec3d(2, 7, 0));
    testLines.emplace_back(Vec3d(5, 1, 0), Vec3d(4, 5, 0));
    testLines.emplace_back(Vec3d(6, 6, 0), Vec3d(-1, 8, 0));
    testLines.emplace_back(Vec3d(7, 1, 0), Vec3d(6, -2, 0));
    testLines.emplace_back(Vec3d(8, 5, 0), Vec3d(0, 6, 0));

    //Make outlier Lines
    testLines.emplace_back(Vec3d(7, 1, 1), Vec3d(6, -2, 1));
    testLines.emplace_back(Vec3d(8, 5, 1), Vec3d(0, 6, 1));

    //Make plane that fits the inlier Lines
    Plane plane(Vec3d(0., 0., 1.), Vec3d(0., 0., 0.));

    //Make the RANSAC object with a good threshold
    RANSAC myRansacFirst(testLines, 1e-5, 0.5, int(1e3));

    //Make the RANSAC object with a bad threshold
    RANSAC myRansacSecond(testLines, 1e-5, 1.5, int(1e3));

    //Evaluate the inliers
    vector<int> inliers(0);
    myRansacFirst.evaluateModel(plane, inliers);
    ASSERT_EQ(inliers.size(), 5);
    inliers.clear();
    myRansacSecond.evaluateModel(plane, inliers);
    ASSERT_EQ(inliers.size(), 7);
}

/* Tests the line configuration check according to a plane */
TEST(RANSAC, haveSameConfiguration)
{
    // Make a reference planeOne
    Plane planeOne(Vec3d(0., 0., 1.), Vec3d(0., 0., 0.));
    Plane planeTwo(Vec3d(0., 0., 1.), Vec3d(0., 0., 0.5));

    // Make a set of lines
    vector<LineRansac> testLines;

    // Lines on positive side of the planeOne and planeTwo
    testLines.emplace_back(Vec3d(4, -2, 1), Vec3d(4, 2, 1)); // 0
    testLines.emplace_back(Vec3d(-1, 0, 1), Vec3d(6, 4, 1)); // 1
    testLines.emplace_back(Vec3d(6, 5, 1), Vec3d(6, -2, 1)); // 2
    testLines.emplace_back(Vec3d(-1, 1, 1), Vec3d(4, 6, 1)); // 3
    testLines.emplace_back(Vec3d(0, -2, 1), Vec3d(4, 3, 1)); // 4

    // Lines on negative side of the planeOne and planeTwo
    testLines.emplace_back(Vec3d(3, 3, -1), Vec3d(7, 7, -1)); // 5
    testLines.emplace_back(Vec3d(1, 6, -1), Vec3d(6, 1, -1)); // 6
    testLines.emplace_back(Vec3d(7, 8, -1), Vec3d(4, 2, -1)); // 7

    // Lines on positive side of planeOne and negative side of PlaneTwo
    testLines.emplace_back(Vec3d(8, 4, 0.25), Vec3d(4, -2, 0.25)); // 8
    testLines.emplace_back(Vec3d(4, 4, 0.25), Vec3d(-1, 6, 0.25)); // 9

    RANSAC myRansac(testLines, 1e-5, 0.1, int(1e3));


    // Make two sets with same configuration on both planes
    vector<int> firstSet = {1, 6, 4};    // + - +
    ASSERT_TRUE(myRansac.haveSameConfiguration(firstSet, planeOne, firstSet, planeTwo));

    // Make two sets with different configurations on the two planes
    vector<int> secondSet = {1, 7, 2, 8};
    ASSERT_FALSE(myRansac.haveSameConfiguration(secondSet, planeOne, secondSet, planeTwo));

    // Check that two sets with different sizes are rejected
    ASSERT_FALSE(myRansac.haveSameConfiguration(firstSet, planeOne, secondSet, planeOne));

    // Check that two sets with same configuration on both planes but different indices are rejected
    vector<int> thirdSet = {3, 6, 4};    // + - +
    ASSERT_FALSE(myRansac.haveSameConfiguration(firstSet, planeOne, thirdSet, planeTwo));

}

/* Test the refitting procedure */
TEST(RANSAC, refitting)
{
    vector<LineRansac> testLines;

    // Make inlier Lines
    testLines.emplace_back(Vec3d(5, 54, -0.246), Vec3d(-49, 38, 0.216));
    testLines.emplace_back(Vec3d(25, 55, -0.471), Vec3d(-23, 54, -0.278));
    testLines.emplace_back(Vec3d(80, -1, -0.177), Vec3d(23, -84, 0.382));
    testLines.emplace_back(Vec3d(91, 98, 0.151), Vec3d(-3, 18, 0.374));
    testLines.emplace_back(Vec3d(-19, 5, 0.126), Vec3d(-83, 98, 0.482));
    testLines.emplace_back(Vec3d(53, -66, -0.212), Vec3d(-40, 2, -0.041));
    testLines.emplace_back(Vec3d(29, -61, 0.206), Vec3d(71, -7, 0.143));

    // Make outlier Lines
    testLines.emplace_back(Vec3d(81, -100, -1.30), Vec3d(85, -40, -0.857));
    testLines.emplace_back(Vec3d(15, 98, 0.77), Vec3d(19, 83, 1.417));

    vector<int> inliers = {0, 1, 2, 5};

    // Make test plane
    Plane plane(Vec3d(0., 0.43588989, 0.9), Vec3d(0., 0., 0.));

    // Make RANSAC object
    RANSAC myRansac(testLines, 1e-5, 0.5, int(1e3));

    // Test refitting
    myRansac.refitting(plane, inliers);

    // Normal should have been corrected
    ASSERT_NEAR(plane.getPlaneNormal()[2], 0.999, 1e-3);

    // Inlier should not be further than the RANSAC threshold
    ASSERT_NEAR(plane.getPlaneInlier()[2], 0., 0.5);
}

/* Tests the RANSAC algorithm in two simple cases */
TEST(RANSAC, extractOneAndTwoSimplePlanes)
{
    //Make 10 fake lines in plane OXY
    vector<LineRansac> testLines;
    for(int i = 0; i < 10; i++)
    {
        Vec3d firstPoint = Vec3d(double(rand()), double(rand()), 0.);
        Vec3d secondPoint = Vec3d(double(rand()), double(rand()), 0.);
        testLines.emplace_back(firstPoint, secondPoint);
    }

    //Check that RANSAC is able to recognize it
    RANSAC myRansac(testLines, 1e-5, 0.2, int(1e3));
    cout.setstate(ios_base::failbit);
    myRansac.extractAllPlanes();
    cout.clear();
    vector<Plane> detectedPlanes = myRansac.getDetectedPlanes();
    ASSERT_EQ(detectedPlanes.size(), 1);
    ASSERT_TRUE(detectedPlanes[0].getPlaneNormal().isApprox(Vec3d(0., 0., -1.)) ||
                detectedPlanes[0].getPlaneNormal().isApprox(Vec3d(0., 0., 1.)));

    //Add 10 fake lines in plane OXZ and 10 fake lines in OXY
    vector<LineRansac> testLinesSecond;
    for(int i = 0; i < 20; i++)
    {
        Vec3d firstPoint = i < 10 ? Vec3d(double(rand()), double(rand()), 0.)
                                  : Vec3d(double(rand()), 0., double(rand()));
        Vec3d secondPoint = i < 10 ? Vec3d(double(rand()), double(rand()), 0.)
                                   : Vec3d(double(rand()), 0., double(rand()));
        testLinesSecond.emplace_back(firstPoint, secondPoint);
    }
    //Add 1 fake line in axis OX
    Vec3d firstPoint = Vec3d(1., 0., 0.);
    Vec3d secondPoint = Vec3d(-1.5, 0., 0.);
    testLinesSecond.emplace_back(firstPoint, secondPoint);
    //Run RANSAC
    RANSAC myRansacSecond = RANSAC(testLinesSecond, 1e-5, 0.2, int(1e3));
    cout.setstate(ios_base::failbit);
    myRansacSecond.extractAllPlanes();
    cout.clear();
    vector<Plane> detectedPlanesSecond = myRansacSecond.getDetectedPlanes();
    //At least one structural line has to be detected
    ASSERT_GE(myRansacSecond.getNbOfStructuralLines(), 1);
    ASSERT_EQ(detectedPlanesSecond.size(), 2);
}

TEST(RANSAC, planesMergingProcedure)
{
    //Make 10 fake lines in plane OXY
    vector<LineRansac> testLines;
    for(int i = 0; i < 10; i++)
    {
        Vec3d firstPoint = Vec3d(double(rand()), double(rand()), 0.);
        Vec3d secondPoint = Vec3d(double(rand()), double(rand()), 0.);
        testLines.emplace_back(firstPoint, secondPoint);
    }
    //Make 10 fake lines in plane OYZ
    for(int i = 0; i < 10; i++)
    {
        Vec3d firstPoint = Vec3d(0., double(rand()), double(rand()));
        Vec3d secondPoint = Vec3d(0., double(rand()), double(rand()));
        testLines.emplace_back(firstPoint, secondPoint);
    }
    //Add one line on 0Y
    Vec3d firstPoint = Vec3d(0., double(rand()), 0.);
    Vec3d secondPoint = Vec3d(0., double(rand()), 0.);
    testLines.emplace_back(firstPoint, secondPoint);

    //Check that RANSAC is able to recognize it
    RANSAC myRansac(testLines, 1e-5, 0.2, int(1e3));
    cout.setstate(ios_base::failbit);
    myRansac.extractAllPlanes();
    cout.clear();
    vector<Plane> detectedPlanes = myRansac.getDetectedPlanes();
    ASSERT_EQ(detectedPlanes.size(), 2);

    // Try to merge the 2 planes
    list<Plane> planeList;
    for(auto &plane: detectedPlanes)
        planeList.push_back(plane);

    auto planeOne = planeList.begin();
    auto planeTwo = planeOne++;
    double score = acos(planeOne->getPlaneNormal().dot(planeTwo->getPlaneNormal()));
    PlanePair mergingPair = make_tuple(planeOne, planeTwo, score);
    auto mergedPlane = myRansac.mergePlanes(mergingPair, planeList);
    ASSERT_EQ(planeList.size(), 1);

    //Try to update the line inlier list
    vector<Plane> detectedPlanesAsVector(planeList.begin(), planeList.end());
    RANSAC::resetLinePlanesAttributionFromPlaneInlierLists(testLines, detectedPlanesAsVector);

    for(const auto &line: testLines)
        ASSERT_EQ(line.getNbTimesClassified(), 1);
}

TEST(RANSAC, planesRefinement)
{
    srand(0);
    //Make 10 fake lines in plane OXY
    vector<LineRansac> testLines;
    for(int i = 0; i < 10; i++)
    {
        double x1 = 100. * double(rand()) / double(RAND_MAX);
        double x2 = 100. * double(rand()) / double(RAND_MAX);
        Vec3d firstPoint = Vec3d(x1, 10.*double(rand()) / double(RAND_MAX), 0.);
        Vec3d secondPoint = Vec3d(x2, 10.*double(rand()) / double(RAND_MAX), 0.);
        testLines.emplace_back(firstPoint, secondPoint);
    }
    //Make 10 fake lines in plane OXY with rot(theta, (0, 0, 1))
    const double theta = TEN_DEGREES_IN_RADIANS - 5.*3.141592/180.;
    for(int i = 0; i < 10; i++)
    {
        double x1 = 2.4 + 3.9 * double(rand()) / double(RAND_MAX);
        double x2 = 2.4 + 3.9 * double(rand()) / double(RAND_MAX);
        Vec3d firstPoint = Vec3d(x1*cos(theta), 10.*double(rand()) / double(RAND_MAX), x1*sin(theta));
        Vec3d secondPoint = Vec3d(x2*cos(theta), 10.*double(rand()) / double(RAND_MAX), x2*sin(theta));
        testLines.emplace_back(firstPoint, secondPoint);
    }
    // Add 5 lines on OY to make mutual support strong
    for(int i=0; i < 5; i++)
    {
        Vec3d firstPoint = Vec3d(0., 10.*double(rand()) / double(RAND_MAX), 0.);
        Vec3d secondPoint = Vec3d(0., 10.*double(rand()) / double(RAND_MAX), 0.);
        testLines.emplace_back(firstPoint, secondPoint);
    }

    //Check that RANSAC is able to recognize the 2 planes
    RANSAC myRansac(testLines, 1e-5, 0.2, int(1e3), 1000, 0, 20);
    cout.setstate(ios_base::failbit);
    myRansac.extractAllPlanes();
    cout.clear();
    vector<Plane> detectedPlanes = myRansac.getDetectedPlanes();
    ASSERT_EQ(detectedPlanes.size(), 2);


    // Check that the 2 planes are correctly merged after the plane refinement procedure
    myRansac.refineExtractedPlanes();
    ASSERT_EQ(myRansac.getDetectedPlanes().size(), 1);

    // All the 25 lines should be inlier
    ASSERT_EQ(myRansac.getDetectedPlanes()[0].getInliers().size(), 25);
}

TEST(RANSAC, planesRefinementStrongMutualSupportHighAngle)
{
    srand(0);
    //Make 10 fake lines in plane OXY
    vector<LineRansac> testLines;
    for(int i = 0; i < 10; i++)
    {
        Vec3d firstPoint = Vec3d(double(rand()), double(rand()), 0.);
        Vec3d secondPoint = Vec3d(double(rand()), double(rand()), 0.);
        testLines.emplace_back(firstPoint, secondPoint);
    }
    //Make 10 fake lines in plane OXY with rot(theta, (0, 0, 1))
    const double theta = TEN_DEGREES_IN_RADIANS + 1.*3.141592/180.; // more than 10 degrees
    for(int i = 0; i < 10; i++)
    {
        double x1 = rand();
        double x2 = rand();
        Vec3d firstPoint = Vec3d(x1*cos(theta), double(rand()), x1*sin(theta));
        Vec3d secondPoint = Vec3d(x2*cos(theta), double(rand()), x2*sin(theta));
        testLines.emplace_back(firstPoint, secondPoint);
    }
    // Add 5 lines on OY to make mutual support strong
    for(int i=0; i < 5; i++)
    {
        Vec3d firstPoint = Vec3d(0., double(rand()), 0.);
        Vec3d secondPoint = Vec3d(0., double(rand()), 0.);
        testLines.emplace_back(firstPoint, secondPoint);
    }

    //Check that RANSAC is able to recognize the 2 planes
    RANSAC myRansac(testLines, 1e-5, 0.2, int(1e3));
    cout.setstate(ios_base::failbit);
    myRansac.extractAllPlanes();
    cout.clear();
    vector<Plane> detectedPlanes = myRansac.getDetectedPlanes();
    ASSERT_EQ(detectedPlanes.size(), 2);

    // Check that the 2 planes are not merged after the plane refinement procedure (angle is too big)
    myRansac.refineExtractedPlanes();
    ASSERT_EQ(myRansac.getDetectedPlanes().size(), 2);
}

TEST(RANSAC, planesRefinementWeakMutualSupport)
{
    srand(0);
    //Make 10 fake lines in plane OXY
    vector<LineRansac> testLines;
    for(int i = 0; i < 10; i++)
    {
        Vec3d firstPoint = Vec3d(double(rand() + 1), double(rand() + 1), 0.);
        Vec3d secondPoint = Vec3d(double(rand() + 1), double(rand() + 1), 0.);
        testLines.emplace_back(firstPoint, secondPoint);
    }
    //Make 10 fake lines in plane OXY with rot(theta, (0, 0, 1))
    const double theta = TEN_DEGREES_IN_RADIANS - 5.*3.141592/180.;
    for(int i = 0; i < 10; i++)
    {
        double x1 = rand() + 1;
        double x2 = rand() + 1;
        Vec3d firstPoint = Vec3d(x1*cos(theta), double(rand() + 1), x1*sin(theta));
        Vec3d secondPoint = Vec3d(x2*cos(theta), double(rand() + 1), x2*sin(theta));
        testLines.emplace_back(firstPoint, secondPoint);
    }
    // Add only 1 line on OY to make mutual support weak
    Vec3d firstPoint = Vec3d(0., double(rand()), 0.);
    Vec3d secondPoint = Vec3d(0., double(rand()), 0.);
    testLines.emplace_back(firstPoint, secondPoint);

    //Check that RANSAC is able to recognize the 2 planes
    RANSAC myRansac(testLines, 1e-5, 0.2, int(1e3));
    cout.setstate(ios_base::failbit);
    myRansac.extractAllPlanes();
    cout.clear();
    vector<Plane> detectedPlanes = myRansac.getDetectedPlanes();
    ASSERT_EQ(detectedPlanes.size(), 2);

    // Check that the 2 planes are indeed not merged after the plane refinement procedure (weak mutual support)
    myRansac.refineExtractedPlanes();
    ASSERT_EQ(myRansac.getDetectedPlanes().size(), 2);
}