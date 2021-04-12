#include "geometric_functions.h"

using namespace std;

Vec3d getMidPointSegment(const LineRansac& firstLine, const LineRansac& secondLine)
{
    const Vec3d& firstDirection = firstLine.getDirection();
    const Vec3d& secondDirection = secondLine.getDirection();
    double denominator = pow(firstDirection.dot(secondDirection), 2) - 1.;
    double t0 = (secondLine.getFirstPoint() - firstLine.getFirstPoint()).dot(
            -firstDirection + firstDirection.dot(secondDirection) * secondDirection) / denominator;
    double t1 = (secondLine.getFirstPoint() - firstLine.getFirstPoint()).dot(
            secondDirection - firstDirection.dot(secondDirection) * firstDirection) / denominator;
    Vec3d firstNearestPoint = firstLine.getFirstPoint() + t0 * firstDirection;
    Vec3d secondNearestPoint = secondLine.getFirstPoint() + t1 * secondDirection;
    return (firstNearestPoint + secondNearestPoint) / 2.;
}

double computeDistanceLineToPlane(const LineRansac& line,const Plane& plane)
{
    const Vec3d& pt1 = line.getFirstPoint();
    const Vec3d& pt2 = line.getSecondPoint();
    double pt1Dist = (pt1 - plane.getPlaneInlier()).dot(plane.getPlaneNormal());
    double pt2Dist = (pt2 - plane.getPlaneInlier()).dot(plane.getPlaneNormal());
    if(pt1Dist * pt2Dist < 0 && fabs(pt1Dist) > 1e-10 && fabs(pt2Dist) > 1e-10)
    {
        Vec3d lineDirection = (pt1 - pt2).normalized();
        // Compute intersection with the plane
        double t = (plane.getPlaneInlier() - pt1).dot(plane.getPlaneNormal())
                   / plane.getPlaneNormal().dot(lineDirection);
        Vec3d intersectionPoint = pt1 + t*lineDirection;
        Vec3d firstMidPoint = (intersectionPoint + pt1) / 2.;
        Vec3d secondMidPoint = (intersectionPoint + pt2) / 2.;
        // The distance is the sum of the distance with the 2 mid-points in this case
        return fabs((firstMidPoint - plane.getPlaneInlier()).dot(plane.getPlaneNormal())) +
                fabs((secondMidPoint - plane.getPlaneInlier()).dot(plane.getPlaneNormal()));
    }
    else
        return (fabs(pt1Dist) + fabs(pt2Dist)) / 2.;
}

double distanceToIntersection(const LineRansac& line, const Plane &planeA, const Plane &planeB)
{
    // Retrieving the characteristics of the plane the line already belongs to
    const Vec3d& linePlaneNormal = planeB.getPlaneNormal();
    const Vec3d& linePlaneInlier = planeB.getPlaneInlier();

    //Setting up linear system
    Mat3d leftMember;
    Vec3d crossNormal = planeA.getPlaneNormal().cross(linePlaneNormal);
    leftMember.row(0) = planeA.getPlaneNormal();
    leftMember.row(1) = linePlaneNormal;
    leftMember.row(2) = crossNormal;

    Vec3d rightMember1(0., 0., 0.);
    rightMember1[0] = planeA.getPlaneNormal().dot(planeA.getPlaneInlier());
    rightMember1[1] = linePlaneNormal.dot(linePlaneInlier);
    rightMember1[2] = crossNormal.dot(line.getFirstPoint());

    Vec3d rightMember2 = rightMember1.replicate(1, 1);
    rightMember2[2] = crossNormal.dot(line.getSecondPoint());

    //Solve linear systems
    Vec3d firstProjOnAxis = leftMember.colPivHouseholderQr().solve(rightMember1);
    Vec3d secondProjOnAxis = leftMember.colPivHouseholderQr().solve(rightMember2);

    //Compute distances to axis
    double firstDistance = (firstProjOnAxis - line.getFirstPoint()).norm();
    double secondDistance = (secondProjOnAxis - line.getSecondPoint()).norm();

    return max(firstDistance, secondDistance);
}
