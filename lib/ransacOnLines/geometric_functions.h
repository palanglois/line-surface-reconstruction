#ifndef RANSAC_ON_LINES_CPP_GEOMETRIC_FUNCTIONS_H
#define RANSAC_ON_LINES_CPP_GEOMETRIC_FUNCTIONS_H

#include <random>
#include <cassert>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#include "eigenTypedefs.h"
#include "Line.h"
#include "Plane.h"

/* Get the point that is at the minimal distance of the two infinite lines defined by
 * firstLine and secondLine*/
Vec3d getMidPointSegment(const LineRansac& firstLine, const LineRansac& secondLine);

/* Compute the distance between a line and the intersection of two planes */
double distanceToIntersection(const LineRansac& line, const Plane &planeA, const Plane &planeB);

/* Compute the distance between a line and a plane */
double computeDistanceLineToPlane(const LineRansac& line, const Plane& plane);

/* Check if a line crosses a plane */
inline bool lineCrossesPlane(const LineRansac& line, const Plane& plane)
{
    double pt1Dist = (line.getFirstPoint() - plane.getPlaneInlier()).dot(plane.getPlaneNormal());
    double pt2Dist = (line.getSecondPoint() - plane.getPlaneInlier()).dot(plane.getPlaneNormal());
    return pt1Dist * pt2Dist < 0 && fabs(pt1Dist) > 1e-10 && fabs(pt2Dist) > 1e-10;
}

#endif //RANSAC_ON_LINES_CPP_GEOMETRIC_FUNCTIONS_H
