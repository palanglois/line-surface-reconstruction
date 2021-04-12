#include "Plane.h"

using namespace std;

Plane::Plane() : planeNormal(Vec3d(0., 0., 0.)),
                 planeInlier(Vec3d(0., 0., 0.))
{

}

Plane& Plane::operator=(const Plane &plane)
{
    if(this != &plane)
    {
        planeNormal = plane.planeNormal;
        planeInlier = plane.planeInlier;
    }
    return *this;
}

Plane::Plane(Vec3d _planeNormal, Vec3d _planeInlier) : planeNormal(move(_planeNormal)),
                                                       planeInlier(move(_planeInlier))
{

}

const Vec3d& Plane::getPlaneInlier() const
{
    return planeInlier;
}

const Vec3d& Plane::getPlaneNormal() const
{
    return planeNormal;
}

const std::vector<int>& Plane::getInliers() const
{
    return inliers;
}

void Plane::setInliers(std::vector<int> newInliers)
{
    inliers = std::move(newInliers);
}
