#ifndef RANSAC_ON_LINES_CPP_PLANE_H
#define RANSAC_ON_LINES_CPP_PLANE_H

#include "eigenTypedefs.h"


class Plane {
public:
    Plane();
    Plane& operator=(const Plane& plane);
    Plane(Vec3d _planeNormal, Vec3d _planeInlier);

    /* Operators */
    bool operator!=(const Plane& rhs) const;

    /* Getters */
    const Vec3d& getPlaneNormal() const;
    const Vec3d& getPlaneInlier() const;
    const std::vector<int>& getInliers() const;

    /* Setters */
    void setInliers(std::vector<int> newInliers);
private:
    Vec3d planeNormal;
    Vec3d planeInlier;
    std::vector<int> inliers;
};

inline bool Plane::operator!=(const Plane& rhs) const
{
    return planeNormal != rhs.planeNormal || planeInlier != rhs.planeInlier;
}

/* Typedefs for Plane refinement */
typedef std::list<Plane>::iterator PlaneIt;
typedef std::tuple<PlaneIt, PlaneIt, double> PlanePair;
class PlanePairComp
{
public:
    bool operator()(PlanePair i,PlanePair j)
    {
        return (std::get<2>(i)<std::get<2>(j));
    }
};


#endif //RANSAC_ON_LINES_CPP_PLANE_H
