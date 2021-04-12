#ifndef LINE_BASED_RECONS_REFACTO_PRIMITIVE_H
#define LINE_BASED_RECONS_REFACTO_PRIMITIVE_H

#include "cgalTypes.h"

class Primitive
{
public:
    Primitive(Point _inlier, Vector _normal);

    // Disable the copy constructor to avoid undesired copy
    Primitive(const Primitive& that) = delete;

    /* Compute orthogonal projection of a point on the plane */
    Point orthogonalProjection(const Point& pointToProject) const;

    /* Compute orthogonal projection of a Segment on the plane */
    Segment orthogonalProjectionSeg(const Segment& segmentToProject) const;

    /* Inverting the plane orientation */
    void invertPlaneOrientation();

    /* Conversion to CGAL plane */
    Plane toCgalPlane() const;

    /* Getters */
    const Point& getInlier() const;
    const Vector& getNormal() const;

private:
    const Point inlier;
    Vector normal;
};

inline Point Primitive::orthogonalProjection(const Point& pointToProject) const
{
    return pointToProject - (pointToProject - inlier)*normal * normal;
}

inline Segment Primitive::orthogonalProjectionSeg(const Segment& segmentToProject) const
{
    return Segment(orthogonalProjection(segmentToProject.source()), orthogonalProjection(segmentToProject.target()));
}


#endif //LINE_BASED_RECONS_REFACTO_PRIMITIVE_H
