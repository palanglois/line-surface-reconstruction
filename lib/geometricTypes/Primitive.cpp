#include "Primitive.h"

using namespace std;

Primitive::Primitive(Point _inlier, Vector _normal) :
        inlier(std::move(_inlier)), normal(std::move(_normal))
{
    assert(fabs(CGAL::to_double(normal.squared_length()) - 1.) < 1e-10);
}

void Primitive::invertPlaneOrientation()
{
    normal = -1.*normal;
}

Plane Primitive::toCgalPlane() const
{
    return Plane(inlier, Direction(normal));
}

const Point& Primitive::getInlier() const
{
    return inlier;
}

const Vector& Primitive::getNormal() const
{
    return normal;
}