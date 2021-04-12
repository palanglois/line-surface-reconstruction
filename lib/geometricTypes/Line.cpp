#include "Line.h"


using namespace std;

Line::Line(Point _firstPoint, Point _secondPoint, vector<Point> _pointOfViews,
           vector<int> _associatedPrimitives) : firstPoint(std::move(_firstPoint)),
                                                secondPoint(std::move(_secondPoint)),
                                                pointOfViews(std::move(_pointOfViews)),
                                                associatedPrimitives(std::move(_associatedPrimitives)),
                                                _isStructural(associatedPrimitives.size() == 2)
{
    assert(associatedPrimitives.size() <= 2);
}

const Point& Line::getFirstPoint() const
{
    return firstPoint;
}

const Point& Line::getSecondPoint() const
{
    return secondPoint;
}

const std::vector<Point>& Line::getPointOfViews() const
{
    return pointOfViews;
}

const std::vector<int>& Line::getAssociatedPrimitives() const
{
    return associatedPrimitives;
}

const bool Line::isStructural() const
{
    return _isStructural;
}

Segment Line::getSegment() const
{
    return Segment(firstPoint, secondPoint);
}