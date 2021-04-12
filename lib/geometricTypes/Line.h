#ifndef LINE_BASED_RECONS_REFACTO_LINE_H
#define LINE_BASED_RECONS_REFACTO_LINE_H

#include "cgalTypes.h"

class Line
{
public:
    Line(Point _firstPoint, Point _secondPoint,
         std::vector<Point> _pointOfViews, std::vector<int> _associatedPrimitives);

    // Disable the copy constructor to avoid undesired copy
    Line(const Line& that) = delete;

    /* Getters */
    const Point& getFirstPoint() const;
    const Point& getSecondPoint() const;
    const std::vector<Point>& getPointOfViews() const;
    const std::vector<int>& getAssociatedPrimitives() const;
    const bool isStructural() const;
    Segment getSegment() const;

private:
    const Point firstPoint;
    const Point secondPoint;
    const std::vector<Point> pointOfViews; //Points from which the line was seen
    const std::vector<int> associatedPrimitives; //Index of primitives associated to the line
    const bool _isStructural; //True if the line is associated to more than one primitive
};


#endif //LINE_BASED_RECONS_REFACTO_LINE_H
