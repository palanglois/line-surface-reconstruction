#include "LineSet.h"

using namespace std;
using Json = nlohmann::json;

LineSet::LineSet() : ObjectSet<Line>(), bbox(CGAL::Bbox_3(DBL_MAX, DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX)),
                     maxNbPlanes(-1)
{

}

void LineSet::insert(const nlohmann::json &lineJson)
{
    //Line's extremities
    Point firstPoint = json2cgal<Point>(lineJson.at("pt1"));
    Point secondPoint = json2cgal<Point>(lineJson.at("pt2"));
    //Index of the planes the line belongs to
    vector<int> associatedPrimitives;
    for(int planeIdx: lineJson.at("plane_index").get<vector<int>>())
        if((planeIdx < maxNbPlanes) || (maxNbPlanes < 0))
            associatedPrimitives.push_back(planeIdx);
    vector<Point> pointOfViews(0);
    //Point of views from which the line was seen
    Json pointOfViewsArray = lineJson.at("pt_views");
    for(const auto &ptView : pointOfViewsArray)
        pointOfViews.push_back(json2cgal<Point>(ptView));
    Line* curLineData = new Line(firstPoint, secondPoint, pointOfViews, associatedPrimitives);
    objects.push_back(curLineData);

    //Update the bounding box
    vector<Point> linePoints = {firstPoint, secondPoint};
    for(const auto& pov: pointOfViews) linePoints.push_back(pov);
    bbox += CGAL::bbox_3(linePoints.begin(), linePoints.end());
}

CGAL::Bbox_3 LineSet::getBbox(const double factor) const
{
    double xIncr = (bbox.xmax() - bbox.xmin()) * factor / 2.;
    double yIncr = (bbox.ymax() - bbox.ymin()) * factor / 2.;
    double zIncr = (bbox.zmax() - bbox.zmin()) * factor / 2.;
    return CGAL::Bbox_3(bbox.xmin() - xIncr, bbox.ymin() - yIncr, bbox.zmin() - zIncr,
                        bbox.xmax() + xIncr, bbox.ymax() + yIncr, bbox.zmax() + zIncr);
}

void LineSet::setMaxNbPlanes(int _maxNbPlanes)
{
    maxNbPlanes = _maxNbPlanes;
}
