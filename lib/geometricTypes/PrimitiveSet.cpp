#include "PrimitiveSet.h"

using namespace std;

PrimitiveSet::PrimitiveSet() : ObjectSet<Primitive>()
{

}

void PrimitiveSet::insert(const nlohmann::json &primitiveJson)
{
    Vector planeNormal = json2cgal<Vector>(primitiveJson.at("normal"));
    Point planeInlier = json2cgal<Point>(primitiveJson.at("inlier"));
    Primitive * curPlanePrim = new Primitive(planeInlier, planeNormal);

    objects.push_back(curPlanePrim);
}

int PrimitiveSet::orientatePrimitives(const LineSet &lineSet)
{
    //Use the line's point of views to orientate the planes
    vector<int> votesForPrimitive(objects.size(), 0);
    for (size_t l_id = 0; l_id < lineSet.size(); l_id++)
    {
        const vector<Point>& curPointOfViews = lineSet[l_id].getPointOfViews();
        for(auto curPrimId : lineSet[l_id].getAssociatedPrimitives())
        {
            const Vector &curPrimNormal = objects[curPrimId]->getNormal();
            const Point &curPrimInlier = objects[curPrimId]->getInlier();
            for (auto &curPtOfView : curPointOfViews)
                votesForPrimitive[curPrimId] += 2 * int((curPtOfView - curPrimInlier) * curPrimNormal > 0.) - 1;
        }
    }
    int count = 0;
    //Invert the primitive normals whose total number of votes is negative
    for (size_t i = 0; i < votesForPrimitive.size(); i++)
        if (votesForPrimitive[i] < 0)
        {
            count++;
            objects[i]->invertPlaneOrientation();
        }
    return count;
}
