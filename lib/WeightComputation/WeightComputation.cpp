#include "WeightComputation.h"

using namespace std;

void computeWeights(PlaneArrangement &arr, const LineSet &allLines, double sigma, bool verbose)
{
    int nbLinesProcessed = 0;
#pragma omp parallel for schedule(dynamic, 1)
    for(size_t l_id=0; l_id < allLines.size(); l_id++)
    {
        // If the line is not associated to a primitive, there is nothing to do
        if(allLines[l_id].getAssociatedPrimitives().empty()) continue;

        // Initiating the projection of the line
        Segment projectedLine;

        //// Primitive term textural ////
        if(!allLines[l_id].isStructural())
        {
            // We project the line on its primitive
            auto myPlane = arr.plane(arr.getPlaneHandleFromPrimitiveId(allLines[l_id].getAssociatedPrimitives()[0]));
            projectedLine = Segment(myPlane.projection(allLines[l_id].getSegment().source()),
                                    myPlane.projection(allLines[l_id].getSegment().target()));
            if(projectedLine.squared_length() == 0) continue;

            // We intersect it with the plane arrangement
            IntersectionsOutput intersections = arr.intersectWithSegment(projectedLine);

            // For each segment intersection, we add the corresponding weights
            for(auto &segmentIntersection: intersections.segments)
            {
                double segmentLength = sqrt(CGAL::to_double(segmentIntersection.segment.squared_length()));
                for(const Point& pointOfView: allLines[l_id].getPointOfViews())
                {
                    int fullTexturialCell = findFullTexturialCell(segmentIntersection, pointOfView, arr);
#pragma omp atomic
                    arr.cell(fullTexturialCell).cell_value_full_points += segmentLength / sigma;
                }
            }
        }
        //// Primitive term structural ////
        else
        {
            // We project the line on the intersection of its two primitives
            auto myPlane0 = arr.plane(arr.getPlaneHandleFromPrimitiveId(allLines[l_id].getAssociatedPrimitives()[0]));
            auto myPlane1 = arr.plane(arr.getPlaneHandleFromPrimitiveId(allLines[l_id].getAssociatedPrimitives()[1]));
            projectedLine = projectSegmentOnPlanesIntersection(allLines[l_id].getSegment(), myPlane0, myPlane1);
            if(projectedLine.squared_length() == 0) continue;

            // We intersect it with the plane arrangement
            IntersectionsOutput intersections = arr.intersectWithSegment(projectedLine);

            // For each segment intersection, we add the corresponding weights
            for (auto &segmentIntersection: intersections.segments)
            {
                double segmentLength = sqrt(CGAL::to_double(segmentIntersection.segment.squared_length()));

                for(const Point& pointOfView: allLines[l_id].getPointOfViews())
                {
                    TripletCell structTripletCells = findStructuralCells(segmentIntersection, pointOfView, arr);
#pragma omp critical
                    arr.insertStructuralCost(structTripletCells, segmentLength/sigma);
                }
            }
        }

        //// Visibility term ////
        for(const Point& pointOfView: allLines[l_id].getPointOfViews())
        {
            // Making the visibility triangle
            Triangle visTriangle(pointOfView, projectedLine.source(), projectedLine.target());

            // We intersect the visibility triangle with the plane arrangement
            IntersectionsOutput intersections = arr.intersectWithTriangle(visTriangle);

            // We also get the facets intersected by the projected line (they should not get visibility penalty)
            set<int> excludedFacets = arr.getListOfIntersectedFacets(projectedLine);

            // For each segment intersection, we add the corresponding weight
            for(auto &segmentIntersection: intersections.segments)
            {
                // We discard degenerate cases where the triangle crosses an edge of the plane arrangement
                if(segmentIntersection.neighbourCells.facet == -1) continue;
                // We discard the facets which are on the projected line
                if(excludedFacets.find(segmentIntersection.neighbourCells.facet) != excludedFacets.end()) continue;
                // We make the update
                double segmentLength = sqrt(CGAL::to_double(segmentIntersection.segment.squared_length()));
#pragma omp atomic
                arr.facet(segmentIntersection.neighbourCells.facet).facet_nbr_ray_vis_point +=
                        segmentLength / sigma;
            }


            // The cell which contains the point of view should be void
            PlaneArrangement::Face_handle hf_view = find_containing_cell((Polyhedral_complex_3::Arrangement_3<Kernel>) arr, pointOfView);
#pragma omp atomic
            arr.cell(hf_view).cell_value_void_view_points += 1; // equivalent to 1*sigma line

        }
#pragma omp atomic
        nbLinesProcessed++;
        if (verbose)
            cout << "Processed line " << nbLinesProcessed << " out of " << allLines.size() << "." << endl;
    }
}

Segment projectSegmentOnPlanesIntersection(const Segment& curSegment, const Plane& plane_0, const Plane& plane_1)
{
    CGAL::cpp11::result_of<Intersect(Plane, Plane)>::type planesIntersection = intersection(plane_0, plane_1);

    if(!planesIntersection)
    {
        cerr << "Problem when intersecting segment : " << curSegment.source() << " " << curSegment.target() << endl;
        return Segment(Point(0., 0., 0.), Point(0., 0., 0.));
    }
    if (const LineCgal *s = boost::get<LineCgal>(&*planesIntersection)) {
        return Segment(s->projection(curSegment.source()), s->projection(curSegment.target()));
    }
    else
    {
        cerr << "Problem when intersecting segment : " << curSegment.source() << " " << curSegment.target() << endl;
        return Segment(Point(0., 0., 0.), Point(0., 0., 0.));
    }
}

/* Among the 4 cells contained in segAndCells, find the 3 one in which the triangle formed by segment and
 * pointOfView is not intersecting. For that, we just use angles.*/
TripletCell findStructuralCells(const SegmentWithNeighbourCells &segAndCells, const Point& pointOfView,
                                const PlaneArrangement &arr)
{
    Plane refPlane(segAndCells.segment.source(), segAndCells.segment.to_vector());
    Point projPov = refPlane.projection(pointOfView);
    vector<int> cellIndice = {segAndCells.neighbourCells.positive, segAndCells.neighbourCells.negative,
                              segAndCells.neighbourCells.neutral[0], segAndCells.neighbourCells.neutral[1]};
    sort(cellIndice.begin(), cellIndice.end());
    vector<Scalar> cosine;
    for(const auto &indice: cellIndice)
    {
        auto projCellCenter = refPlane.projection(arr.cell(indice).point());
        cosine.push_back((projCellCenter-segAndCells.segment.source())*(projPov-segAndCells.segment.source()));
    }
    int povCellIndex = static_cast<int>(distance(cosine.begin(), max_element(cosine.begin(), cosine.end())));
    switch(povCellIndex)
    {
        case 0:
            return make_tuple(cellIndice[1], cellIndice[2], cellIndice[3]);
            break;
        case 1:
            return make_tuple(cellIndice[0], cellIndice[2], cellIndice[3]);
            break;
        case 2:
            return make_tuple(cellIndice[0], cellIndice[1], cellIndice[3]);
            break;
        case 3:
            return make_tuple(cellIndice[0], cellIndice[1], cellIndice[2]);
            break;
        default:
            break;
    }
    return make_tuple(-1, -1, -1);
}

int findFullTexturialCell(const SegmentWithNeighbourCells &segAndCells, const Point& pointOfView,
                          const PlaneArrangement &arr)
{
    Plane refPlane(segAndCells.segment.source(), segAndCells.segment.to_vector());
    Point projPov = refPlane.projection(pointOfView);

    auto projCellCenterPos = refPlane.projection(arr.cell(segAndCells.neighbourCells.positive).point());
    auto projCellCenterNeg = refPlane.projection(arr.cell(segAndCells.neighbourCells.negative).point());
    Scalar cosinePos = (projCellCenterPos-segAndCells.segment.source())*(projPov-segAndCells.segment.source());
    Scalar cosineNeg = (projCellCenterNeg-segAndCells.segment.source())*(projPov-segAndCells.segment.source());

    return cosinePos > cosineNeg ? segAndCells.neighbourCells.negative : segAndCells.neighbourCells.positive;

}


