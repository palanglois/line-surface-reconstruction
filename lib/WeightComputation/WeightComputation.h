#ifndef LINE_BASED_RECONS_REFACTO_WEIGHTCOMPUTATION_H
#define LINE_BASED_RECONS_REFACTO_WEIGHTCOMPUTATION_H

#include "PlaneArrangement.h"

void computeWeights(PlaneArrangement &arr, const LineSet &allLines, double sigma, bool verbose=false);

Segment projectSegmentOnPlanesIntersection(const Segment& curSegment, const Plane& plane_0, const Plane& plane_1);

TripletCell findStructuralCells(const SegmentWithNeighbourCells &segAndCells, const Point& pointOfView,
                                const PlaneArrangement &arr);

int findFullTexturialCell(const SegmentWithNeighbourCells &segAndCells, const Point& pointOfView,
                          const PlaneArrangement &arr);

#endif //LINE_BASED_RECONS_REFACTO_WEIGHTCOMPUTATION_H
