#ifndef LINE_BASED_RECONS_REFACTO_PLANE_ARRANGEMENT_WEIGHT_FIXTURE_H
#define LINE_BASED_RECONS_REFACTO_PLANE_ARRANGEMENT_WEIGHT_FIXTURE_H

#include "WeightComputation.h"
#include "ArrangementToMosek.h"

class PlaneArrangementWeightFixture: public ::testing::Test
{
public:
    PlaneArrangementWeightFixture();
    void SetUp();
    void TearDown();
    ~PlaneArrangementWeightFixture();
protected:
    PlaneArrangement* myPlaneArrangement;
    LineSet* myLineSet;
    PrimitiveSet* myPrimitiveSet;
};

#endif //LINE_BASED_RECONS_REFACTO_PLANE_ARRANGEMENT_WEIGHT_FIXTURE_H
