#ifndef LINE_BASED_RECONS_REFACTO_PLANE_ARRANGEMENT_FIXTURE_H
#define LINE_BASED_RECONS_REFACTO_PLANE_ARRANGEMENT_FIXTURE_H

#include <cmath>
#include <map>
#include "WeightComputation.h"

class PlaneArrangementFixture: public ::testing::Test
{
public:
    PlaneArrangementFixture();
    void SetUp();
    void TearDown();
    ~PlaneArrangementFixture();
protected:
    PlaneArrangement* myPlaneArrangement;
};

#endif //LINE_BASED_RECONS_REFACTO_PLANE_ARRANGEMENT_FIXTURE_H
