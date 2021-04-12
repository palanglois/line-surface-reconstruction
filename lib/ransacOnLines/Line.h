#ifndef RANSAC_ON_LINES_CPP_LINE_H
#define RANSAC_ON_LINES_CPP_LINE_H

#include "eigenTypedefs.h"


class LineRansac
{
public:
    explicit LineRansac(Vec3d _firstPoint = Vec3d(0., 0., 0.), Vec3d _secondPoint = Vec3d(0., 0., 0.),
         std::vector<Vec3d> _pointOfViews=std::vector<Vec3d>(0),
         std::vector<std::vector<double>>_residuals=std::vector<std::vector<double>>(0));

    /* Signal to tell the lines that it has been classified in the plane i*/
    void classify(int i);

    /* Check if the line was already classified in plane planeIndex */
    bool hasBeenClassifiedInPlane(int planeIndex);

    /* Facility accessor for the plane refinement process */
    void emptyPlaneIndex();

    /* Getters */
    unsigned int getNbTimesClassified() const;
    const Vec3d& getFirstPoint() const;
    const Vec3d& getSecondPoint() const;
    const std::vector<Vec3d>& getPointOfViews() const;
    const std::vector<std::vector<double>>& getResiduals() const;
    const Vec3d& getDirection() const;
    double getNorm() const;
    const std::vector<int>& getPlanesIndex() const;
    int getFirstCluster() const;

private:
    const Vec3d firstPoint;
    const Vec3d secondPoint;
    const Vec3d direction;
    const double norm;
    const std::vector<Vec3d> pointOfViews;
    const std::vector<std::vector<double>> residuals;
    unsigned int nbTimesClassified;
    std::vector<int> planesIndex;
};

#endif //RANSAC_ON_LINES_CPP_LINE_H
