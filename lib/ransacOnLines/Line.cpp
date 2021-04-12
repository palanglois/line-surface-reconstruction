#include "Line.h"

using namespace std;

LineRansac::LineRansac(Vec3d _firstPoint, Vec3d _secondPoint,
           vector<Vec3d> _pointOfViews,
           std::vector<std::vector<double>> _residuals) : firstPoint(_firstPoint),
                                                          secondPoint(_secondPoint),
                                                          direction((_secondPoint -
                                                                     _firstPoint).normalized()),
                                                          norm((_secondPoint -
                                                                _firstPoint).norm()),
                                                          pointOfViews(std::move(
                                                                  _pointOfViews)),
                                                          residuals(std::move(_residuals)),
                                                          nbTimesClassified(0)
{

}

void LineRansac::classify(int i)
{
    nbTimesClassified ++;
    assert(find(planesIndex.begin(), planesIndex.end(), i) == planesIndex.end() or planesIndex.empty());
    planesIndex.push_back(i);
    assert(planesIndex.size() <= 2);
}

bool LineRansac::hasBeenClassifiedInPlane(int planeIndex)
{
    return find(planesIndex.begin(), planesIndex.end(), planeIndex) != planesIndex.end() && !planesIndex.empty();
}

void LineRansac::emptyPlaneIndex()
{
  planesIndex = vector<int>(0);
  nbTimesClassified = 0;
}

unsigned int LineRansac::getNbTimesClassified() const
{
    assert(nbTimesClassified == planesIndex.size());
    return nbTimesClassified;
}

const Vec3d& LineRansac::getFirstPoint() const
{
    return firstPoint;
}

const Vec3d& LineRansac::getSecondPoint() const
{
    return secondPoint;
}

const Vec3d& LineRansac::getDirection() const
{
    return direction;
}

double LineRansac::getNorm() const
{
    return norm;
}

const std::vector<Vec3d>& LineRansac::getPointOfViews() const
{
    return pointOfViews;
}

const vector<vector<double>>& LineRansac::getResiduals() const
{
    return residuals;
}

const std::vector<int>& LineRansac::getPlanesIndex() const
{
    return planesIndex;
}

int LineRansac::getFirstCluster() const
{
    assert(planesIndex.size() == 1);
    return planesIndex[0];
}
