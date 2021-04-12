#ifndef METRO_TRIANGLE_H
#define METRO_TRIANGLE_H

// STL
#include <vector>
#include <set>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

// External
#include "Eigen/Core"
#include "Eigen/Dense"
#include "tinyply/tinyply.h"
#include "nanoflann/nanoflann.hpp"

struct float3 { float x, y, z; };
struct uint3 { uint32_t x, y, z; };

typedef Eigen::Vector3d Point;
typedef Eigen::Matrix<double, Eigen::Dynamic, 3> PointCloud;
typedef nanoflann::KDTreeEigenMatrixAdaptor<PointCloud> EigenKdTree;

class Triangle
{
public:
    Triangle();
    Triangle(Point _a, Point _b, Point _c);
    double getArea() const;
    const Point& getA() const;
    const Point& getB() const;
    const Point& getC() const;
private:
    const Point a;
    const Point b;
    const Point c;
};

std::vector<Triangle> loadTrianglesFromPly(const std::string& path);

PointCloud samplePointsOnMesh(const std::vector<Triangle>& mesh, int nbSamples);

std::multiset<double> findPcDistance(const PointCloud& refPointCloud, const PointCloud& queryPointCloud);

#endif //METRO_TRIANGLE_H
