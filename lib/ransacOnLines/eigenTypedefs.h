#ifndef RANSAC_ON_LINES_CPP_EIGENTYPEDEFS_H
#define RANSAC_ON_LINES_CPP_EIGENTYPEDEFS_H


#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <list>
#include "json/json.hpp"

typedef Eigen::Vector3d Vec3d;
typedef Eigen::Matrix3d Mat3d;

/* Conversion from Vec3d to vector<double> */
inline std::vector<double> eigen2vec(const Vec3d& eigenVec)
{
    return std::vector<double>(eigenVec.data(), eigenVec.data() + eigenVec.rows() * eigenVec.cols());
}

/* Conversion from json to Vec3d */
inline Vec3d json2vec3d(const nlohmann::json& jsonData)
{
    assert(jsonData.type() == nlohmann::json::value_t::array);
    assert(jsonData.size() == 3);
    return {jsonData[0], jsonData[1], jsonData[2]};
}

/* Conversion from json to vector<double> */
inline std::vector<double> json2vector(const nlohmann::json& jsonData)
{
    assert(jsonData.type() == nlohmann::json::value_t::array);
    std::vector<double> outVector(0);
    for(size_t i=0; i < jsonData.size(); i++)
        outVector.push_back(jsonData[i]);
    return outVector;
}

#endif //RANSAC_ON_LINES_CPP_EIGENTYPEDEFS_H
