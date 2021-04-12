#ifndef LINE_BASED_RECONS_REFACTO_JSONTOOLS_H
#define LINE_BASED_RECONS_REFACTO_JSONTOOLS_H

#include <json/json.hpp>


/* Conversion from json to cgal 3D data */
template <typename T>
inline T json2cgal(const nlohmann::json &jsonData)
{
    assert(jsonData.type() == nlohmann::json::value_t::array);
    assert(jsonData.size() == 3);
    return {jsonData[0].get<double>(), jsonData[1].get<double>(), jsonData[2].get<double>()};
}


#endif //LINE_BASED_RECONS_REFACTO_JSONTOOLS_H
