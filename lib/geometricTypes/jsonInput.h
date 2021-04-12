#ifndef LINE_BASED_RECONS_REFACTO_JSONINPUT_H
#define LINE_BASED_RECONS_REFACTO_JSONINPUT_H

#include <fstream>
#include <json/json.hpp>
#include "PrimitiveSet.h"

inline void loadInputFromJson(std::string path, LineSet &myLineSet, PrimitiveSet &myPrimitiveSet,
                              int maxNumberOfPlanes=-1, bool verbose=false)
{

    //Loading json data
    nlohmann::json inputData;
    std::ifstream inputStream(path.c_str());
    if(!inputStream)
    {
        std::cerr << "Could not load file located at : " << path << std::endl;
        return;
    }
    inputStream >> inputData;

    //Loading planes
    nlohmann::json planesArray = inputData.at("planes");
    size_t nbPlanes = maxNumberOfPlanes < 0 ? planesArray.size() : min((int) planesArray.size(), maxNumberOfPlanes);
    for (int i = 0; i < nbPlanes; i++)
        myPrimitiveSet.insert(planesArray[i]);

    //Loading lines
    myLineSet.setMaxNbPlanes(nbPlanes); // Ignores links towards planes that are not loaded
    nlohmann::json linesArray = inputData.at("lines");
    for (const auto& curLine : linesArray)
        myLineSet.insert(curLine);
    if(verbose)
        std::cout << "Loaded " << myLineSet.size() << " lines and " << myPrimitiveSet.size() << " planes." << std::endl;
}

#endif //LINE_BASED_RECONS_REFACTO_JSONINPUT_H
