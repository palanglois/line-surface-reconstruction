//
// Created by langlois on 14/03/18.
//

#include "ioRansac.h"

using namespace std;
using Json = nlohmann::json;

void getline_in_sstream(ifstream &ifs, stringstream &sstr)
{
    sstr.str("");
    sstr.clear();
    string line;
    getline(ifs, line);
    sstr << line;
}


void loadFromCsv(string path, vector<LineRansac>& lineSet)
{
    ifstream in_stream(path.c_str());
    if (!in_stream)
        cerr << "Could not read data" << endl;
    unsigned long nbOfLines = 0;
    stringstream streamForCurrentLine;
    getline_in_sstream(in_stream, streamForCurrentLine);
    streamForCurrentLine >> nbOfLines;
    lineSet.reserve(nbOfLines);
    double x1, x2, y1, y2, z1, z2, vx, vy, vz = 0.;
    for (unsigned long i = 0; i < nbOfLines; i++) {
        getline_in_sstream(in_stream, streamForCurrentLine);
        streamForCurrentLine >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> vx >> vy >> vz;
        Vec3d firstPoint(x1, y1, z1);
        Vec3d secondPoint(x2, y2, z2);
        Vec3d pointOfView(vx, vy, vz);
        lineSet.push_back(LineRansac(firstPoint, secondPoint, {pointOfView}));
    }
    cout << nbOfLines << " " << lineSet.size() << endl;
}

void loadFromJson(std::string path, std::vector<LineRansac> &lineSet)
{
    ifstream inStream(path.c_str());
    if (!inStream)
        cerr << "Could not read data" << endl;

    Json jsonData;
    inStream >> jsonData;

    assert(jsonData.type() == nlohmann::json::value_t::object);
    assert(jsonData.at("lines").type() == nlohmann::json::value_t::array);
    Json lineArray = jsonData.at("lines");
    lineSet.reserve(lineArray.size());
    for (auto &line : lineArray)
    {
        Vec3d firstPoint = json2vec3d(line.at("pt1"));
        Vec3d secondPoint = json2vec3d(line.at("pt2"));
        vector<Vec3d> pointOfViews(0);
        assert(line.at("pt_views").type() == nlohmann::json::value_t::array);
        Json ptViews = line.at("pt_views");
        for (const auto &ptView : ptViews)
            pointOfViews.push_back(json2vec3d(ptView));
        vector<vector<double>> resid(0);
        if(line.find("residual_per_pt_view") != line.end()) {
            if (line.at("residual_per_pt_view").type() == nlohmann::json::value_t::array) {
                Json residuals = line.at("residual_per_pt_view");
                for (const auto &residual : residuals)
                    resid.push_back(json2vector(residual));
            }
        }
        lineSet.emplace_back(firstPoint, secondPoint, pointOfViews, resid);
    }
    cout << "Loaded " << lineSet.size() << " lines." << endl;
}