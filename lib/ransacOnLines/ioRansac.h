#ifndef RANSAC_ON_LINES_CPP_IORANSAC_H
#define RANSAC_ON_LINES_CPP_IORANSAC_H

#include "geometric_functions.h"
#include "Line.h"

void getline_in_sstream(std::ifstream &ifs, std::stringstream &sstr);

void loadFromCsv(std::string path, std::vector<LineRansac> &lineSet);

void loadFromJson(std::string path, std::vector<LineRansac> &lineSet);

#endif //RANSAC_ON_LINES_CPP_IO_RANSAC_H
