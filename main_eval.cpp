#include "Triangle.h"

using namespace std;

int main(int argc, char* argv[])
{
    if(argc != 4)
    {
        cerr << "Error : there should be exactly 3 arguments; example: ./mesh_metrics ref_mesh.ply query_mesh.ply 10000"
                << endl;
        return 1;
    }
    string fileOne = argv[1];
    string fileTwo = argv[2];
    if(fileOne.substr(fileOne.length()-3, 3) != "ply" && fileOne.substr(fileOne.length()-3, 3) != "PLY")
    {
        cerr << "Error the 1st file should end with ply or PLY !" << endl;
        return 1;
    }
    if(fileTwo.substr(fileTwo.length()-3, 3) != "ply" && fileTwo.substr(fileTwo.length()-3, 3) != "PLY")
    {
        cerr << "Error the 2nd file should end with ply or PLY !" << endl;
        return 1;
    }
    int nbSample;
    try
    {
        nbSample = stoi(argv[3]);
    }
    catch (invalid_argument) {
        cerr << "Could not convert number of samples to an integer !\n";
        return 1;
    }

    // Random seed
    srand(0);

    // Load the meshes
    vector<Triangle> meshOne = loadTrianglesFromPly(fileOne);
    vector<Triangle> meshTwo = loadTrianglesFromPly(fileTwo);

    // Sample the points
    PointCloud sampledOne = samplePointsOnMesh(meshOne, nbSample);
    PointCloud sampledTwo = samplePointsOnMesh(meshTwo, nbSample);

    // Retrieve the NN distances
    multiset<double> nnDistancesOne = findPcDistance(sampledOne, sampledTwo);
    multiset<double> nnDistancesTwo = findPcDistance(sampledTwo, sampledOne);

    // Output the results
    const double percentage = 0.95;
    cout << "# Number of point sampled on each cloud : " << nbSample << endl;
    cout << "# Distance d such as 95% NN distances are under d" << endl;
    cout << "# Mesh 1 reference; Mesh 2 query (precision if Mesh 1 is ground truth) : " << endl;
    auto itOne = nnDistancesOne.begin();
    advance(itOne, int(percentage*(nnDistancesOne.size()-1)));
    cout << *itOne << endl;
    cout << "# Mesh 2 reference; Mesh 1 query (completeness if Mesh 1 is ground truth) : " << endl;
    auto itTwo = nnDistancesTwo.begin();
    advance(itTwo, int(percentage*(nnDistancesTwo.size()-1)));
    cout << *itTwo << endl;
    cout << "# Average of all measurement (mean metro)" << endl;
    double average = 0.;
    for(const auto &measurement : nnDistancesOne)
        average += measurement;
    for(const auto &measurement : nnDistancesTwo)
        average += measurement;
    cout << average / double(nnDistancesOne.size() + nnDistancesTwo.size()) << endl;
    cout << "# Max measurement (metro distance)" << endl;
    cout << max(*nnDistancesOne.rbegin(), *nnDistancesTwo.rbegin()) << endl;

    return 0;
}