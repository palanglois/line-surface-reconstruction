#include <iostream>
#include "OptionParser/option_parser.h"
#include "lib/ransacOnLines/RANSAC.h"
#include "lib/ransacOnLines/ioRansac.h"

using namespace std;


int main(int argc, char* argv[])
{
    //Create parsing options
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Input line file", "");
    opt.add_option("-o", "--output", "Output line file", "./");
    opt.add_option("-e", "--epsilon", "epsilon", "0.02");
    opt.add_option("-n", "--nb_iter", "Number of RANSAC iterations", "100");
    opt.add_option("-nr", "--nb_max_refit_iter", "Max number of re-fitting iterations", "20");
    opt.add_option("-np", "--max_nb_planes", "Maximum number of planes to extract", "1000");
    opt.add_option("-r", "--no_refine", "Don't refine the obtained planes");
    opt.add_option("-ao", "--angles_refine_only", "Chooses angles only refinement (if -r is not used)");
    opt.add_option("-s", "--seed","Random seed", "-1");
    opt.add_option("-v", "--verbose", "Verbosity trigger");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if(!correctParsing)
        return EXIT_FAILURE;

    if(op::str2bool(opt["-h"]))
    {
        opt.show_help();
        return 0;
    }

    if(opt["-i"].empty())
    {
        cerr << "Input json lines file (-i) is required!" << endl;
        opt.show_help();
        return EXIT_FAILURE;
    }

    string path = opt["-i"];
    double epsilon = op::str2double(opt["-e"]);
    int nbRansacIter = op::str2int(opt["-n"]);
    int maxRefitIter = op::str2int(opt["-nr"]);
    double parallelThreshold = 1e-5;
    int maxNbPlanes = op::str2int(opt["-np"]);
    int seed = opt["-s"] == "-1" ? (int)time(NULL) : op::str2int(opt["-s"]);
    bool verbose = op::str2bool(opt["-v"]);

    string path_out = opt["-o"];
    if(path_out[path_out.size()-1] != '/')
        path_out.append("/");

    // Initialize random number generator
    srand((unsigned int)seed);

    // Read the lines from file
    vector<LineRansac> lineSet(0);
    loadFromJson(path, lineSet);

    //Prepare and launch the RANSAC
    RANSAC myRansac = RANSAC(lineSet, parallelThreshold, epsilon, int(1e5), nbRansacIter,
            maxRefitIter, maxNbPlanes, verbose);
    myRansac.extractAllPlanes();
    if(!op::str2bool(opt["-r"])) {
        myRansac.writeOutputJson(path_out + "test_no_refine");
        if(op::str2bool(opt["-ao"]))
            myRansac.refineExtractedPlanesAngleOnly();
        else
            myRansac.refineExtractedPlanes();
    }

    //Write the output
    myRansac.writeOutputJson(path_out + "test");

    return 0;
}
