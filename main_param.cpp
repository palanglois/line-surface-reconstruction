#include "OptionParser/option_parser.h"
#include "jsonInput.h"
#include "WeightComputation.h"
#include "ArrangementToMosek.h"

using namespace std;

int main(int argc, char* argv[])
{

    //Create parsing options
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-v", "--verbose", "Verbosity trigger");
    opt.add_option("-i", "--input", "Input line file", "");
    opt.add_option("-o", "--output", "Output line file", "./");
    opt.add_option("-vs", "--voxel-size", "Voxel size", "0");
    opt.add_option("-ext", "--exterior", "Exterior scene mode (-ext or -int REQUIRED)");
    opt.add_option("-int", "--interior", "Interior scene mode (-ext or -int REQUIRED)");
    opt.add_option("-np", "--max_number_planes", "(Optional) Max number of planes to insert", "-1");

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

    bool force_interior = op::str2bool(opt["-int"]);
    bool force_exterior = op::str2bool(opt["-ext"]);
    int maxNumberPlanes = op::str2int(opt["-np"]);

    if (!force_exterior && !force_interior)
    {
        cerr << "Scene type should be specified ! (--exterior or --interior)" << endl;
        return EXIT_FAILURE;
    }

    if(force_exterior && force_interior)
    {
        cerr << "A scene can't be simultaneously interior and exterior !" << endl;
        return EXIT_FAILURE;
    }

    // Retreiving output path
    string outputPath = opt["-o"];
    if(outputPath[outputPath.size() - 1] != '/')
        outputPath.push_back('/');

    // Converting the parameters
    bool   verbose         = op::str2bool(opt["-v"]);
    double voxel_size      = op::str2double(opt["-vs"]);

    //Load input data
    PrimitiveSet myPrimitiveSet;
    LineSet myLineSet;
    loadInputFromJson(opt["-i"], myLineSet, myPrimitiveSet, maxNumberPlanes, verbose);

    //Orientate the planes
    int nbOfReorientation = myPrimitiveSet.orientatePrimitives(myLineSet);
    if (verbose) cout << "Nb of reoriented planes : " << nbOfReorientation << endl;

    //Make the plane arrangement, fill it and save it
    double bboxIncreaseFactor = 0.1;
    PlaneArrangement myArrangement(myLineSet.getBbox(bboxIncreaseFactor), myPrimitiveSet, voxel_size, verbose);

    //Compute the weights
    computeWeights(myArrangement, myLineSet, 1., verbose);

    // Loop on the parameters
    bool force_filled_bounding_box = force_interior;
    bool force_empty_bounding_box = force_exterior;
    while(true)
    {
        double cost_primitive = 1.; // Reference
        double cost_visibility = 0.1;
        double cost_area       = 0.;
        double cost_edge       = 0.01;
        double cost_corners    = 0.01;
        cout << "Enter primitive cost (usually 1) : " << endl;
        cin >> cost_primitive;
        cout << "Enter visibility cost : " << endl;
        cin >> cost_visibility;
        cout << "Enter area cost : " << endl;
        cin >> cost_area;
        cout << "Enter edge cost : " << endl;
        cin >> cost_edge;
        cout << "Enter corner cost : " << endl;
        cin >> cost_corners;

        auto *mosekOptimizer = new ArrangementToMosek(myArrangement, cost_primitive,
                                                      cost_visibility, cost_area, cost_edge, cost_corners,
                                                      force_filled_bounding_box,
                                                      force_empty_bounding_box, verbose);

        auto optResults = mosekOptimizer->setAndSolve();

        std::ostringstream strs;
        strs << "cp_" << cost_primitive << "_cv_" << cost_visibility << "_ca_" << cost_area << "_ce_" <<
             cost_edge << "_cc_" << cost_corners;
        std::string file_title = strs.str();

        cout << "||| saving LP" << endl;
        cout << "out folder : " << outputPath + file_title + ".ply" << endl;
        myArrangement.savePlyFromLabel(outputPath + file_title + ".ply", optResults.second, optResults.first);

        delete mosekOptimizer;

        cout << "Would you like to quit ? (y/n)" << endl;
        string isQuitting;
        cin >> isQuitting;
        if(isQuitting == "y") break;
    }

    return 0;
}
