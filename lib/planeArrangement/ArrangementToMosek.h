#ifndef LINE_BASED_RECONS_REFACTO_ARRANGEMENTTOMOSEK_H
#define LINE_BASED_RECONS_REFACTO_ARRANGEMENTTOMOSEK_H

#include "fusion.h"
#include "PlaneArrangement.h"

using namespace mosek::fusion;
using namespace monty;

class ArrangementToMosek : private Model
{
public:
    ArrangementToMosek(const PlaneArrangement &_arrangement, double cost_primitive, double cost_visibility,
                       double cost_area, double cost_edge, double cost_corner, bool _force_filled_bbox,
                       bool _force_empty_bbox, bool _verbose = false);
    ~ArrangementToMosek();

    // Setting the mosek problem and solving it
    std::pair<std::vector<size_t>, std::map<int, int>> setAndSolve();

    // Getters
    int getNu(PlaneArrangement::Face_handle cell_handle);

protected:

    // Methods to compute the different terms of the linear program
    void computeBoundaryTerm();
    void computeTexturialTerm();
    void computeStructuralTerm();
    void computeVisibilityAndAreaTerms();
    void computeEdgeTerm();
    void computeCornerTerm();

    // Method to compute the nu sign for each cell, which is used in the edge and corner terms computation
    void computeNu();

    // Debug
    void checkCellFromInlierPoint(Point query, std::shared_ptr<ndarray<double,1>> sol, std::vector<double> weightsCopy);
    void printEachTermMagnitude(std::vector<double> weightsCopy);

private:

    // Variables and Constraints
    int nbVariables;
    int nbConstraints;
    ndarray<double, 1> *variablesWeights;
    Variable::t* variables;
    std::vector<std::vector<std::pair<size_t, double>>> constraintsCoorAndWeights;
    std::vector<double> constraintsUpperBounds;

    // Mappings between the plane arrangement and the linear program's variables
    std::map<int, int> cellToVariable;
    std::map<int, int> variableToCell;
    std::map<std::pair<int, int>, int> texturialPairToVariable;
    std::map<int, std::pair<int, int>> variableToTexturialPair;
    std::map<TripletCell, int> structuralTripletToVariable;
    std::map<int, TripletCell> variableToStructuralTriplet;
    std::map<int, int> facetVisToVariable;
    std::map<int, int> variableToFacetVis;
    std::map<int, int> facetAreaToVariable;
    std::map<int, int> variableTofacetArea;
    std::map<int, int> edgeToVariable;
    std::map<int, int> variableToEdge;
    std::map<int, int> verticeToVariable;
    std::map<int, int> variableToVertice;
    std::map<std::pair<int, int>, int> neutralCellsToVariable;
    std::map<int, std::pair<int, int>> variableToNeutralCells;

    // A mapping between the plane arrangement's cells and the nu sign
    std::map<PlaneArrangement::Face_handle, int> cellToNu;

    // Arrangement
    const PlaneArrangement& arrangement;

    // Bounding box forcing conditions
    bool force_filled_bbox;
    bool force_empty_bbox;

    // Cost for each term
    const double cp;
    const double cv;
    const double ca;
    const double ce;
    const double cc;

    // Verbosity trigger
    const bool verbose;
};


#endif //LINE_BASED_RECONS_REFACTO_ARRANGEMENTTOMOSEK_H
